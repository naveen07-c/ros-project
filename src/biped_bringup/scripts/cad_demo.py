#!/usr/bin/env python3
"""Generate an IK animation from the actual CAD-visual URDF and a stair world.

Deliberately kinematic: the demo world has no Physics system. Original ROS
controllers and physics launch files remain separate.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro

STAIR_START = 0.66
TREAD = 0.36
RISE = 0.04
COUNT = 4
DT = 1 / 60
STEP_TIME = 2.4
BASE_HEIGHT = 0.53


def smooth(t):
    t = max(0., min(1., t))
    return t*t*t*(10 + t*(-15 + 6*t))


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    m = np.eye(4)
    m[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    m[:3, 3] = xyz
    return m


def values(element, key):
    return [float(x) for x in element.get(key, '0 0 0').split()] if element is not None else [0., 0., 0.]


def pose(m):
    return [*m[:3, 3], math.atan2(m[2, 1], m[2, 2]),
            math.atan2(-m[2, 0], math.hypot(m[0, 0], m[1, 0])), math.atan2(m[1, 0], m[0, 0])]


def solve(urdf, base, feet, sole_heights=None):
    """Solve the existing co-located hip/ankle roll rig, preserving flat soles."""
    sole_heights = sole_heights or {'left': .15, 'right': .15}
    joints = {j.get('name'): j for j in urdf.findall('joint')}
    q = {}
    for side, foot in zip(('left', 'right'), feet):
        hip_offset = values(joints[side+'_hip_roll'].find('origin'), 'xyz')
        a = abs(values(joints[side+'_knee'].find('origin'), 'xyz')[2])
        b = abs(values(joints[side+'_ankle_pitch'].find('origin'), 'xyz')[2])
        ankle_height = sole_heights[side]
        hip = np.array(base) + hip_offset
        ankle = np.array(foot) + [0, 0, ankle_height]
        dx, dy, dz = ankle - hip
        span = math.hypot(dy, dz)
        distance = math.hypot(dx, span)
        if not abs(a-b) < distance < a+b:
            raise ValueError('Unreachable leg target: '+str(distance))
        roll = math.atan2(dy, -dz)
        knee = math.acos((distance*distance-a*a-b*b)/(2*a*b))
        pitch = math.atan2(-dx, span)-math.atan2(b*math.sin(knee), a+b*math.cos(knee))
        angles = (roll, pitch, knee, -pitch-knee, -roll)
        for suffix, angle in zip(('hip_roll', 'hip_pitch', 'knee', 'ankle_pitch', 'ankle_roll'), angles):
            name = side+'_'+suffix
            limit = joints[name].find('limit')
            if not float(limit.get('lower'))-1e-8 <= angle <= float(limit.get('upper'))+1e-8:
                raise ValueError(f'{name} outside limits: {math.degrees(angle):.2f} degrees')
            q[name] = angle
    matrices = {'base_link': transform(base)}
    pending = list(joints.values())
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in matrices:
                continue
            origin = joint.find('origin')
            local = transform(values(origin, 'xyz'), values(origin, 'rpy'))
            axis = values(joint.find('axis'), 'xyz')
            angle = q.get(joint.get('name'), 0.)
            local = local @ transform(rpy=[angle*v for v in axis])
            matrices[joint.find('child').get('link')] = matrices[parent] @ local
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError('Disconnected URDF tree')
    for side, foot in zip(('left', 'right'), feet):
        actual = matrices[side+'_foot'] @ np.array([0, 0, -sole_heights[side], 1])
        if not np.allclose(actual[:3], foot, atol=1e-9):
            raise ValueError('Foot IK/FK mismatch')
        if not np.allclose(matrices[side+'_foot'][:3,:3], np.eye(3), atol=1e-9):
            raise ValueError('Sole is not level')
    return matrices, q


def motion():
    feet = [np.array([0., .1, 0.]), np.array([0., -.1, 0.])]
    for _ in range(120):
        yield [0., 0., BASE_HEIGHT], [f.copy() for f in feet], 'START'
    targets = [(x, 0.) for x in (.12, .24, .36, .48)]
    targets += [(STAIR_START+(i+.5)*TREAD, (i+1)*RISE) for i in range(COUNT)]
    targets += [(2.16, COUNT*RISE), (2.40, COUNT*RISE)]
    for waypoint, (x, z) in enumerate(targets):
        # Alternate which leg leads on successive treads.
        for moving in (waypoint % 2, 1-waypoint % 2):
            start = feet[moving].copy()
            end = np.array([x, start[1], z])
            stationary = feet[1-moving].copy()
            body_start = (start + stationary)/2
            body_end = (end + stationary)/2
            crest = max(start[2], end[2]) + .02
            for i in range(1, round(STEP_TIME/DT)+1):
                u = i / round(STEP_TIME/DT)
                travel = smooth((u-.2)/.6)
                swing = start + (end-start)*travel
                if u < .2:
                    swing[2] = start[2]+(crest-start[2])*smooth(u/.2)
                elif u < .8:
                    swing[2] = crest
                else:
                    swing[2] = crest+(end[2]-crest)*smooth((u-.8)/.2)
                feet[moving] = swing
                body = body_start+(body_end-body_start)*travel
                body[1] = stationary[1] * .08 * math.sin(math.pi*u)**2
                body[2] += BASE_HEIGHT
                yield body, [f.copy() for f in feet], 'CLIMB' if z > 0 else 'WALK'
            feet[moving] = end
    for _ in range(120):
        yield [2.40, 0., BASE_HEIGHT+COUNT*RISE], [f.copy() for f in feet], 'COMPLETE'


def generate(description_share, bringup_share, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    urdf = ET.fromstring(xacro.process_file(str(Path(description_share)/'urdf/biped.urdf.xacro'), mappings={'with_payload': 'false'}).toxml())
    tree = ET.parse(Path(bringup_share)/'worlds/stairs_world.sdf')
    world = tree.getroot().find('world')
    world.find('physics/max_step_size').text = str(DT)
    world.find('physics/real_time_update_rate').text = '60'
    # Use the same physical environment, but explicitly remove physics for animation.
    for plugin in list(world.findall('plugin')):
        if plugin.get('name') == 'gz::sim::systems::Physics':
            world.remove(plugin)
    model = ET.SubElement(world, 'model', name='cad_walking_demo')
    visual_links = [link for link in urdf.findall('link') if link.find('visual') is not None]
    manifest = json.loads((Path(description_share)/'meshes/cad/manifest.json').read_text())
    sole_heights = {side: -manifest['meshes'][side+'_foot']['bounds_m'][0][2] for side in ('left', 'right')}
    rows, extrema = [], {}
    for index, (base, feet, phase) in enumerate(motion()):
        matrices, angles = solve(urdf, base, feet, sole_heights)
        for name, angle in angles.items():
            low, high = extrema.get(name, (angle, angle))
            extrema[name] = min(low, angle), max(high, angle)
        rows.append([index*DT, *[v for link in visual_links for v in pose(matrices[link.get('name')])]])
        if index == 0:
            for source in visual_links:
                link = ET.SubElement(model, 'link', name=source.get('name'))
                ET.SubElement(link, 'pose').text = ' '.join(map(str, pose(matrices[source.get('name')])))
                ET.SubElement(link, 'gravity').text = 'false'
                for src_visual in source.findall('visual'):
                    visual = ET.SubElement(link, 'visual', name=src_visual.get('name'))
                    uri = src_visual.find('geometry/mesh').get('filename')
                    mesh_path = Path(description_share) / uri.split('package://biped_description/')[1]
                    if not mesh_path.is_file():
                        raise FileNotFoundError(mesh_path)
                    geometry = ET.SubElement(visual, 'geometry')
                    mesh = ET.SubElement(geometry, 'mesh')
                    ET.SubElement(mesh, 'uri').text = str(mesh_path)
                    material = ET.SubElement(visual, 'material')
                    for field in ('ambient', 'diffuse'):
                        ET.SubElement(material, field).text = '0.792157 0.819608 0.933333 1'
    trajectory = output/'cad_walk.frames'
    with trajectory.open('w') as stream:
        stream.write(' '.join(link.get('name') for link in visual_links)+'\n')
        for row in rows:
            stream.write(' '.join(f'{v:.9f}' for v in row)+'\n')
    plugin = ET.SubElement(model, 'plugin', filename='libbiped_cad_playback.so', name='biped::CadPlayback')
    ET.SubElement(plugin, 'trajectory').text = str(trajectory)
    world_path = output/'cad_stairs_demo.sdf'
    ET.indent(tree)
    tree.write(world_path, encoding='unicode', xml_declaration=True)
    report = {'mode': 'kinematic demonstration; no dynamics or balance validation',
              'frames': len(rows), 'duration_s': rows[-1][0], 'stairs': COUNT,
              'rise_m': RISE, 'tread_m': TREAD, 'final_sole_height_m': COUNT*RISE,
              'final_x_m': 2.4, 'joint_ranges_deg': {k: [math.degrees(v) for v in vs] for k, vs in extrema.items()}}
    (output/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    return str(world_path)


if __name__ == '__main__':
    import argparse
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(generate(get_package_share_directory('biped_description'), get_package_share_directory('biped_bringup'), args.output))
