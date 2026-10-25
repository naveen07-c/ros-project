#!/usr/bin/env python3
"""Check CAD demo IK, foot clearance, planted feet, and landing geometry."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('cad_demo', ROOT/'src/biped_bringup/scripts/cad_demo.py')
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)
urdf = ET.fromstring(xacro.process_file(str(ROOT/'src/biped_description/urdf/biped.urdf.xacro'), mappings={'with_payload': 'false'}).toxml())
manifest = json.loads((ROOT/'src/biped_description/meshes/cad/manifest.json').read_text())
heights = {side: -manifest['meshes'][side+'_foot']['bounds_m'][0][2] for side in ('left', 'right')}
world = ET.parse(ROOT/'src/biped_bringup/worlds/stairs_world.sdf')
steps = []
for model in world.findall(".//model"):
    if not model.get('name').startswith('stair_'):
        continue
    center = [float(v) for v in model.findtext('pose').split()]
    size = [float(v) for v in model.findtext('link/collision/geometry/box/size').split()]
    steps.append((center[0]-size[0]/2, center[0]+size[0]/2, center[2]+size[2]/2))
assert len(steps) == demo.COUNT
for i, (left, right, top) in enumerate(steps):
    assert abs(left-(demo.STAIR_START+i*demo.TREAD)) < 1e-8
    assert abs(top-(i+1)*demo.RISE) < 1e-8
previous_feet = previous_q = None
max_velocity = 0.
for count, (base, feet, phase) in enumerate(demo.motion(), 1):
    matrices, q = demo.solve(urdf, base, feet, heights)
    grounded = []
    for side, foot in zip(('left', 'right'), feet):
        bounds = manifest['meshes'][side+'_foot']['bounds_m']
        heel, toe = foot[0]+bounds[0][0], foot[0]+bounds[1][0]
        surface = max([0.]+[top for left, right, top in steps if toe > left+1e-8 and heel < right-1e-8])
        assert foot[2] >= surface-1e-8, (count, side, 'foot intersects a riser', foot, surface)
        grounded.append(abs(foot[2]-surface) < 1e-8)
    assert any(grounded), (count, 'both feet airborne')
    if previous_feet is not None:
        assert any(np.allclose(a, b, atol=1e-10) for a, b in zip(feet, previous_feet)), (count, 'both feet slide')
        for name in q:
            velocity = abs(q[name]-previous_q[name])/demo.DT
            limit = float(urdf.find("joint[@name='"+name+"']/limit").get('velocity'))
            assert velocity <= limit+1e-6, (count, name, velocity)
            max_velocity = max(max_velocity, velocity)
    previous_feet, previous_q = feet, q
assert phase == 'COMPLETE'
assert all(abs(f[2]-.16) < 1e-9 and abs(f[0]-2.4) < 1e-9 for f in feet)
print(f'PASS {count} frames: IK/FK agreement, level soles, joint position/velocity limits')
print(f'PASS actual CAD foot bounds clear all 4 risers; at least one stationary grounded foot throughout')
print(f'PASS both feet finish at x=2.40 m, z=0.16 m; peak joint speed {max_velocity:.3f} rad/s')
print('Kinematic checks only; no force, contact stability, or balance claim.')
