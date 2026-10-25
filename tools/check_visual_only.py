#!/usr/bin/env python3
"""Assert CAD visuals leave every nonvisual URDF and SDF element unchanged."""
import pathlib
import json
import hashlib
import struct
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import xacro

ROOT = pathlib.Path(__file__).resolve().parents[1]
BASELINE = '8d6433f'

def physics(xml):
    root = ET.fromstring(xml)
    def clean(node):
        for child in list(node):
            if child.tag in ('visual', 'material'):
                node.remove(child)
            else:
                clean(child)
        node.text = (node.text or '').strip()
        node.tail = ''
    clean(root)
    return ET.canonicalize(ET.tostring(root, encoding='unicode'))

with tempfile.TemporaryDirectory(prefix='biped-visual-check-') as folder:
    original = pathlib.Path(folder)
    files = subprocess.check_output(['git', '-C', str(ROOT), 'ls-tree', '-r', '--name-only', BASELINE, 'src/biped_description/urdf'], text=True).splitlines()
    for path in files:
        (original / pathlib.Path(path).name).write_bytes(subprocess.check_output(['git', '-C', str(ROOT), 'show', f'{BASELINE}:{path}']))
    for payload in ('true', 'false'):
        models = []
        for path in (original / 'biped.urdf.xacro', ROOT / 'src/biped_description/urdf/biped.urdf.xacro'):
            models.append(xacro.process_file(str(path), mappings={'with_payload': payload}).toxml())
        assert physics(models[0]) == physics(models[1]), f'Nonvisual URDF changed (payload={payload})'
        converted = []
        for i, xml in enumerate(models):
            urdf = original / f'model-{i}.urdf'
            urdf.write_text(xml)
            converted.append(subprocess.check_output(['gz', 'sdf', '-p', str(urdf)], text=True))
        if physics(converted[0]) != physics(converted[1]):
            import difflib
            print('\n'.join(difflib.unified_diff(physics(converted[0]).replace('><', '>\n<').splitlines(), physics(converted[1]).replace('><', '>\n<').splitlines())))
            raise AssertionError(f'Nonvisual SDF changed (payload={payload})')
        print(f'PASS payload={payload}: all joints, inertias, collisions, sensors, control plugins and contact settings identical in URDF and SDF')
    unchanged = subprocess.check_output(['git', '-C', str(ROOT), 'diff', BASELINE, '--', 'src/biped_control', 'src/biped_bringup/worlds/flat_world.sdf'], text=True)
    assert not unchanged, 'Control code, control configuration or original flat world changed'
    print('PASS control sources, controller/gait YAML and original flat world are unchanged')

# Validate the committed CAD assets and their source without CAD libraries.
mesh_dir = ROOT / "src/biped_description/meshes/cad"
manifest = json.loads((mesh_dir / "manifest.json").read_text())
source = (mesh_dir / manifest["source"]).resolve()
assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest["source_sha256"], "Source STEP changed"
instances = []
for name, mesh in manifest["meshes"].items():
    data = (mesh_dir / (name + ".stl")).read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert count == mesh["triangles"] and len(data) == 84 + count * 50, f"Invalid mesh {name}"
    for triangle in struct.iter_unpack("<12fH", data[84:]):
        assert all(math.isfinite(v) for v in triangle[:12]), f"Nonfinite mesh {name}"
    instances.extend(tuple(part["path"]) for part in mesh["parts"])
assert len(instances) == len(set(instances)) == 115, "Missing or duplicated CAD parts"
for visual in ET.fromstring(models[1]).findall(".//visual"):
    filename = visual.find("geometry/mesh").get("filename")
    assert filename.startswith("package://biped_description/meshes/cad/")
    assert (mesh_dir / filename.rsplit("/", 1)[1]).is_file()
for visual in ET.fromstring(converted[1]).findall(".//visual"):
    color = [float(v) for v in visual.findtext("material/diffuse").split()]
    assert all(abs(a-b) < 1e-6 for a,b in zip(color,manifest["cad_colour_srgb"])), "Gazebo color differs from STEP"
print("PASS 115 CAD instances in 10 valid meshes, STEP checksum unchanged, mesh paths resolve, Gazebo color matches STEP")
