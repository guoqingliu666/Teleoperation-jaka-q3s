"""Export the existing display URDF and an exact, read-only mesh allowlist."""
import hashlib
import json
import math
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def vector(text, size, default):
    value = [float(v) for v in text.split()] if text else default
    if len(value) != size or not all(math.isfinite(v) for v in value):
        raise ValueError('Invalid display geometry vector')
    return value


def origin(element):
    return dict(xyz=vector(element.get('xyz') if element is not None else None, 3, [0, 0, 0]),
                rpy=vector(element.get('rpy') if element is not None else None, 3, [0, 0, 0]))


def export_model(root=ROOT):
    path = root / '.cache/operator/display.urdf'
    robot = ET.parse(path).getroot()
    assets, links, joints = {}, [], []
    for link in robot.findall('link'):
        visuals = []
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            shape = next(iter(geometry))
            attributes = dict(shape.attrib)
            if shape.tag == 'mesh':
                uri = attributes.pop('filename')
                if uri.startswith('package://v3_moveit/'):
                    mesh = root / 'src/v3_moveit' / uri.removeprefix('package://v3_moveit/')
                elif uri.startswith('file://'):
                    mesh = Path(unquote(urlparse(uri).path))
                else:
                    raise ValueError('Unsupported mesh URI: ' + uri)
                mesh = mesh.resolve(strict=True)
                suffix = mesh.suffix.lower()
                if suffix not in ('.stl', '.obj'):
                    raise ValueError('Unsupported mesh type')
                # Only files actually named by the local display URDF are served.
                key = hashlib.sha256(mesh.read_bytes()).hexdigest() + suffix
                assets[key] = mesh
                attributes['url'] = '/xr/mesh/' + key
                attributes['scale'] = vector(attributes.get('scale'), 3, [1, 1, 1])
            elif shape.tag == 'box':
                attributes['size'] = vector(attributes['size'], 3, [])
            elif shape.tag in ('cylinder', 'sphere'):
                attributes = {k: float(v) for k, v in attributes.items()}
            else:
                raise ValueError('Unsupported display geometry: ' + shape.tag)
            color = visual.find('material/color')
            visuals.append(dict(origin=origin(visual.find('origin')), kind=shape.tag,
                                geometry=attributes,
                                rgba=vector(color.get('rgba') if color is not None else None,
                                            4, [.65, .72, .78, 1])))
        links.append(dict(name=link.get('name'), visuals=visuals))
    for joint in robot.findall('joint'):
        axis = joint.find('axis')
        mimic = joint.find('mimic')
        item = dict(name=joint.get('name'), type=joint.get('type'),
                    parent=joint.find('parent').get('link'), child=joint.find('child').get('link'),
                    origin=origin(joint.find('origin')),
                    axis=vector(axis.get('xyz') if axis is not None else None, 3, [1, 0, 0]))
        if mimic is not None:
            item['mimic'] = dict(joint=mimic.get('joint'), multiplier=float(mimic.get('multiplier', '1')),
                                 offset=float(mimic.get('offset', '0')))
        joints.append(item)
    metadata = json.loads((root / 'src/v3_moveit/config/model_metadata.json').read_text())
    return dict(schema=1, units='metres', frame='world', links=links, joints=joints,
                initial_joints={'joint_' + str(i + 1): v for i, v in enumerate(metadata['q_reference_rad'])},
                model_sha256=hashlib.sha256(path.read_bytes()).hexdigest()), assets
