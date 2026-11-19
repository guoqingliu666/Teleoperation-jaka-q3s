"""Import the supplied 11.13 DH116 placement; offline only, never calls ROS/SDK.

Modern stream framing: https://github.com/XRTC5/sldprt-export (MIT).
Only the CRC-checked component XML is read, not guessed binary coordinates.
The saved placement is independently compared with the prior native COM export.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET
import zlib
import numpy as np
from scipy.spatial.transform import Rotation
from prepare_assets import origin, transform

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path('/home/gq/projects_lgq/JAKA-S5_开发资料_2026-09-07/整合模型/（11.13）手臂.SLDASM')
OLD = ROOT.parent/'V2_servo/digital_twin/cad/derived/20261027_hand_mount'
REF = ROOT/'reference/hand_mount_20261113'
URDF = ROOT/'src/v3_moveit/description/s5.urdf'
CONFIG = ROOT/'src/v3_moveit/config/hand_mount.json'
SOURCE_SHA = 'a5649dd29e39537e366f18b78c47040aee90ba0237a09fab74dcb5d54b96bc22'
BEFORE_SHA = '0f3dbc8509259cbbad0c1eef1a3ea68e7045e702a199ad93b02fb2c2bfdbd551'
HAND = 'DH116-R000-A4-灵巧右手-2'
FLANGE = 'JAKA S 5机械臂.STEP-1/S5 06.STEP-1'
NS = {'s': 'http://www.solidworks.com/sw2003/schema'}


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def require(ok, message):
    if not ok:
        raise ValueError(message)


def native_tree(path):
    data = Path(path).read_bytes()
    signature = b'\x14\x00\x06\x00\x08\x00'
    swap = bytes(((v << 4) | (v >> 4)) & 255 for v in range(256))
    found = []
    start = 0
    while True:
        at = data.find(signature, start)
        if at < 0:
            break
        start = at + len(signature)
        if at + 26 > len(data):
            continue
        _, crc, compressed, expanded, name_size = struct.unpack_from('<5I', data, at + 6)
        if name_size > 4096 or at + 26 + name_size + compressed > len(data):
            continue
        name = data[at + 26:at + 26 + name_size].translate(swap)
        if name != b'swXmlContents/COMPINSTANCETREE':
            continue
        require(expanded < 16 * 1024 * 1024, 'Component XML exceeds import limit')
        begin = at + 26 + name_size
        decoder = zlib.decompressobj(-15)
        xml = decoder.decompress(data[begin:begin + compressed], expanded + 1)
        require(decoder.eof and not decoder.unused_data and len(xml) == expanded,
                'Component XML stream size mismatch')
        require(zlib.crc32(xml) == crc, 'Component XML CRC mismatch')
        found.append(xml)
    require(len(found) == 1, 'Expected one unambiguous saved component tree')
    return found[0]


def placements(xml):
    root = ET.fromstring(xml)
    models = {m.get('id'): m for m in root.find('s:swModelList', NS)}
    configs = list(root.find('s:swConfigurationList', NS))
    active = [c for c in configs if c.get('swMostRecentConfiguration') == 'YES']
    require(len(active) == 1 and active[0].get('swConfigurationNeedsUpdate') == 'NO',
            'Saved configuration is ambiguous or needs rebuild')
    out = {}
    def walk(model, parent, prefix, stack):
        require(model.get('id') not in stack, 'Cyclic assembly reference')
        stack = stack | {model.get('id')}
        for ref in model.findall('s:swReference', NS):
            require(ref.get('swSuppressed') == 'NO' and ref.get('swHidden') == 'NO',
                    'Suppressed or hidden component requires explicit handling')
            values = np.fromstring(ref.get('swTransform', ''), sep=' ')
            require(len(values) == 16 and np.isfinite(values).all(), 'Invalid native transform')
            local = values.reshape(4, 4).T
            require(np.allclose(local[3], [0, 0, 0, 1], atol=1e-12, rtol=0) and
                    np.allclose(local[:3, :3].T @ local[:3, :3], np.eye(3), atol=1e-10, rtol=0) and
                    abs(np.linalg.det(local[:3, :3]) - 1) < 1e-10, 'Non-rigid native transform')
            name = prefix + ref.get('swName') + '-' + ref.get('swReferenceNumber')
            require(name not in out, 'Duplicate component path')
            out[name] = parent @ local
            walk(models[ref.get('swModelRef')], out[name], name + '/', stack)
    walk(models[active[0].get('swModelRef')], np.eye(4), '', set())
    return out


def affected(element):
    name = element.get('name', '')
    return (element.tag in ('visual', 'collision') and
            (name == 'DH_base_link_visual' or name.startswith('DH_static_') or
             name in {f'DH_finger{i}_sweep' for i in range(1, 6)}))


def derive():
    require(sha(REF/'source.SLDASM') == SOURCE_SHA, '11.13 source changed')
    require(sha(REF/'before_s5.urdf') == BEFORE_SHA, 'Baseline V3 model changed')
    xml = native_tree(REF/'source.SLDASM')
    require(xml == (REF/'new_components.xml').read_bytes(), 'Saved XML differs from native source')
    old = placements((REF/'old_components.xml').read_bytes())
    new = placements(xml)
    exported = json.loads((REF/'before_components.json').read_text(encoding='utf-8-sig'))
    rows = exported['components']
    require(set(old) == set(new) == {c['name'] for c in rows} and len(new) == 66,
            'Assembly component identities changed')
    for c in rows:
        values = np.array(c['total_transform'])
        require(values.shape == (16,) and abs(values[12] - 1) < 1e-12, 'Invalid COM transform')
        expected = transform(values[9:12], values[:9].reshape(3, 3).T)
        require(np.allclose(old[c['name']], expected, atol=1e-12, rtol=0),
                'Native XML differs from independent COM export: ' + c['name'])
    changed = [name for name in old if not np.allclose(old[name], new[name], atol=1e-9, rtol=0)]
    require(changed == [HAND], 'This update supports only the observed hand placement change')
    baseline = ET.fromstring((REF/'before_s5.urdf').read_bytes())
    palm = origin(baseline.find("link[@name='Link_06']/visual[@name='DH_base_link_visual']/origin"))
    assembly = json.loads((REF/'before_assembly.json').read_text())
    old_mount = transform(assembly['mount_xyz'], Rotation.from_euler('xyz', assembly['mount_rpy']).as_matrix())
    sdk_from_nominal = palm @ np.linalg.inv(old_mount)
    sdk_from_cad = sdk_from_nominal @ np.array(assembly['registration']['cad_flange_to_urdf_link06'])
    hand_from_a1 = np.array(assembly['registration']['hand_urdf_to_cad'])
    old_hand = sdk_from_cad @ np.linalg.inv(old[FLANGE]) @ old[HAND]
    new_hand = sdk_from_cad @ np.linalg.inv(new[FLANGE]) @ new[HAND]
    require(np.allclose(old_hand @ hand_from_a1, palm, atol=1e-12, rtol=0),
            'Existing palm does not match prior CAD registration')
    delta = new_hand @ np.linalg.inv(old_hand)
    result = copy.deepcopy(baseline)
    records = []
    for element in result.find("link[@name='Link_06']"):
        if not affected(element):
            continue
        node = element.find('origin')
        before = origin(node)
        after = delta @ before
        node.set('xyz', ' '.join(format(x, '.16g') for x in after[:3, 3]))
        node.set('rpy', ' '.join(format(x, '.16g') for x in Rotation.from_matrix(after[:3, :3]).as_euler('xyz')))
        records.append({'kind': element.tag, 'name': element.get('name'), 'before': before.tolist(), 'after': after.tolist()})
    require(sum(r['kind'] == 'visual' for r in records) == 17 and
            sum(r['kind'] == 'collision' for r in records) == 6, 'Hand geometry count changed')
    ET.indent(result)
    content = ET.tostring(result, encoding='utf-8', xml_declaration=True)
    report = dict(schema=1, source_file=str(SOURCE), source_snapshot='reference/hand_mount_20261113/source.SLDASM',
                  source_sha256=SOURCE_SHA, baseline_urdf_sha256=BEFORE_SHA,
                  component_count=66, changed_components=changed, unchanged_components=65,
                  native_xml_crc_verified=True, old_native_matches_independent_COM_export=True,
                  sdk_link06_delta=delta.tolist(), palm_mount_before=palm.tolist(),
                  palm_mount_after=(new_hand @ hand_from_a1).tolist(),
                  rotation_change_deg=float(np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude())),
                  updated_elements=records, control_urdf_sha256=hashlib.sha256(content).hexdigest(),
                  preserved=['arm kinematics', 'TCP', 'quick-change A/B', 'connecting flange', 'mount adapter',
                             'mesh geometry', 'hand joint limits and couplings', 'collision envelope sizes'],
                  limitations=['Reuses prior CAD-to-arm and A4-to-A1 surface registrations; not physical calibration.',
                               'A1 articulated visualization retained; this updates the A4 assembly installation pose.',
                               'Payload/CoG and live physical checks require fresh acquisition; no hardware calls made.'])
    report['evidence_sha256'] = {str(p.relative_to(ROOT)): sha(p) for p in sorted(REF.iterdir()) if p.is_file()}
    return content, report


def main():
    parser = argparse.ArgumentParser();parser.add_argument('--apply', action='store_true');args = parser.parse_args()
    if args.apply:
        require(sha(SOURCE) == SOURCE_SHA, 'Unexpected source file')
        require(sha(URDF) == BEFORE_SHA, 'Only the audited baseline can be updated')
        REF.mkdir(parents=True, exist_ok=True)
        copies = {SOURCE: REF/'source.SLDASM', URDF: REF/'before_s5.urdf',
                  OLD/'components_complete.json': REF/'before_components.json',
                  OLD.parent/'complete_end_effector/assembly.json': REF/'before_assembly.json'}
        for source, target in copies.items():
            require(not target.exists() or sha(target) == sha(source), 'Existing baseline differs')
            if not target.exists():shutil.copy2(source, target)
        (REF/'new_components.xml').write_bytes(native_tree(SOURCE))
        (REF/'old_components.xml').write_bytes(native_tree(OLD/'working_copy/（10.27）手臂.SLDASM'))
    content, report = derive()
    if args.apply:
        URDF.write_bytes(content)
        CONFIG.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    else:
        require(URDF.read_bytes() == content, 'Active model differs from derived 11.13 placement')
        require(json.loads(CONFIG.read_text()) == report, 'Hand mount provenance differs')
    print(json.dumps({k: report[k] for k in ('source_file', 'rotation_change_deg', 'unchanged_components', 'control_urdf_sha256')}, ensure_ascii=False))


if __name__ == '__main__':main()
