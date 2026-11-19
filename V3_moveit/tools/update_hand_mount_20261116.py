"""Derive the shared control/display placement from the supplied 11.16 assembly.

Offline only. Reuses the audited meshes and CAD registration; reads CRC-checked
native component transforms, never treats the assembly as executable content.
"""
import argparse
import copy
import json
import shutil
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from prepare_assets import origin, set_origin
from update_hand_mount_20261113 import (ROOT, HAND, FLANGE, native_tree, placements,
                                      sha, require, derive as prior_derive, REF as PRIOR)

SOURCE = ROOT/'JAKAS5-Hand-model-SW/（11.16）手臂.SLDASM'
SOURCE_SHA = 'da5aeb91e873c3f5b8d7c45e20db93c2c11c882b2f58901640e662ad62d82635'
REF = ROOT/'reference/hand_mount_20261116'
URDF = ROOT/'src/v3_moveit/description/s5.urdf'
CONFIG = ROOT/'src/v3_moveit/config/hand_mount.json'

def affected(element):
    return element.tag in ('visual', 'collision') and element.get('name', '').startswith('DH_')

def derive():
    require(sha(SOURCE) == sha(REF/'source.SLDASM') == SOURCE_SHA, '11.16 source identity changed')
    baseline, prior = prior_derive()
    require((REF/'before_s5.urdf').read_bytes() == baseline, '11.13 baseline differs')
    xml = native_tree(SOURCE)
    require(xml == (REF/'new_components.xml').read_bytes(), 'Native component XML differs')
    old = placements((PRIOR/'new_components.xml').read_bytes()); new = placements(xml)
    require(set(old) == set(new) and len(new) == 66, 'Assembly component identities changed')
    changed = [n for n in old if not np.allclose(old[n], new[n], atol=1e-9, rtol=0)]
    require(len(changed) == 58 and all(not n.startswith('JAKA ') for n in changed), 'Unexpected arm/attachment change')
    assembly_delta = new[HAND] @ np.linalg.inv(old[HAND])
    require(all(np.allclose(new[n] @ np.linalg.inv(old[n]), assembly_delta, atol=1e-8, rtol=0)
                for n in changed), 'Attachments no longer share one rigid installation change')
    require(np.allclose(old[FLANGE], new[FLANGE], atol=1e-9, rtol=0), 'Robot flange changed')
    registration = json.loads((PRIOR/'before_assembly.json').read_text())['registration']
    sdk_from_world = np.array(prior['palm_mount_after']) @ np.linalg.inv(registration['hand_urdf_to_cad']) @ np.linalg.inv(old[HAND])
    delta = sdk_from_world @ assembly_delta @ np.linalg.inv(sdk_from_world)
    result = ET.fromstring(baseline); records = []
    for element in result.find("link[@name='Link_06']"):
        if affected(element):
            before = origin(element.find('origin')); after = delta @ before
            node=element.find('origin')
            node.set('xyz',' '.join(format(v,'.16g') for v in after[:3,3]))
            node.set('rpy',' '.join(format(v,'.16g') for v in Rotation.from_matrix(after[:3,:3]).as_euler('xyz')))
            records.append(dict(kind=element.tag, name=element.get('name'), before=before.tolist(), after=after.tolist()))
    require(len(records) == 31, 'Unexpected attachment geometry count')
    ET.indent(result); content = ET.tostring(result, encoding='utf-8', xml_declaration=True)
    import hashlib
    report = dict(schema=2, source_file=str(SOURCE.relative_to(ROOT)), source_snapshot=str((REF/'source.SLDASM').relative_to(ROOT)),
                  source_sha256=SOURCE_SHA, baseline_urdf_sha256=sha(REF/'before_s5.urdf'), component_count=66,
                  changed_components=changed, unchanged_components=8, native_xml_crc_verified=True,
                  sdk_link06_delta=delta.tolist(), rotation_change_deg=float(np.rad2deg(Rotation.from_matrix(delta[:3,:3]).magnitude())),
                  updated_elements=records, control_urdf_sha256=hashlib.sha256(content).hexdigest(),
                  limitations=['Reuses audited mesh geometry and prior CAD registration; not a new surface export or physical calibration.',
                               'Articulated hand uses manufacturer A1 meshes aligned to the A4 assembly placement.',
                               'Payload and centroid must be read from the calibrated controller; no hardware calls made.'])
    return content, report

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--apply', action='store_true'); args=parser.parse_args()
    if args.apply:
        baseline, _ = prior_derive()
        require(URDF.read_bytes() == baseline or REF.exists(), 'Unexpected active model')
        REF.mkdir(parents=True, exist_ok=True)
        for path, data in [(REF/'source.SLDASM', SOURCE.read_bytes()), (REF/'before_s5.urdf', baseline),
                           (REF/'new_components.xml', native_tree(SOURCE))]:
            require(not path.exists() or path.read_bytes() == data, 'Existing evidence differs')
            if not path.exists(): path.write_bytes(data)
    content, report = derive()
    if args.apply:
        URDF.write_bytes(content); CONFIG.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    else:
        require(URDF.read_bytes() == content and json.loads(CONFIG.read_text()) == report, 'Active model/provenance differs')
    print(json.dumps({k: report[k] for k in ('source_file','rotation_change_deg','unchanged_components','control_urdf_sha256')}, ensure_ascii=False))

if __name__ == '__main__': main()
