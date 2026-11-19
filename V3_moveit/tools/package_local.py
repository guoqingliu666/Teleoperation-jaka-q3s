"""打包本地 Ubuntu 源码交付，不含环境、缓存、私钥或虚构的 ROS 通过记录。"""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, ZipInfo, ZIP_DEFLATED
from verify_offline import ROOT, source_digest


def main():
    files=[]
    for dirname in ('src','vendor','reference','tools','quest'):
        files.extend(p for p in (ROOT/dirname).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    files.extend(p for p in ROOT.iterdir() if p.is_file())
    files.extend(p for p in (ROOT/'artifacts').glob('*') if p.suffix in ('.json','.txt') and not p.name.startswith('ros_smoke_pass'))
    files=sorted(set(files));manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    dist=ROOT/'dist';dist.mkdir(exist_ok=True)
    target=dist/'V3_moveit_Ubuntu24.04_20261112.zip'
    with ZipFile(target,'w',ZIP_DEFLATED,compresslevel=6) as archive:
        for path in files:
            name='V3_moveit/'+path.relative_to(ROOT).as_posix()
            info=ZipInfo(name);info.compress_type=ZIP_DEFLATED
            executable=path.suffix=='.sh' or path.parent==ROOT/'src/v3_moveit/scripts'
            info.external_attr=((0o100755 if executable else 0o100644)<<16)
            archive.writestr(info,path.read_bytes())
        archive.writestr('V3_moveit/package_manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
    with ZipFile(target) as archive:
        if archive.testzip() is not None:raise RuntimeError('ZIP CRC check failed')
        for name,digest in manifest.items():
            if hashlib.sha256(archive.read('V3_moveit/'+name)).hexdigest()!=digest:raise RuntimeError('ZIP content mismatch '+name)
    report=dict(archive=str(target),bytes=target.stat().st_size,files=len(manifest),
        sha256=hashlib.sha256(target.read_bytes()).hexdigest(),package_sha256=source_digest(),
        zip_readback_passed=True,ubuntu_build_verified=False,robot_commands=0)
    (dist/'package_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
