"""项目内运行依赖；本机配置不进入公开发布包。"""
import hashlib,json
from pathlib import Path
from .configuration import ROOT

def load_runtime_options():
    options=json.loads((ROOT/'config/teleop.json').read_text(encoding='utf-8'))
    local=ROOT/'config/local/runtime.json'
    # 旧配置中的外部路径仅留作历史溯源。即使遗漏本机覆盖文件，也不悄悄回退 F 盘。
    override={'sdk_directory':'vendor/jaka/Windows/python3/x64'}
    if local.is_file():
        override=json.loads(local.read_text(encoding='utf-8-sig'))
    directory=(ROOT/override['sdk_directory']).resolve()
    if not directory.is_relative_to(ROOT):raise ValueError('Runtime SDK must be installed inside V2')
    options['sdk_directory']=str(directory)
    return options

def sdk_artifacts(directory):
    d=Path(directory)
    paths=[d/'jakaAPI.dll']+[d.parents[1]/'c&c++/inc_of_c'/n for n in ('jakaAPI.h','jktypes.h')]
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}

def same_sdk_artifacts(a,b):
    """只接受同名三项的完全相同字节；允许SDK复制后安装目录改变。"""
    def identity(record):
        if not isinstance(record,dict) or len(record)!=3:return None
        result={str(k).replace('\\','/').rsplit('/',1)[-1]:v for k,v in record.items()}
        if set(result)!={'jakaAPI.dll','jakaAPI.h','jktypes.h'}:return None
        if any(not isinstance(v,str) or len(v)!=64 for v in result.values()):return None
        return result
    aa,bb=identity(a),identity(b)
    return aa is not None and bb is not None and aa==bb
