"""Launch the two DH116 owners with only their installed raw-socket capability."""
import os,struct
from pathlib import Path

def native_command(binary,args):
    binary=Path(binary)
    try:
        caps=os.getxattr(binary,'security.capability')
        flags,permitted,inheritable=struct.unpack('<III',caps[:12])
        ready=bool(flags&1 and permitted&(1<<13))  # CAP_NET_RAW, effective
    except (OSError,ValueError,struct.error):ready=False
    if ready:return str(binary),list(args)
    # Keep the original, authenticated path usable until optional setup is approved,
    # and after a rebuild has removed file capabilities. Never retain credentials.
    return '/usr/bin/pkexec',['--keep-cwd',str(binary),*args]

def connection_error(detail):
    if 'identity' in detail.lower() or 'SII vendor/product/revision' in detail:return '设备身份检查失败，请核对网口和 DH116 右手型号。'
    if 'Missing joint/tactile feedback' in detail:return '未收到完整六轴和五指触觉反馈，请检查手部供电与网线。'
    if 'Joint/tactile feedback lost' in detail:return '六轴或触觉反馈中断，已停止本次连接。'
    if 'Homing required' in detail:return '六轴尚未全部归零。请连接 DH116，点击“强制归零”，完成后再使能关节。'
    if 'homing feedback not confirmed' in detail:return '未收到本次六轴归零完成的反馈，已请求停机。'
    if 'Axis disabled or faulted during homing' in detail:return '归零时关节去使能或报告故障，已请求停机。'
    if 'Normal force limit reached' in detail:return '指尖受力达到方案上限，已请求停机。'
    if 'INIT' in detail and ('takeover' in detail or 'Expected one slave' in detail):
        return '灵巧手总线未空闲：请先在官方 LHandPro 或其他手控程序中断开连接，再连接 V3。若已断开，请检查网线和手部电源。'
    if 'lock' in detail.lower():return '已有 V3 灵巧手连接正在使用总线，请先断开该会话。'
    if 'socket' in detail.lower():return '无法打开 EtherCAT 网口，请检查有线网口及 DH116 网口权限。'
    return detail
