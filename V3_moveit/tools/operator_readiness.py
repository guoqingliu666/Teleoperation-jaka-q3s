"""Read-only readiness explanation. No robot connection, ROS node or commands."""
import json
from pathlib import Path
from operator_session import ROOT,acceptance_report,saved_acceptance,validate_live_fake,read

def probe():
    from verify_offline import source_digest
    from check_ros_pass import validate_reports
    from hardware_current_preflight import validate as validate_preflight
    digest=source_digest();result=dict(loaded=True)
    def check(name,fn,good):
        try:fn();result[name]=dict(ok=True,detail=good)
        except Exception as e:result[name]=dict(ok=False,detail=str(e))
    check('software',lambda:validate_reports(ROOT,digest),'当前控制版本的软件检查通过')
    check('fake',lambda:validate_live_fake(digest),'当前 Quest 仿真检查通过')
    check('preflight',lambda:validate_preflight(ROOT,digest),'实机状态、TCP、负载及两组反馈检查通过（有效期内）')
    check('hardware',acceptance_report,'真机跟随、松 Grip、摇杆停止及 Servo 退出记录完整')
    if not result['hardware']['ok']:
        try:
            old=read(ROOT/'artifacts/quest_commissioning_hardware/result.json')
            if old.get('failure'):
                failure=old['failure'].replace('Production/observer fault: ','').replace('Observed joint excursion exceeded commissioning exit threshold','关节移动超过该次检查的退出范围')
                result['hardware']['detail']='上次真机检查未通过：'+failure
        except (OSError,ValueError):pass
    check('accepted',saved_acceptance,'本版本的现场开发验收记录已保存')
    if result['accepted']['ok']:
        # Onboarding readiness comes from the validated saved acceptance. Keep
        # the latest standalone diagnostic separately; reruns cannot erase it.
        for name,title in [('fake','Quest 仿真检查'),('hardware','真机跟随与停止检查')]:
            result[name]=dict(ok=True,reused=True,detail=title+'已纳入有效的现场验收，无需重复',latest_check=result[name])
    return result

if __name__=='__main__':print(json.dumps(probe(),ensure_ascii=False))
