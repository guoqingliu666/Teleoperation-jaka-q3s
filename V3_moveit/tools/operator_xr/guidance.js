import * as THREE from 'three';

const states = {
  idle: ['等待遥操作', '在上位机点击启动', 'neutral', 'standby'],
  input: ['等待右手柄', '让头显看到右手柄', 'neutral', 'standby'],
  starting: ['正在准备', '控制器就绪后会在这里提示', 'neutral', 'standby'],
  checking: ['正在检查设备', '等待上位机完成检查', 'neutral', 'standby'],
  waiting_feedback: ['等待控制器反馈', '正在确认遥操作状态', 'neutral', 'standby'],
  ready: ['可以开始遥操作', '握住右侧 Grip，{tool} 跟随', 'green', 'check'],
  following: ['{tool} 跟随中', '松开 Grip 暂停', 'blue', 'follow'],
  engaging: ['正在接入跟随', '等待控制器响应', 'neutral', 'standby'],
  braking: ['正在停止跟随', '等待控制器停稳', 'neutral', 'pause'],
  paused: ['已暂停跟随', '握住右侧 Grip 继续', 'green', 'pause'],
  release: ['Grip 已握住', '松开一次即可就绪', 'amber', 'pause'],
  software_stop: ['软件停止', '松开 Grip 复位', 'amber', 'stop'],
  tracking: ['等待手柄追踪', '让头显看到右手柄', 'amber', 'standby'],
  unavailable: ['状态连接中断', '等待上位机恢复连接', 'amber', 'stop'],
  fault: ['控制器故障', '请查看上位机运行日志', 'red', 'stop'],
  stopping: ['正在结束遥操作', '等待控制器退出', 'neutral', 'standby'],
  ended: ['会话已结束', '可在上位机再次启动', 'neutral', 'standby'],
  failed: ['会话未能完成', '请查看上位机运行日志', 'red', 'stop']
};
const colors = {neutral:'#e0e4ed', green:'#7aefb0', blue:'#91c5ff', amber:'#ffd18a', red:'#ff9f99'};

export function guidanceView(data, elapsed) {
  let phase = data?.phase || 'unavailable';
  if (!Object.hasOwn(states, phase) || elapsed > (data?.ttl_s ?? 1.5)) phase = 'unavailable';
  const [title, hint, tone, icon] = states[phase];
  const tool = data?.tool_id === 0 ? '法兰' : Number.isInteger(data?.tool_id) ? 'TCP'+data.tool_id : '工具中心点';
  const mode = data?.mode === 'hardware' ? '真机' : data?.mode === 'fake' ? '仿真' : '';
  const context = [mode, data?.task === 'commission' ? '开发检查' : ''].filter(Boolean).join(' · ');
  const detail = (data?.instruction && phase === data.phase) ? data.instruction : hint.replace('{tool}', tool);
  return {phase, title:title.replace('{tool}',tool), detail, context, color:colors[tone], icon};
}

export class SpatialGuide {
  constructor() {
    this.canvas = document.createElement('canvas');
    this.canvas.width = 1056; this.canvas.height = 224;
    this.ctx = this.canvas.getContext('2d');
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.mesh = new THREE.Mesh(new THREE.PlaneGeometry(.69,.69*224/1056),
      new THREE.MeshBasicMaterial({map:this.texture, transparent:true, depthTest:false, depthWrite:false, toneMapped:false}));
    this.mesh.visible = false; this.mesh.renderOrder = 160;
    this.pivot = new THREE.Object3D(); this.destination = new THREE.Vector3();
    this.facing = new THREE.Quaternion(); this.relative = new THREE.Vector3();
    this.headingInverse = new THREE.Quaternion(); this.lastFrame = 0;
    this.badge = document.querySelector('#guide'); this.key = '';
  }
  update(data, elapsed) {
    const cue = guidanceView(data,elapsed);
    const key = [cue.phase,cue.title,cue.detail,cue.context].join('|');
    if (this.key === key) return;
    this.key = key;
    this.badge.dataset.phase = cue.phase;
    this.badge.style.setProperty('--signal',cue.color);
    this.badge.querySelector('strong').textContent = cue.title;
    this.badge.querySelector('small').textContent = cue.detail;
    this.badge.querySelector('.guide-mode').textContent = cue.context;
    const c=this.ctx;
    c.clearRect(0,0,1056,224);
    const glass=c.createLinearGradient(0,0,840,260);
    glass.addColorStop(0,'rgba(47,52,63,.96)');glass.addColorStop(1,'rgba(22,26,33,.94)');
    c.beginPath();c.roundRect(8,8,1040,208,60);c.fillStyle=glass;c.fill();
    c.lineWidth=2.5;c.strokeStyle='rgba(255,255,255,.33)';c.stroke();
    c.save();c.translate(109,112);
    c.fillStyle=cue.color+'22';c.beginPath();c.arc(0,0,53,0,Math.PI*2);c.fill();
    c.strokeStyle=cue.color;c.fillStyle=cue.color;c.lineWidth=7;c.lineCap='round';c.lineJoin='round';c.beginPath();
    if(cue.icon==='check'){c.moveTo(-21,0);c.lineTo(-6,15);c.lineTo(22,-15);}
    else if(cue.icon==='pause'){c.moveTo(-10,-15);c.lineTo(-10,15);c.moveTo(10,-15);c.lineTo(10,15);}
    else if(cue.icon==='follow'){c.moveTo(-17,17);c.lineTo(16,-16);c.moveTo(-6,-16);c.lineTo(16,-16);c.lineTo(16,6);}
    else if(cue.icon==='stop'){c.roundRect(-16,-16,32,32,5);c.fill();}
    else{c.arc(0,0,15,0,Math.PI*2);}
    c.stroke();c.restore();
    const font='system-ui, "Noto Sans CJK SC", sans-serif';
    c.textBaseline='middle';c.fillStyle='#fff';c.font='600 56px '+font;
    c.fillText(cue.title,193,85,755);
    c.fillStyle='#d4d9e1';c.font='400 34px '+font;c.fillText(cue.detail,195,153,790);
    if(cue.context){
      c.font='500 24px '+font;c.textAlign='right';c.fillStyle='#c5ccd7';c.fillText(cue.context,992,50,160);c.textAlign='left';
    }
    this.texture.needsUpdate=true;
  }
  place(viewer, time) {
    this.pivot.position.copy(viewer.position);this.pivot.quaternion.copy(viewer.orientation);
    this.relative.copy(this.destination).sub(this.pivot.position).applyQuaternion(this.headingInverse.copy(this.pivot.quaternion).invert());
    const distance=-this.relative.z;
    // The card stays in space during small head movements. Recenter only when
    // the previous position leaves the upper, comfortably visible area.
    if(!this.lastFrame || distance<.8 || distance>1.7 || Math.abs(this.relative.x)>distance*.25 ||
       this.relative.y<distance*.035 || this.relative.y>distance*.32){
      this.destination.set(0,.23,-1.24).applyQuaternion(this.pivot.quaternion).add(this.pivot.position);
      this.facing.copy(this.pivot.quaternion);
    }
    const blend=this.lastFrame ? 1-Math.exp(-Math.max(0,time-this.lastFrame)/110) : 1;
    this.mesh.position.lerp(this.destination,blend);this.mesh.quaternion.slerp(this.facing,blend);
    this.lastFrame=time;
  }
}
