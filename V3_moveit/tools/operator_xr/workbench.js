import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {loadRobot,axes,label} from './robot.js';
import {ROS_TO_XR,placeWorkbench,trailBuckets} from './view.js';
import {SpatialGuide} from './guidance.js';

class Workbench {
  constructor(){
    this.canvas=document.querySelector('#xr');
    const context=this.canvas.getContext('webgl2',{xrCompatible:true,alpha:true,antialias:true});
    if(!context)throw Error('浏览器未提供 WebGL2，无法显示三维工作台。');
    this.renderer=new THREE.WebGLRenderer({canvas:this.canvas,context,alpha:true,antialias:true});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));this.renderer.autoClear=false;
    this.scene=new THREE.Scene();this.anchor=new THREE.Group();this.ros=new THREE.Group();
    this.ros.quaternion.copy(ROS_TO_XR);this.ros.position.set(-.1,0,.25);
    this.anchor.add(this.ros);this.scene.add(this.anchor);
    this.scene.add(new THREE.HemisphereLight(0xddeeff,0x475063,2.5));
    const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(2,4,3);this.scene.add(light);
    this.camera=new THREE.PerspectiveCamera(48,1,.01,30);
    this.controls=new OrbitControls(this.camera,this.canvas);this.controls.enableDamping=true;this.controls.minDistance=.25;this.controls.maxDistance=8;
    this.resetDesktop();this.xrCamera=new THREE.PerspectiveCamera(50,1,.05,30);
    this.renderer.xr.enabled=true;this.renderer.xr.setReferenceSpaceType('local-floor');
    this.state=null;this.received=0;this.transportAge=0;this.xrSession=null;this.needsPlacement=true;
    this.lastUI=0;this.lastTrail=0;this.frameCount=0;this.lastViews=0;
    this.guide=new SpatialGuide();this.scene.add(this.guide.mesh);
    this.buildHUD();this.ready=this.load();
    this.ready.catch(error=>{document.querySelector('#loading').textContent='模型加载失败：'+error.message;});
    document.querySelector('#reset').onclick=()=>{this.needsPlacement=true;if(!this.xrSession)this.resetDesktop();};
    this.animate=(t,frame)=>{
      if(frame&&this.xrSession){
        try{this.renderXR(t,frame);this.onInputFrame(t,frame);}
        catch(error){this.onFrameError?.(error);}
      }else if(!frame&&!this.xrSession)this.renderDesktop(t);
    };
    this.renderer.setAnimationLoop(this.animate);
    this.poll();
  }
  resetDesktop(){
    this.anchor.position.set(0,0,0);this.anchor.quaternion.identity();
    this.camera.position.set(1.25,1.55,1.65);this.controls.target.set(0,1,0);this.controls.update();
  }
  async load(){
    const response=await fetch('/xr/model');if(!response.ok)throw Error('模型服务 '+response.status);
    this.description=await response.json();
    this.robot=await loadRobot(this.description,(done,total)=>document.querySelector('#loading').textContent=`加载模型 ${done} / ${total}`);
    this.ros.add(this.robot.root);this.values={...this.description.initial_joints};
    const base=this.robot.links.get('base_link');base.add(axes(.15));
    const grid=new THREE.GridHelper(3,30,0x557788,0x33495b);grid.rotation.x=Math.PI/2;grid.position.z=-.005;base.add(grid);
    const tcp=axes(.08);tcp.add(label('TCP','#a9bccf',.1));this.robot.links.get('tcp').add(tcp);
    this.target=axes(.12,true);const tag=label('TARGET','#6ff0cd',.14);tag.position.z=.15;this.target.add(tag);
    const point=new THREE.Mesh(new THREE.SphereGeometry(.007,12,8),new THREE.MeshBasicMaterial({color:0x69efc8,depthTest:false}));
    point.renderOrder=110;this.target.add(point);this.target.visible=false;base.add(this.target);
    this.trails=Array.from({length:20},(_,i)=>{
      const color=new THREE.Color().setHSL(.45+i/19*.33,.8,.62);
      const geometry=new THREE.BufferGeometry();
      geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(750*6),3).setUsage(THREE.DynamicDrawUsage));
      geometry.setDrawRange(0,0);
      const line=new THREE.LineSegments(geometry,new THREE.LineBasicMaterial({color,transparent:true,opacity:.9*(1-i/20),depthTest:false,depthWrite:false}));
      line.frustumCulled=false;line.renderOrder=105;base.add(line);return line;
    });
    document.querySelector('#loading').hidden=true;document.body.dataset.modelReady='true';
  }
  buildHUD(){
    this.hudCanvas=document.createElement('canvas');this.hudCanvas.width=1024;this.hudCanvas.height=192;
    this.hudTexture=new THREE.CanvasTexture(this.hudCanvas);
    this.hud=new THREE.Mesh(new THREE.PlaneGeometry(.92,.1725),new THREE.MeshBasicMaterial({map:this.hudTexture,transparent:true,depthTest:false,depthWrite:false,side:THREE.DoubleSide}));
    this.hud.scale.setScalar(.58);this.hud.position.set(0,.68,.12);this.hud.renderOrder=150;this.hud.visible=false;this.anchor.add(this.hud);
  }
  async poll(){
    while(true){
      const began=performance.now();
      try{
        const response=await fetch('/xr/state',{cache:'no-store',signal:AbortSignal.timeout(800)});
        if(!response.ok)throw Error('状态连接失败');
        const data=await response.json();
        if(data.schema!==1)throw Error('状态格式不符');
        this.state=data;this.received=performance.now();this.transportAge=(this.received-began)/1000;
      }catch(_){/* Local ageing immediately removes live labels/targets on loss. */}
      // A headset may suspend flat-page animation frames before entering XR.
      // Keep the status accessible there; XR frames also enforce local expiry.
      this.guide.update(this.state?.guidance,(performance.now()-this.received)/1000+this.transportAge);
      await new Promise(resolve=>setTimeout(resolve,50));
    }
  }
  update(t){
    const elapsed=(performance.now()-this.received)/1000+this.transportAge;
    this.guide.update(this.state?.guidance,elapsed);
    if(!this.robot)return;
    const state=this.state,armLive=!!state?.arm_fresh&&elapsed+state.arm_age_s<.5;
    const handLive=!!state?.hand_fresh&&elapsed+state.hand_age_s<state.hand_ttl_s;
    if(state){
      for(const [name,value] of Object.entries(state.joints)){
        if(Number.isFinite(value)&&(name.startsWith('DH_')?handLive:armLive))this.values[name]=value;
      }
      this.robot.update(this.values);
    }
    const target=state?.target;
    this.target.visible=!!target&&elapsed+target.age_s<.15;
    if(this.target.visible){this.target.position.fromArray(target.position_m);this.target.quaternion.fromArray(target.quaternion_xyzw);}
    if(t-this.lastTrail>30){
      this.lastTrail=t;
      trailBuckets(state?.trail||[],elapsed).forEach((positions,i)=>{
        const geo=this.trails[i].geometry,attribute=geo.getAttribute('position');
        attribute.array.set(positions);attribute.needsUpdate=true;geo.setDrawRange(0,positions.length/3);
        this.trails[i].visible=positions.length>0;
      });
    }
    if(t-this.lastUI>200){
      this.lastUI=t;
      const feedback=armLive?`${state.mode==='hardware'?'真机':'仿真'} · 实时关节反馈`:'模型预览 · 无新鲜机械臂反馈';
      const targetText=this.target.visible?'目标 / mm  '+target.position_m.map((v,i)=>`${'XYZ'[i]} ${(v*1000).toFixed(1)}`).join('   '):'目标 · 等待跟随';
      const badge=document.querySelector('#feedback');badge.textContent=feedback;badge.classList.toggle('live',armLive);
      document.querySelector('#target').textContent=targetText;
      document.querySelector('#hand').textContent=handLive?'灵巧手 · 实时反馈':'灵巧手 · 模型预览';
      const ctx=this.hudCanvas.getContext('2d');ctx.clearRect(0,0,1024,192);
      ctx.fillStyle='#0c1c2aee';ctx.fillRect(0,0,1024,192);ctx.fillStyle=armLive?'#74ecc4':'#f5c382';ctx.font='500 32px system-ui';ctx.fillText(feedback,26,47);
      ctx.fillStyle='#deebf5';ctx.font='27px system-ui';ctx.fillText(targetText,26,94);
      ctx.fillStyle='#98b4c9';ctx.font='23px system-ui';ctx.fillText('基坐标 · 虚拟工作台尚未与实物对齐',26,145);this.hudTexture.needsUpdate=true;
      this.hud.visible=!!this.xrSession&&this.target.visible;
      document.body.dataset.armFresh=String(armLive);document.body.dataset.targetVisible=String(this.target.visible);
    }
  }
  async enter(session,space,onFrame,onError){
    this.xrSession=session;this.needsPlacement=true;this.guide.mesh.visible=true;this.guide.lastFrame=0;this.controls.enabled=false;
    this.xrSpace=space;this.onInputFrame=onFrame;this.onFrameError=onError;
    this.renderer.setClearColor(0,0);
    await this.renderer.xr.setSession(session);
    if(this.xrSession===session)this.renderer.xr.setReferenceSpace(space);
  }
  exit(session){
    if(this.xrSession!==session)return;
    this.xrSession=null;this.hud.visible=false;this.guide.mesh.visible=false;this.controls.enabled=true;
    this.lastViews=0;delete this.canvas.dataset.xrViews;
    this.xrSpace=null;this.onInputFrame=null;this.onFrameError=null;this.resetDesktop();
  }
  resetPlacement(){
    this.needsPlacement=true;this.guide.lastFrame=0;
  }
  renderXR(t,frame){
    const pose=this.xrSession.visibilityState==='visible'?frame.getViewerPose(this.xrSpace):null;
    if(pose&&this.needsPlacement){placeWorkbench(this.anchor,pose.transform);this.needsPlacement=false;}
    if(pose)this.guide.place(pose.transform,t);
    this.update(t);
    // WebXRManager binds the compositor's projection surface and renders its
    // per-eye cameras together. Leave that framebuffer under its ownership.
    if(pose){this.renderer.clear();this.renderer.render(this.scene,this.xrCamera);}
    this.lastViews=pose?.views.length||0;this.frameCount++;
    const views=String(this.lastViews);
    if(this.canvas.dataset.xrViews!==views)this.canvas.dataset.xrViews=views;
  }
  renderDesktop(t){
    const w=this.canvas.clientWidth,h=this.canvas.clientHeight;if(!w||!h)return;
    const size=this.renderer.getSize(new THREE.Vector2());
    if(size.x!==w||size.y!==h){this.renderer.setSize(w,h,false);this.camera.aspect=w/h;this.camera.updateProjectionMatrix();}
    this.renderer.setRenderTarget(null);this.renderer.setViewport(0,0,w,h);this.renderer.setScissorTest(false);
    this.renderer.setClearColor(0x111d2b,1);this.renderer.clear();this.controls.update();this.update(t);
    this.renderer.render(this.scene,this.camera);this.frameCount++;
  }
}

export const workbench=new Workbench();
