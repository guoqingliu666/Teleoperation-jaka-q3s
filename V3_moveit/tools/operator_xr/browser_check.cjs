// Real Chromium/WebGL, with an explicitly synthetic XR runtime and isolated relay.
const {chromium}=require('playwright');
const fs=require('fs');const assert=require('assert/strict');
const [url,output,fixturePath]=process.argv.slice(2);
const cases=JSON.parse(fs.readFileSync(fixturePath,'utf8'));
(async()=>{
  const browser=await chromium.launch({executablePath:process.env.V3_CHROMIUM||undefined,headless:true,args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader'],downloadsPath:output});
  try{
    const page=await browser.newPage({viewport:{width:1440,height:1060},ignoreHTTPSErrors:true}),errors=[];
    const inputSockets=[];page.on('websocket',socket=>{if(new URL(socket.url()).pathname==='/input')inputSockets.push(socket);});
    page.on('pageerror',e=>errors.push(String(e)));
    page.on('console',msg=>{if(msg.type()==='error')errors.push(msg.text());});
    await page.addInitScript(()=>{
      const pitch=-Math.PI/12,c=Math.cos(pitch),s=Math.sin(pitch);
      const matrix=(x=0,y=0,z=0)=>[1,0,0,0,0,c,s,0,0,-s,c,0,x,y,z,1];
      const projection=()=>{const n=.01,f=30,s=1/Math.tan(Math.PI/6);return [s,0,0,0,0,s,0,0,0,0,-(f+n)/(f-n),-1,0,0,-2*f*n/(f-n),0];};
      window.xrTest={headX:0,grip:0,stop:false,visible:true,ended:false};
      // There is no physical XR compositor in this test. GL drawing stays real.
      Object.defineProperty(window,'XRWebGLBinding',{value:undefined,configurable:true});
      WebGL2RenderingContext.prototype.makeXRCompatible=async()=>{};
      class Session extends EventTarget{
        constructor(){super();this.renderState={};this.space=new EventTarget();this.inputSources=[{handedness:'right',gripSpace:{},gamepad:{mapping:'xr-standard',buttons:[{value:.35},{value:0},{value:0},{pressed:false}]}}];}
        get visibilityState(){return xrTest.visible?'visible':'hidden';}
        updateRenderState(state){Object.assign(this.renderState,state);}
        async requestReferenceSpace(){
          this.spaceRequests=(this.spaceRequests||0)+1;
          if(xrTest.startupFloorReset&&this.spaceRequests===2){
            this.space.dispatchEvent(new Event('reset'));this.space.dispatchEvent(new Event('reset'));
            xrTest.startupFloorReset=false;xrTest.resetDuringEntry=true;
          }
          return this.space;
        }
        cancelAnimationFrame(id){window.cancelAnimationFrame(id);}
        requestAnimationFrame(fn){return window.requestAnimationFrame(t=>{
          if(xrTest.ended)return;
          this.inputSources[0].gamepad.buttons[1].value=xrTest.grip;
          this.inputSources[0].gamepad.buttons[3].pressed=xrTest.stop;
          fn(t,{getPose:()=>({transform:{position:{x:.2,y:1.1,z:-.3},orientation:{x:0,y:0,z:0,w:1}}}),getViewerPose:()=>({
            transform:{position:{x:xrTest.headX,y:1.6,z:0},orientation:{x:Math.sin(pitch/2),y:0,z:0,w:Math.cos(pitch/2)}},
            views:['left','right'].map((eye,i)=>({eye,transform:{matrix:matrix(xrTest.headX+(i?1:-1)*.032,1.6,0)},projectionMatrix:projection()}))
          })});
        });}
        async end(){if(!xrTest.ended){xrTest.ended=true;this.dispatchEvent(new Event('end'));}}
      }
      Object.defineProperty(navigator,'xr',{value:{isSessionSupported:async()=>true,requestSession:async()=>{
        if(xrTest.waitForPermission)await new Promise((resolve,reject)=>{xrTest.deny=()=>reject(new DOMException('Runtime rejected','NotSupportedError'));});
        xrTest.ended=false;return xrTest.session=new Session();
      }}});
      window.XRWebGLLayer=class{
        constructor(session,gl){
          this.gl=gl;this.framebufferWidth=1200;this.framebufferHeight=600;this.framebuffer=gl.createFramebuffer();
          gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
          this.texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,this.texture);
          gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,1200,600,0,gl.RGBA,gl.UNSIGNED_BYTE,null);
          gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR);
          gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,this.texture,0);
          this.depth=gl.createRenderbuffer();gl.bindRenderbuffer(gl.RENDERBUFFER,this.depth);gl.renderbufferStorage(gl.RENDERBUFFER,gl.DEPTH_COMPONENT16,1200,600);
          gl.framebufferRenderbuffer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,gl.RENDERBUFFER,this.depth);
          if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE)throw Error('Synthetic framebuffer incomplete');
          gl.bindFramebuffer(gl.FRAMEBUFFER,null);
        }
        getViewport(view){return {x:view.eye==='left'?0:600,y:0,width:600,height:600};}
      };
    });
    await page.goto(url);await page.waitForFunction(()=>document.body.dataset.modelReady==='true',{timeout:60000});
    await page.evaluate(()=>{const n=document.createElement('div');n.textContent='合成显示验证 · 非实机反馈';n.style.cssText='position:fixed;right:12px;bottom:6px;color:#ffcc88;font:12px system-ui;z-index:100';document.body.append(n);});
    await page.screenshot({path:output+'/desktop_preview.png'});
    const math=await page.evaluate(async cases=>{
      const {workbench:w}=await import('/xr/workbench.js');const T=await import('three');const {ROS_TO_XR,trailBuckets}=await import('/xr/view.js');
      const results=[];
      for(const test of cases){
        const q=Object.fromEntries(test.q.map((v,i)=>['joint_'+(i+1),v]));w.robot.update(q);w.scene.updateMatrixWorld(true);
        const base=w.robot.links.get('base_link').matrixWorld,tcp=w.robot.links.get('tcp').matrixWorld;
        const actual=new T.Matrix4().copy(base).invert().multiply(tcp);
        results.push(Math.max(...test.expected.flatMap((row,r)=>row.map((v,c)=>Math.abs(v-actual.elements[c*4+r])))));
      }
      w.robot.update(w.description.initial_joints);
      const mapped=new T.Vector3(1,2,3).applyQuaternion(ROS_TO_XR).toArray();
      const gap=trailBuckets([{position_m:[0,0,0],age_s:1,segment:1},{position_m:[1,0,0],age_s:0,segment:2}],0).flat().length;
      const expired=trailBuckets([{position_m:[0,0,0],age_s:4,segment:1},{position_m:[1,0,0],age_s:3,segment:1}],2).flat().length;
      let triangles=0;w.robot.root.traverse(o=>{if(o.isMesh)triangles+=(o.geometry.index?.count||o.geometry.attributes.position.count)/3;});
      return {fkErrors:results,mapped,gap,expired,meshes:w.robot.meshCount,triangles};
    },cases);
    assert(math.fkErrors.every(v=>v<1e-10));assert(Math.abs(math.mapped[0]-1)<1e-12&&Math.abs(math.mapped[1]-3)<1e-12&&Math.abs(math.mapped[2]+2)<1e-12);
    assert.equal(math.gap,0);assert.equal(math.expired,0);
    let disconnected=false,phase='ready';
    await page.route('**/xr/state',route=>disconnected?route.abort():route.fulfill({json:{schema:1,arm_fresh:true,arm_age_s:.01,mode:'fake',hand_fresh:false,hand_age_s:null,hand_ttl_s:.08,
      guidance:{phase,mode:'fake',task:'teleop',tool_id:1,ttl_s:.4,instruction:''},
      joints:Object.fromEntries(cases[0].q.map((v,i)=>['joint_'+(i+1),v])),target:{position_m:[.35,-.15,.4],quaternion_xyzw:[0,0,.382683432365,.923879532511],age_s:.01},target_session:10,
      trail:Array.from({length:80},(_,i)=>({position_m:[.35-.08*Math.cos(i/79*5),-.15+.08*Math.sin(i/79*5),.4],age_s:(79-i)/20,segment:i<40?1:2}))}}));
    await page.waitForFunction(()=>document.body.dataset.targetVisible==='true');
    await page.evaluate(async()=>{
      const {workbench:w}=await import('/xr/workbench.js');w.renderer.setAnimationLoop(null);
      w.received=performance.now();w.transportAge=0;w.lastUI=0;w.renderDesktop(performance.now());
    });
    await page.screenshot({path:output+'/desktop_target.png'});
    const targetDebug=await page.evaluate(async()=>{
      const {workbench:w}=await import('/xr/workbench.js'),T=await import('three');
      w.received=performance.now();w.transportAge=0;w.lastUI=0;w.renderDesktop(performance.now());
      const world=w.target.getWorldPosition(new T.Vector3()),screen=world.clone().project(w.camera);
      const ids=[];w.target.traverse(o=>ids.push(o.id));const list=w.renderer.renderLists.get(w.scene,0);
      const gl=w.renderer.getContext(),before=new Uint8Array(gl.drawingBufferWidth*gl.drawingBufferHeight*4),after=new Uint8Array(before.length);
      gl.readPixels(0,0,gl.drawingBufferWidth,gl.drawingBufferHeight,gl.RGBA,gl.UNSIGNED_BYTE,before);
      w.target.visible=false;w.renderer.clear();w.renderer.render(w.scene,w.camera);
      gl.readPixels(0,0,gl.drawingBufferWidth,gl.drawingBufferHeight,gl.RGBA,gl.UNSIGNED_BYTE,after);
      let targetPixels=0;for(let i=0;i<before.length;i+=4)if(Math.abs(before[i]-after[i])+Math.abs(before[i+1]-after[i+1])+Math.abs(before[i+2]-after[i+2])>20)targetPixels++;
      w.target.visible=true;w.renderer.clear();w.renderer.render(w.scene,w.camera);
      return {visible:w.target.visible,targetPixels,world:world.toArray(),screen:screen.toArray(),rendered:list.opaque.concat(list.transparent).filter(r=>ids.includes(r.object.id)).length,png:w.canvas.toDataURL()};
    });
    fs.writeFileSync(output+'/target_canvas.png',Buffer.from(targetDebug.png.split(',')[1],'base64'));delete targetDebug.png;
    assert.equal(targetDebug.visible,true);assert(targetDebug.screen.every(v=>Math.abs(v)<=1));assert(targetDebug.targetPixels>80,JSON.stringify(targetDebug));
    await page.evaluate(async()=>{const {workbench:w}=await import('/xr/workbench.js');w.renderer.setAnimationLoop(w.animate);});
    disconnected=true;await page.waitForFunction(()=>document.body.dataset.armFresh==='false'&&document.body.dataset.targetVisible==='false'&&document.querySelector('#guide').dataset.phase==='unavailable');
    await page.screenshot({path:output+'/desktop_stale.png'});disconnected=false;
    // Failed requests above are deliberate. Record subsequent renderer errors only.
    errors.length=0;
    await page.evaluate(()=>xrTest.waitForPermission=true);
    await page.click('#start');await page.waitForFunction(()=>typeof xrTest.deny==='function');
    assert(await page.locator('#start').isDisabled());assert.equal(inputSockets.length,0);
    assert.equal(await page.locator('#stop').textContent(),'取消进入');
    await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('头显里的系统提示'));
    await page.evaluate(()=>{xrTest.waitForPermission=false;xrTest.deny();});
    await page.waitForFunction(()=>!document.querySelector('#start').disabled&&document.querySelector('#status').textContent.includes('完全退出 Quest Browser'));
    assert.equal(inputSockets.length,0);
    await page.evaluate(()=>xrTest.startupFloorReset=true);
    await page.click('#start');
    try{await page.waitForFunction(()=>document.body.classList.contains('xr'));}
    catch(error){console.error('XR start status:',await page.locator('#status').innerText(),errors);throw error;}
    await page.waitForFunction(()=>document.querySelector('#xr').dataset.xrViews==='2');
    assert(await page.evaluate(()=>xrTest.resetDuringEntry));
    const capture=async name=>{
      const data=await page.evaluate(async()=>{
        const {workbench:w}=await import('/xr/workbench.js');const layer=xrTest.session.renderState.baseLayer,gl=layer.gl;
        gl.bindFramebuffer(gl.FRAMEBUFFER,layer.framebuffer);
        const bytes=new Uint8Array(1200*600*4);gl.readPixels(0,0,1200,600,gl.RGBA,gl.UNSIGNED_BYTE,bytes);
        const canvas=document.createElement('canvas');canvas.width=1200;canvas.height=600;
        const ctx=canvas.getContext('2d'),image=ctx.createImageData(1200,600);let left=0,right=0,difference=0;
        for(let y=0;y<600;y++)for(let x=0;x<1200;x++){
          const src=(y*1200+x)*4,dst=((599-y)*1200+x)*4;image.data.set(bytes.subarray(src,src+4),dst);
          if(bytes[src+3]>30){if(x<600)left++;else right++;}
          if(x<600)difference+=Math.abs(bytes[src]-bytes[src+600*4]);
        }
        ctx.putImageData(image,0,0);ctx.fillStyle='#ffcc88';ctx.font='16px sans-serif';ctx.fillText('SYNTHETIC WebXR — LEFT / RIGHT — NOT HEADSET ACCEPTANCE',18,25);
        return {png:canvas.toDataURL(),left,right,difference,anchor:w.anchor.position.toArray(),orientation:w.anchor.quaternion.toArray(),eyes:w.lastViews,error:gl.getError()};
      });
      fs.writeFileSync(output+'/'+name+'.png',Buffer.from(data.png.split(',')[1],'base64'));delete data.png;return data;
    };
    const stereo=await capture('stereo');assert(stereo.left>5000&&stereo.right>5000);assert(stereo.difference>10000);assert.equal(stereo.error,0);
    const visualStates=[];
    for(const [code,grip,title] of [['ready',0,'可以开始遥操作'],['following',1,'TCP1 跟随中'],['paused',0,'已暂停跟随']]){
      phase=code;await page.evaluate(v=>xrTest.grip=v,grip);
      await page.waitForFunction(c=>document.querySelector('#guide').dataset.phase===c,code);
      const cue=await page.evaluate(async()=>{
        const {workbench:w}=await import('/xr/workbench.js');
        return {title:document.querySelector('#guide strong').textContent,visible:w.guide.mesh.visible,
                stereoViews:w.lastViews,texture:w.guide.canvas.toDataURL()};
      });
      assert(cue.visible&&cue.stereoViews===2);assert.equal(cue.title,title);
      fs.writeFileSync(output+'/guide_'+code+'.png',Buffer.from(cue.texture.split(',')[1],'base64'));
      await capture('stereo_'+code);visualStates.push(cue.title);
    }
    await page.evaluate(()=>{xrTest.headX=.2;xrTest.stop=true;});
    await page.waitForTimeout(250);const shifted=await capture('stereo_head_shift');assert.deepEqual(shifted.anchor,stereo.anchor);assert.deepEqual(shifted.orientation,stereo.orientation);
    assert.notEqual(shifted.left,stereo.left);
    await page.evaluate(()=>xrTest.visible=false);await page.waitForTimeout(150);
    await page.click('#stop');await page.waitForFunction(()=>!document.body.classList.contains('xr'));
    await page.waitForFunction(async()=>!(await (await fetch('/xr/health')).json()).input_active);
    assert.equal(await page.locator('#status').textContent(),'输入已停止。');
    assert.equal(inputSockets.length,1);
    assert.equal(await page.locator('#xr').getAttribute('data-xr-views'),null);
    const stopped=await page.evaluate(()=>xrTest.ended);assert(stopped);
    await page.evaluate(()=>{xrTest.visible=true;xrTest.grip=0;xrTest.stop=false;});
    await page.click('#start');
    await page.waitForFunction(()=>document.querySelector('#xr').dataset.xrViews==='2'&&document.querySelector('#status').textContent.includes('已发送'));
    assert.equal(inputSockets.length,2);
    await page.evaluate(()=>xrTest.session.end());
    await page.waitForFunction(async()=>!(await (await fetch('/xr/health')).json()).input_active);
    assert.equal(await page.locator('#status').textContent(),'已退出透视模式。');
    assert(await page.locator('#start').isEnabled());assert.deepEqual(errors,[]);
    await page.click('#start');
    await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('追踪有效'));
    await page.evaluate(()=>xrTest.session.space.dispatchEvent(new Event('reset')));
    await page.waitForFunction(async()=>!document.body.classList.contains('xr')&&!(await (await fetch('/xr/health')).json()).input_active);
    assert((await page.locator('#status').textContent()).includes('追踪空间已重置，输入已停止'));
    assert.deepEqual(errors,[]);
    const report={passed:true,realChromium:true,syntheticXR:true,physicalHeadset:false,math,targetDebug,stereo,shifted,visualStates,disconnectHidesTarget:true,stopEndsSession:stopped,reentered:true,floorResetBeforeInputContinues:true,floorResetDuringInputStops:true,errors};
    fs.writeFileSync(output+'/browser.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report));
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
