// The standalone input page needs only a transparent projection surface.
export const display = {
  ready: Promise.resolve(),
  async enter(session, space, onFrame, onError) {
    const gl=document.querySelector('#xr').getContext('webgl',{xrCompatible:true,alpha:true});
    if(!gl)throw Error('浏览器未提供 WebGL');
    await gl.makeXRCompatible();
    session.updateRenderState({baseLayer:new XRWebGLLayer(session,gl,{alpha:true,ignoreDepthValues:true})});
    let active=true;
    session.addEventListener('end',()=>{active=false;},{once:true});
    const draw=(time,frame)=>{
      if(!active)return;
      session.requestAnimationFrame(draw);
      try{
        const layer=session.renderState.baseLayer;
        if(!layer)return;
        gl.bindFramebuffer(gl.FRAMEBUFFER,layer.framebuffer);
        gl.clearColor(0,0,0,0);gl.clear(gl.COLOR_BUFFER_BIT);
        onFrame(time,frame);
      }catch(error){onError(error);}
    };
    session.requestAnimationFrame(draw);
  },
  exit() {}
};
