import * as THREE from 'three';
import {OBJLoader} from './vendor/OBJLoader.js';
import {STLLoader} from './vendor/STLLoader.js';

export function applyOrigin(object, origin) {
  object.position.fromArray(origin.xyz);
  // URDF fixed XYZ angles: Rz(yaw) * Ry(pitch) * Rx(roll).
  object.quaternion.setFromEuler(new THREE.Euler(...origin.rpy, 'ZYX'));
}

export function label(text, color='#d9f3ee', width=.24) {
  const canvas=document.createElement('canvas');canvas.width=512;canvas.height=80;
  const ctx=canvas.getContext('2d');ctx.font='600 38px system-ui';ctx.fillStyle=color;
  ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,256,40);
  const texture=new THREE.CanvasTexture(canvas);
  const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,depthTest:false,depthWrite:false}));
  sprite.scale.set(width,width*80/512,1);sprite.renderOrder=120;
  return sprite;
}

export function axes(length, overlay=false) {
  const group=new THREE.Group();
  [[1,0,0,0xff4545],[0,1,0,0x46df68],[0,0,1,0x448dff]].forEach(([x,y,z,color])=>{
    const arrow=new THREE.ArrowHelper(new THREE.Vector3(x,y,z),new THREE.Vector3(),length,color,length*.2,length*.09);
    if(overlay)arrow.traverse(o=>{o.renderOrder=110;if(o.material){o.material.depthTest=false;o.material.depthWrite=false;}});
    group.add(arrow);
  });
  return group;
}

export async function loadRobot(description, progress=()=>{}) {
  const links=new Map(), joints=new Map(), resources=new Map();
  const objLoader=new OBJLoader(),stlLoader=new STLLoader();
  const urls=[...new Set(description.links.flatMap(link=>link.visuals.filter(v=>v.kind==='mesh').map(v=>v.geometry.url)))];
  let next=0,done=0;
  // Bound decode/download concurrency; all meshes finish before XR input starts.
  await Promise.all(Array.from({length:4},async()=>{
    while(next<urls.length){
      const url=urls[next++];
      const object=url.endsWith('.obj')?await objLoader.loadAsync(url):new THREE.Mesh(await stlLoader.loadAsync(url));
      resources.set(url,object);progress(++done,urls.length);
    }
  }));
  for(const link of description.links){
    const node=new THREE.Group();node.name=link.name;links.set(link.name,node);
    for(const visual of link.visuals){
      const g=visual.geometry;let object;
      if(visual.kind==='mesh'){object=resources.get(g.url).clone(true);object.scale.fromArray(g.scale);}
      else if(visual.kind==='box')object=new THREE.Mesh(new THREE.BoxGeometry(...g.size));
      else if(visual.kind==='sphere')object=new THREE.Mesh(new THREE.SphereGeometry(g.radius,24,16));
      else if(visual.kind==='cylinder'){
        const geo=new THREE.CylinderGeometry(g.radius,g.radius,g.length,32);geo.rotateX(Math.PI/2);object=new THREE.Mesh(geo);
      }
      else throw Error('Unsupported model geometry');
      const [r,b,c,a]=visual.rgba;
      const material=new THREE.MeshStandardMaterial({color:new THREE.Color(r,b,c),roughness:.65,metalness:.12,
        transparent:a<1,opacity:a,depthWrite:a>=1,side:THREE.DoubleSide});
      object.traverse(o=>{if(o.isMesh)o.material=material;});
      applyOrigin(object,visual.origin);node.add(object);
    }
  }
  for(const spec of description.joints){
    const mount=new THREE.Group(),turn=new THREE.Group();
    applyOrigin(mount,spec.origin);mount.add(turn);turn.add(links.get(spec.child));links.get(spec.parent).add(mount);
    joints.set(spec.name,{...spec,turn,axis:new THREE.Vector3(...spec.axis).normalize()});
  }
  function update(values){
    const visited=new Set();
    function angle(name){
      if(Number.isFinite(values[name]))return values[name];
      if(visited.has(name))throw Error('Cyclic mimic joint');
      visited.add(name);const mimic=joints.get(name)?.mimic;
      const result=mimic?angle(mimic.joint)*mimic.multiplier+mimic.offset:0;
      visited.delete(name);return result;
    }
    for(const [name,joint] of joints){
      if(joint.type==='revolute'||joint.type==='continuous')joint.turn.quaternion.setFromAxisAngle(joint.axis,angle(name));
      else if(joint.type==='prismatic')joint.turn.position.copy(joint.axis).multiplyScalar(angle(name));
    }
  }
  update(description.initial_joints);
  return {root:links.get('world'),links,joints,update,meshCount:urls.length};
}
