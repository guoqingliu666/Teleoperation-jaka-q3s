import * as THREE from 'three';

// Display coordinates only: ROS (x,y,z) -> WebXR (x,z,-y), a proper rotation.
export const ROS_TO_XR=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2);

export function placeWorkbench(anchor, viewer, distance=1.35) {
  const forward=new THREE.Vector3(0,0,-1).applyQuaternion(new THREE.Quaternion().copy(viewer.orientation));
  forward.y=0;
  if(forward.lengthSq()<.01)forward.set(0,0,-1);else forward.normalize();
  anchor.quaternion.setFromAxisAngle(new THREE.Vector3(0,1,0),Math.atan2(-forward.x,-forward.z));
  // Keep floor-relative height: head movement never drags the workbench around.
  anchor.position.set(viewer.position.x+forward.x*distance,0,viewer.position.z+forward.z*distance);
}

export function trailBuckets(points, elapsed) {
  const buckets=Array.from({length:20},()=>[]);
  for(let i=1;i<points.length;i++){
    const a=points[i-1],b=points[i],age=Math.max(a.age_s,b.age_s)+elapsed;
    if(a.segment!==b.segment||age<0||age>=5)continue;
    buckets[Math.min(19,Math.floor(age*4))].push(...a.position_m,...b.position_m);
  }
  return buckets;
}
