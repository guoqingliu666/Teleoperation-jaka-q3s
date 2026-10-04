using UnityEngine;
using UnityEngine.XR;

// Presets are starting poses. Any mouse gesture enters free camera mode.
// OpenXR stereo/head tracking remains active; mouse nudges its world anchor.
public sealed class TwinCameraModes : MonoBehaviour {
    public TwinRobot robot;
    public Camera viewCamera;
    public int mode=1;
    public Vector3 focus;
    Vector3 xrAnchorPosition;
    Quaternion xrAnchorRotation;
    void Start() {SetMode(1);}
    void Update() {
        if(Input.GetKeyDown(KeyCode.F1)) SetMode(1);
        if(Input.GetKeyDown(KeyCode.F2)) SetMode(2);
        if(Input.GetKeyDown(KeyCode.F3)) SetMode(3);
        if(Input.GetKeyDown(KeyCode.F4)) SetMode(4);
        if(mode==4 && robot.link06!=null) {
            focus=robot.link06.position;
            xrAnchorPosition=robot.link06.TransformPoint(new Vector3(0,.18f,-.35f));
            xrAnchorRotation=Quaternion.LookRotation(focus-xrAnchorPosition,Vector3.up);
            if(!XRSettings.isDeviceActive) {
                viewCamera.transform.position=xrAnchorPosition;
                viewCamera.transform.rotation=xrAnchorRotation;
            }
        }
        float dx=Input.GetAxis("Mouse X"),dy=Input.GetAxis("Mouse Y");
        float wheel=Input.GetAxis("Mouse ScrollWheel");
        bool pan=Input.GetMouseButton(2) || (Input.GetKey(KeyCode.LeftShift) && Input.GetMouseButton(0));
        bool orbit=Input.GetMouseButton(0) && !pan;
        bool look=Input.GetMouseButton(1);
        if(((pan || orbit || look) && (Mathf.Abs(dx)+Mathf.Abs(dy)>0)) || Mathf.Abs(wheel)>0) {
            if(mode!=3) ActivateFree();
            if(XRSettings.isDeviceActive) {
                if(look || orbit) xrAnchorRotation=Quaternion.AngleAxis(dx*2f,Vector3.up)*xrAnchorRotation*
                    Quaternion.AngleAxis(-dy*2f,Vector3.right);
                if(pan) xrAnchorPosition-=xrAnchorRotation*(new Vector3(dx,dy,0)*.02f);
                if(Mathf.Abs(wheel)>0) xrAnchorPosition+=xrAnchorRotation*Vector3.forward*(wheel*.8f);
            } else {
                if(pan) Pan(dx,dy);
                else if(orbit) Orbit(dx,dy);
                else if(look) Look(dx,dy);
                if(Mathf.Abs(wheel)>0) Dolly(wheel);
            }
        }
        if(XRSettings.isDeviceActive) {
            var head=InputDevices.GetDeviceAtXRNode(XRNode.Head);
            Vector3 localPosition;Quaternion localRotation;
            if(head.isValid && head.TryGetFeatureValue(CommonUsages.devicePosition,out localPosition) &&
               head.TryGetFeatureValue(CommonUsages.deviceRotation,out localRotation)) {
                viewCamera.transform.position=xrAnchorPosition+xrAnchorRotation*localPosition;
                viewCamera.transform.rotation=xrAnchorRotation*localRotation;
                return;
            }
        }
        if(mode==3) {
            float speed=Input.GetKey(KeyCode.LeftShift)?2f:.6f;
            Vector3 move=new Vector3(Input.GetAxisRaw("Horizontal"),
                (Input.GetKey(KeyCode.E)?1:0)-(Input.GetKey(KeyCode.Q)?1:0),Input.GetAxisRaw("Vertical"));
            Vector3 world=viewCamera.transform.TransformDirection(move)*speed*Time.deltaTime;
            viewCamera.transform.position+=world;focus+=world;
        }
    }
    void ActivateFree() {
        if(mode==4 && robot.link06!=null) focus=robot.link06.position;
        mode=3;xrAnchorPosition=viewCamera.transform.position;xrAnchorRotation=viewCamera.transform.rotation;
    }
    public void Orbit(float dx,float dy) {
        if(mode!=3)ActivateFree();
        Vector3 offset=viewCamera.transform.position-focus;
        Quaternion turn=Quaternion.AngleAxis(dx*2f,Vector3.up)*
            Quaternion.AngleAxis(-dy*2f,viewCamera.transform.right);
        viewCamera.transform.position=focus+turn*offset;
        viewCamera.transform.LookAt(focus,Vector3.up);
        xrAnchorPosition=viewCamera.transform.position;xrAnchorRotation=viewCamera.transform.rotation;
    }
    public void Pan(float dx,float dy) {
        if(mode!=3)ActivateFree();
        float scale=Mathf.Max(.15f,Vector3.Distance(focus,viewCamera.transform.position))*.02f;
        Vector3 shift=-(viewCamera.transform.right*dx+viewCamera.transform.up*dy)*scale;
        viewCamera.transform.position+=shift;focus+=shift;xrAnchorPosition=viewCamera.transform.position;
    }
    public void Dolly(float wheel) {
        if(mode!=3)ActivateFree();
        Vector3 offset=viewCamera.transform.position-focus;
        float distance=Mathf.Clamp(offset.magnitude*Mathf.Exp(-wheel*2f),.05f,20f);
        viewCamera.transform.position=focus+offset.normalized*distance;
        xrAnchorPosition=viewCamera.transform.position;
    }
    public void Look(float dx,float dy) {
        if(mode!=3)ActivateFree();
        viewCamera.transform.rotation=Quaternion.AngleAxis(dx*2f,Vector3.up)*viewCamera.transform.rotation*
            Quaternion.AngleAxis(-dy*2f,Vector3.right);
        focus=viewCamera.transform.position+viewCamera.transform.forward*
            Mathf.Max(.15f,Vector3.Distance(focus,viewCamera.transform.position));
        xrAnchorRotation=viewCamera.transform.rotation;
    }
    public void SetMode(int selected) {
        mode=selected;
        if(robot.config==null || viewCamera==null || mode==4) return;
        var view=robot.config.views[mode==1?0:mode==2?1:2];
        focus=TwinRobot.CadToUnity(view.look_at_cad_m);
        viewCamera.transform.position=TwinRobot.CadToUnity(view.position_cad_m);
        viewCamera.transform.LookAt(focus,Vector3.up);
        xrAnchorPosition=viewCamera.transform.position;xrAnchorRotation=viewCamera.transform.rotation;
    }
}
