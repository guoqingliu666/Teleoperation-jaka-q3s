using System;
using System.IO;
using System.Collections;
using UnityEngine;

// Explicit desktop acceptance only. Synthetic head poses exercise anchor math, not headset acceptance.
public sealed class TwinViewProbe : MonoBehaviour {
    [Serializable] public class Report {
        public string scope="actual_Player_camera_and_panel_synthetic_head_math_no_hardware";
        public bool default_details_hidden,panel_fixed_in_world,desktop_mouse_functions,headset_worn_test=false;
        public bool xr_active;
        public bool details_can_toggle,free_mode_preserves_xr_anchor;
        public float default_roll_sine,default_table_yaw_sine,anchor_up_error;
        public int robot_commands=0;
    }
    string folder;
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Boot() {
        var path=Environment.GetEnvironmentVariable("V2_VIEW_PROBE");if(string.IsNullOrEmpty(path))return;
        new GameObject("Explicit_view_acceptance_only").AddComponent<TwinViewProbe>().folder=path;
    }
    IEnumerator Start() {
        Directory.CreateDirectory(folder);yield return new WaitForSecondsRealtime(3);
        var modes=FindObjectOfType<TwinCameraModes>();var hud=FindObjectOfType<QuestCommissioningHud>();
        var cam=Camera.main;var r=new Report{xr_active=UnityEngine.XR.XRSettings.isDeviceActive};modes.SetMode(1);
        r.default_details_hidden=!TwinCameraModes.ShowDiagnostics && hud!=null && !hud.PanelVisible;
        r.default_roll_sine=Mathf.Abs(Vector3.Dot(cam.transform.right,Vector3.up));
        r.default_table_yaw_sine=Mathf.Abs(cam.transform.forward.x);
        yield return new WaitForEndOfFrame();ScreenCapture.CaptureScreenshot(Path.Combine(folder,"default_unobstructed.png"));
        yield return new WaitForSecondsRealtime(.3f);
        TwinCameraModes.ShowDiagnostics=true;yield return new WaitForEndOfFrame();
        r.details_can_toggle=TwinCameraModes.ShowDiagnostics && (!r.xr_active || hud.PanelVisible);
        Vector3 panel=hud.PanelPosition;
        Vector3 start=cam.transform.position;
        modes.Orbit(6,3);bool orbit=Vector3.Distance(start,cam.transform.position)>.01f;
        start=cam.transform.position;modes.Pan(3,2);bool pan=Vector3.Distance(start,cam.transform.position)>.01f;
        start=cam.transform.position;modes.Dolly(.2f);bool dolly=Vector3.Distance(start,cam.transform.position)>.01f;
        yield return null;
        r.panel_fixed_in_world=Vector3.Distance(panel,hud.PanelPosition)<1e-7f;
        r.desktop_mouse_functions=orbit && pan && dolly;
        TwinCameraModes.ShowDiagnostics=false;
        modes.SetMode(1);modes.RecenterHead(new Vector3(.2f,1.65f,.1f),Quaternion.Euler(23,41,17));
        r.anchor_up_error=Vector3.Distance(modes.AnchorRotation*Vector3.up,Vector3.up);
        Quaternion anchor=modes.AnchorRotation;modes.SendMessage("ActivateFree");
        r.free_mode_preserves_xr_anchor=!r.xr_active || Quaternion.Angle(anchor,modes.AnchorRotation)<.0001f;
        modes.SetMode(1);
        File.WriteAllText(Path.Combine(folder,"view_report.json"),JsonUtility.ToJson(r,true));
        yield return new WaitForSecondsRealtime(.4f);Application.Quit();
    }
}
