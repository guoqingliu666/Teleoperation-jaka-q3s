using System;
using System.Collections;
using System.IO;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

// Development-only command line probe. Sends a synthetic SIM state to the local read-only receiver.
public sealed class TwinRuntimeProbe : MonoBehaviour {
    public TwinRobot robot;
    public TwinStateReceiver receiver;
    public TwinCameraModes cameraModes;
    void Start() {
        foreach(var arg in Environment.GetCommandLineArgs())
            if(arg=="--v2-probe") {StartCoroutine(Check());return;}
            else if(arg=="--v2-gui-probe") {StartCoroutine(CheckGui());return;}
    }
    IEnumerator Check() {
        yield return new WaitForSecondsRealtime(1);
        if(robot==null || robot.config==null) {Finish(false,"robot config missing");yield break;}
        if(receiver!=null)receiver.StartRecording();
        var q=(float[])robot.config.initial_joint_sample_rad.Clone();q[0]+=.08f;
        var packet=new TwinStatusPacket{schema=1,mode="sim",state="PROBE_SIM_ONLY",connected=true,
            feedback_valid=true,feedback_age_ms=1,q_rad=q,tcp_mm_rad=new float[6],alarm=""};
        byte[] raw=Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));
        using(var udp=new UdpClient()) udp.Send(raw,raw.Length,"127.0.0.1",5007);
        yield return new WaitForSecondsRealtime(.7f);
        bool ok=Mathf.Abs(robot.q[0]-q[0])<1e-4f;
        if(receiver!=null){receiver.StopRecording();receiver.StartReplay();}
        yield return new WaitForSecondsRealtime(.2f);
        bool replayOk=receiver!=null && receiver.replayApplied>0;
        bool cameraOk=false;
        if(cameraModes!=null) {
            cameraModes.SetMode(1);var start=cameraModes.viewCamera.transform.position;
            cameraModes.Orbit(3f,2f);var orbital=cameraModes.viewCamera.transform.position;
            cameraModes.Pan(2f,-1f);var panned=cameraModes.viewCamera.transform.position;
            cameraModes.Dolly(1f);var zoomed=cameraModes.viewCamera.transform.position;
            cameraOk=cameraModes.mode==3 && Vector3.Distance(start,orbital)>.001f &&
                Vector3.Distance(orbital,panned)>.001f && Vector3.Distance(panned,zoomed)>.001f;
        }
        var hand=robot.GetComponentInChildren<TwinHand>();
        bool handOk=hand!=null && hand.Apply(new[]{.1f,.2f,.3f,.2f,.3f,.2f});
        Finish(ok && replayOk && cameraOk && handOk,(ok && replayOk && cameraOk && handOk)?
            "UDP joint, JSONL record/replay and free orbit/pan/zoom passed":"UDP, replay or camera controls failed");
    }
    IEnumerator CheckGui() {
        float start=Time.realtimeSinceStartup;
        while(Time.realtimeSinceStartup-start<21f) {
            if(receiver!=null && receiver.received>5 && !receiver.stale) {
                Finish(true,"GUI status updated Unity joints", "unity_gui_runtime_probe.json","gui_sim_only");yield break;
            }
            yield return new WaitForSecondsRealtime(.1f);
        }
        Finish(false,"GUI status not received or stale", "unity_gui_runtime_probe.json","gui_sim_only");
    }
    void Finish(bool pass,string message,string fileName="unity_runtime_probe.json",string mode="synthetic_sim_only") {
        string dataDir=Environment.GetEnvironmentVariable("V2_TWIN_DATA_DIR");
        if(string.IsNullOrEmpty(dataDir)) dataDir=Path.GetFullPath(Path.Combine(Application.dataPath,"../../data"));
        Directory.CreateDirectory(dataDir);
        File.WriteAllText(Path.Combine(dataDir,fileName),
            "{\"pass\":"+(pass?"true":"false")+",\"message\":\""+message+"\",\"mode\":\""+mode+"\",\"robot_commands\":0}");
        Debug.Log("[V2Twin] RUNTIME_PROBE_"+(pass?"PASS":"FAIL")+" "+message);
        Application.Quit(pass?0:2);
    }
}
