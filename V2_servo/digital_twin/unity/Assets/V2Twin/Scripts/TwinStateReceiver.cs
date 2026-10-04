using System;
using System.IO;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

[Serializable] public class TwinStatusPacket {
    public int schema;
    public string mode,state,alarm,gripper_state,zed_mount,source_session;
    public bool connected,active,feedback_valid,zed_transform_validated;
    public float[] q_rad,tcp_mm_rad;
    public bool hand_feedback_valid;
    public float[] hand_angle_rad;
    public float feedback_age_ms;
    public double sent_wall_unix_s;
}
public sealed class TwinStateReceiver : MonoBehaviour {
    public TwinRobot robot;
    public VrPoseUdpSender vrSender;
    public int port=5007;
    public string state="waiting", alarm="";
    public bool stale=true, demoMotion=false, recording=false;
    public string tcpText="TCP: --";
    public int received=0,rejected=0;
    public string sourceMode="unknown";
    public int replayApplied=0;
    UdpClient udp; Thread thread; volatile bool running; readonly object gate=new object();
    TwinStatusPacket pending; float lastArrival=-999f, demoStart;
    string dataDir,recordPath;
    StreamWriter recordFile;
    List<TwinStatusPacket> replay=new List<TwinStatusPacket>();int replayIndex=0;float replayNext=0;
    void Start() {
        dataDir=Environment.GetEnvironmentVariable("V2_TWIN_DATA_DIR");
        if(string.IsNullOrEmpty(dataDir))
            dataDir=Path.GetFullPath(Path.Combine(Application.dataPath,Application.isEditor?"../../data":"../../../data"));
        Directory.CreateDirectory(dataDir);
        try {
            udp=new UdpClient(new IPEndPoint(IPAddress.Loopback,port)); udp.Client.ReceiveTimeout=250;
            running=true; thread=new Thread(Receive){IsBackground=true,Name="V2Twin-UDP-readonly"};thread.Start();
            Debug.Log("[V2Twin] read-only state udp://127.0.0.1:"+port);
        } catch(Exception e) { alarm="UDP bind failed: "+e.Message; Debug.LogError(alarm); }
    }
    void Receive() {
        var peer=new IPEndPoint(IPAddress.Loopback,0);
        while(running) try {
            var raw=udp.Receive(ref peer);
            if(!IPAddress.IsLoopback(peer.Address)) continue;
            var packet=JsonUtility.FromJson<TwinStatusPacket>(Encoding.UTF8.GetString(raw));
            if(packet.schema!=1 || packet.q_rad==null || packet.q_rad.Length!=6) { Interlocked.Increment(ref rejected); continue; }
            lock(gate) pending=packet; // bounded latest-slot: never block render or servo threads.
        } catch(SocketException) { } catch(ObjectDisposedException) { break; }
          catch(Exception) { Interlocked.Increment(ref rejected); }
    }
    void Update() {
        TwinStatusPacket packet=null; lock(gate) { packet=pending; pending=null; }
        if(packet!=null) {
            if(packet.feedback_valid && robot.Apply(packet.q_rad)) {
                received++;lastArrival=Time.unscaledTime;state=packet.state;alarm=packet.alarm;sourceMode=packet.mode;
                if(packet.hand_feedback_valid) {
                    var hand=robot.GetComponentInChildren<TwinHand>();
                    if(hand==null || !hand.Apply(packet.hand_angle_rad))rejected++;
                }
                if(packet.tcp_mm_rad!=null && packet.tcp_mm_rad.Length>=3)
                    tcpText=string.Format("TCP mm: {0:F1}, {1:F1}, {2:F1} (只读, 模型外参未确认)",
                        packet.tcp_mm_rad[0],packet.tcp_mm_rad[1],packet.tcp_mm_rad[2]);
                if(recording && recordFile!=null) { recordFile.WriteLine(JsonUtility.ToJson(packet));recordFile.Flush(); }
            } else rejected++;
        }
        stale=Time.unscaledTime-lastArrival>.25f;
        // 真机试验窗口只显示反馈；反馈过期时不能切到仿真动画冒充实机。
        if(VrPoseUdpSender.CommissioningInputRequested)demoMotion=false;
        else if(Input.GetKeyDown(KeyCode.F5)) {demoMotion=!demoMotion;demoStart=Time.unscaledTime;}
        if(Input.GetKeyDown(KeyCode.F6) && vrSender!=null && !QuestReadOnlyDiagnostics.Requested) vrSender.enabled=!vrSender.enabled;
        if(demoMotion && stale && robot.config!=null) {
            var q=(float[])robot.config.initial_joint_sample_rad.Clone();
            q[0]+=0.15f*Mathf.Sin((Time.unscaledTime-demoStart)*.8f);
            robot.Apply(q);state="SIM_DEMO_ONLY";
            var h=robot.GetComponentInChildren<TwinHand>();
            if(h!=null){float a=.15f+.1f*Mathf.Sin(Time.unscaledTime);h.Apply(new[]{a,a,a,a,a,a});}
        }
        if(replay.Count>0 && Time.unscaledTime>=replayNext) {
            if(!stale) {replay.Clear();alarm="实时数据到达，回放已停止";}
            else if(replayIndex>=replay.Count) {replay.Clear();state="REPLAY_DONE";}
            else {robot.Apply(replay[replayIndex++].q_rad);replayApplied++;state="REPLAY_READONLY";replayNext=Time.unscaledTime+.04f;}
        }
        if(Input.GetKeyDown(KeyCode.F9)) StartRecording();
        if(Input.GetKeyDown(KeyCode.F10)) StopRecording();
        if(Input.GetKeyDown(KeyCode.F11)) StartReplay();
        if(Input.GetKeyDown(KeyCode.F12)) {
            string path=Path.Combine(dataDir,"unity_"+DateTime.Now.ToString("yyyyMMdd_HHmmss")+".png");
            ScreenCapture.CaptureScreenshot(path);Debug.Log("[V2Twin] screenshot "+path);
        }
    }
    public void StartRecording() {
        StopRecording(); recordPath=Path.Combine(dataDir,"unity_state_"+DateTime.Now.ToString("yyyyMMdd_HHmmss")+".jsonl");
        recordFile=new StreamWriter(recordPath,false,Encoding.UTF8);recording=true;Debug.Log("[V2Twin] record "+recordPath);
    }
    public void StopRecording() { recording=false;if(recordFile!=null){recordFile.Close();recordFile=null;} }
    public void StartReplay() {
        replay.Clear();replayIndex=0;
        if(VrPoseUdpSender.CommissioningInputRequested){alarm="现场试验窗口只显示实测反馈；回放请使用独立普通孪生窗口";return;}
        if(!stale) {alarm="实时反馈新鲜，先断开状态源再回放";return;}
        var files=Directory.GetFiles(dataDir,"unity_state_*.jsonl");
        if(files.Length==0){alarm="没有录制文件";return;}
        Array.Sort(files,StringComparer.Ordinal);string path=files[files.Length-1];
        foreach(var line in File.ReadLines(path)) try {
            var p=JsonUtility.FromJson<TwinStatusPacket>(line);
            if(p.q_rad!=null && p.q_rad.Length==6) replay.Add(p);
        } catch(Exception) {rejected++;}
        replayNext=Time.unscaledTime;Debug.Log("[V2Twin] read-only replay "+path+" frames "+replay.Count);
    }
    void OnGUI() {
        if(QuestReadOnlyDiagnostics.Requested) return;
        GUI.Box(new Rect(10,10,650,216),"V2 数字孪生 · "+(sourceMode=="simulation"?"模拟状态（非实机反馈）":"状态显示 / Quest输入"));
        GUI.Label(new Rect(20,32,440,22),"状态: "+state+" | "+(stale?"反馈过期":"反馈新鲜")+" | UDP: "+received+" / 拒绝 "+rejected);
        GUI.Label(new Rect(20,54,440,22),"左 S5: "+(robot.config!=null?robot.config.s5_serial:"配置缺失")+" | ZED-M eye-in-hand: 外参未确认");
        GUI.Label(new Rect(20,76,440,22),"F1/F2 箱内视角  F3 自由  F4 末端  F5 仅仿真  F6 VR发送");
        GUI.Label(new Rect(20,98,500,22),"F9 录制  F10 停止  F11 回放最新  F12 截图 | "+(recording?"录制中":"未录制"));
        GUI.Label(new Rect(20,120,500,22),"夹爪: 未接入 | 真实 ZED 外参: 未确认 | 仅显示，不控制机械臂");
        GUI.Label(new Rect(20,142,500,22),tcpText);
        var right=vrSender!=null?vrSender.LastRight:null;
        GUI.Label(new Rect(20,164,620,22),"Quest UDP 5006: "+(vrSender!=null && vrSender.enabled?"发送中":"关闭")+
            " | 帧="+(vrSender!=null?vrSender.SentPackets:0)+" | 运动权限由独立Python安全门控制");
        GUI.Label(new Rect(20,186,620,22),right==null?"等待右手柄；单轴入口自动开启发送，无需F6":
            string.Format("右手追踪={0} Grip={1:F2} 摇杆Y={2:F2} 点击={3} | 输入方式/阶段请看下方提示",right.pose_valid,right.grip,right.thumbstick.y,right.thumbstick_click));
        if(!string.IsNullOrEmpty(alarm)) GUI.Label(new Rect(20,226,600,24),"报警: "+alarm);
    }
    void OnDestroy() {running=false; if(udp!=null)udp.Close();if(thread!=null)thread.Join(500);StopRecording();}
}
