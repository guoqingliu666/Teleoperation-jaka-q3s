using System;
using System.IO;
using System.Collections.Concurrent;
using System.Threading;
using System.Collections;
using UnityEngine;
using UnityEngine.XR;
using System.Runtime.InteropServices;

/// <summary>现有 V2Twin 的只读诊断模式。不创建 SDK，不向控制 UDP 端口发包。</summary>
public sealed class QuestReadOnlyDiagnostics : MonoBehaviour {
    // Unity/Mono Stopwatch 的纪元可能是进程启动，明确调用 Windows QPC 与 Python perf_counter 对齐。
    static class Qpc {
        [DllImport("Kernel32.dll")] static extern bool QueryPerformanceCounter(out long count);
        [DllImport("Kernel32.dll")] static extern bool QueryPerformanceFrequency(out long frequency);
        public static long Frequency {get {QueryPerformanceFrequency(out long f);return f;}}
        public static long GetTimestamp(){QueryPerformanceCounter(out long n);return n;}
    }
    public static bool Requested => Array.IndexOf(Environment.GetCommandLineArgs(), "--quest-diagnostics") >= 0;
    [Serializable] public sealed class Frame {
        public string kind="openxr_readonly", step, session;
        public int sequence, dropped;
        public long host_qpc_ticks, host_qpc_frequency;
        public double unity_time_s;
        public bool xr_active;
        public string xr_device;
        public VrPoseUdpSender.ControllerSample head, left, right;
        public string inputsystem_right_device;
        public bool inputsystem_right_click_available, inputsystem_right_click, inputsystem_right_stick_available;
        public VrPoseUdpSender.Vec2 inputsystem_right_stick=new VrPoseUdpSender.Vec2();
    }
    readonly ConcurrentQueue<string> lines=new ConcurrentQueue<string>();
    readonly string[] steps={"自由检查 / 不冻结映射", "全部松开，缓慢转头", "仅右 Grip：按住再松开", "仅右扳机：扣下再松开", "仅右摇杆：上下左右", "仅右摇杆：点击再松开", "仅右 A：按下再松开", "仅右 B：按下再松开", "移动左右手柄，最后全部松开"};
    volatile bool writing; Thread writer; string folder, error="", session;
    int sequence, dropped, step; Frame latest; float screenshotAt;
    float recordingStart;
    bool restarting;
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Boot() {
        if(!Requested) return;
        foreach(var sender in FindObjectsOfType<VrPoseUdpSender>(true)) sender.enabled=false;
        new GameObject("Quest_READONLY_NoRobotCommands").AddComponent<QuestReadOnlyDiagnostics>();
    }
    void Start() {
        Application.runInBackground=true;
        session=Guid.NewGuid().ToString();
        folder=Environment.GetEnvironmentVariable("V2_QUEST_DIAGNOSTICS_DIR");
        if(string.IsNullOrEmpty(folder)) folder=Path.GetFullPath(Path.Combine(Application.dataPath,"../../../../data/quest_"+DateTime.Now.ToString("yyyyMMdd_HHmmss")));
        Directory.CreateDirectory(folder);
        File.WriteAllText(Path.Combine(folder,"session.json"),"{\"schema\":1,\"session\":\""+session+"\",\"mode\":\"read_only_no_robot_commands\",\"qpc_frequency\":"+Qpc.Frequency+",\"mapping_frozen\":false}");
        writing=true;writer=new Thread(WriteLoop){IsBackground=true,Name="Quest-readonly-log"};writer.Start();
        screenshotAt=Time.unscaledTime+8;
        recordingStart=Time.unscaledTime;
        Debug.Log("[QuestDiagnostic] READONLY_START "+folder);
    }
    void Update() {
        latest=new Frame {session=session, sequence=sequence++, dropped=dropped, step=steps[step],
            host_qpc_ticks=Qpc.GetTimestamp(),host_qpc_frequency=Qpc.Frequency,unity_time_s=Time.realtimeSinceStartupAsDouble,
            xr_active=XRSettings.isDeviceActive,xr_device=XRSettings.loadedDeviceName,
            head=VrPoseUdpSender.ReadController(InputDevices.GetDeviceAtXRNode(XRNode.Head)),
            left=VrPoseUdpSender.ReadController(InputDevices.GetDeviceAtXRNode(XRNode.LeftHand)),
            right=VrPoseUdpSender.ReadController(InputDevices.GetDeviceAtXRNode(XRNode.RightHand))};
        ReadInputSystemRight(latest);
        if(writing && Time.unscaledTime-recordingStart>=600) {
            writing=false;error="录制已达10分钟上限，请退出并重新启动建立新会话";
            Debug.Log("[QuestDiagnostic] RECORDING_LIMIT_600_SECONDS");
        }
        if(writing) {if(lines.Count<2048) lines.Enqueue(JsonUtility.ToJson(latest));else dropped++;}
        if(Input.GetKeyDown(KeyCode.F7)) step=(step+1)%steps.Length;
        if(Input.GetKeyDown(KeyCode.F8)) Capture();
        if(screenshotAt>0 && Time.unscaledTime>=screenshotAt){screenshotAt=-1;Capture();}
    }
    void Capture(){ScreenCapture.CaptureScreenshot(Path.Combine(folder,"diagnostic_"+DateTime.Now.ToString("yyyyMMdd_HHmmss")+".png"));}
    IEnumerator RetryXR() {
        restarting=true;
        var settings=UnityEngine.XR.Management.XRGeneralSettings.Instance;
        if(settings==null || settings.Manager==null) {error="XR manager missing";restarting=false;yield break;}
        var manager=settings.Manager;
        manager.StopSubsystems();manager.DeinitializeLoader();
        yield return manager.InitializeLoader();
        if(manager.activeLoader!=null) {manager.StartSubsystems();error="";}
        else error="OpenXR unavailable: enter Quest Link on headset, then retry";
        restarting=false;
    }
    void WriteLoop() {
        try {using(var output=new StreamWriter(new FileStream(Path.Combine(folder,"openxr_raw.jsonl"),FileMode.CreateNew,FileAccess.Write,FileShare.Read),new System.Text.UTF8Encoding(false))){
            while(writing || !lines.IsEmpty) {int count=0;while(count++<256 && lines.TryDequeue(out var line)) output.WriteLine(line);output.Flush();Thread.Sleep(20);}
        }}catch(Exception e){error=e.ToString();writing=false;}
    }
    static string Describe(string name,VrPoseUdpSender.ControllerSample s) {
        return string.Format("{0}: {1}\nconnected={2} tracked={3} pose={4}\nGrip={5:F3} Trigger={6:F3} Stick=({7:F2},{8:F2})\nprimary={9} secondary={10} stickClick={11}",name,s.device_name,s.connected,s.tracked,s.pose_valid,s.grip,s.trigger,s.thumbstick.x,s.thumbstick.y,s.primary_button,s.secondary_button,s.thumbstick_click);
    }
    static void ReadInputSystemRight(Frame frame) {
        // 与旧 XR.CommonUsages 并排记录。此处只诊断，不把未核验的点击直接绑定停止。
        foreach(var device in UnityEngine.InputSystem.InputSystem.devices) {
            bool right=false;
            foreach(var usage in device.usages) if(usage.ToString()=="RightHand") right=true;
            if(!right) continue;
            frame.inputsystem_right_device=device.displayName+" / "+device.layout;
            var click=device.TryGetChildControl<UnityEngine.InputSystem.Controls.ButtonControl>("thumbstickClicked");
            if(click!=null) {frame.inputsystem_right_click_available=true;frame.inputsystem_right_click=click.isPressed;}
            var stick=device.TryGetChildControl<UnityEngine.InputSystem.Controls.StickControl>("thumbstick");
            if(stick!=null) {frame.inputsystem_right_stick_available=true;frame.inputsystem_right_stick=VrPoseUdpSender.Vec2.From(stick.ReadValue());}
            break;
        }
    }
    void OnGUI() {
        if(latest==null) return;
        GUI.Box(new Rect(8,8,720,555),"Quest / OpenXR 只读诊断：无机器人命令，F6发送已禁用");
        GUI.Label(new Rect(20,35,690,30),"XR active="+latest.xr_active+" | "+latest.xr_device+" | 帧="+sequence+" | 写盘丢帧="+dropped);
        GUI.Label(new Rect(20,70,680,85),Describe("头显",latest.head));
        GUI.Label(new Rect(20,158,680,85),Describe("左手",latest.left));
        GUI.Label(new Rect(20,246,680,85),Describe("右手",latest.right));
        GUI.Label(new Rect(20,313,680,28),"InputSystem右: "+latest.inputsystem_right_device+" 点击="+latest.inputsystem_right_click+" Stick="+latest.inputsystem_right_stick.x.ToString("F2")+","+latest.inputsystem_right_stick.y.ToString("F2"));
        GUI.Label(new Rect(20,335,680,30),"当前操作标签："+steps[step]);
        if(GUI.Button(new Rect(20,369,210,30),"下一步标签 (F7)")) step=(step+1)%steps.Length;
        if(GUI.Button(new Rect(240,369,170,30),"截图 (F8)")) Capture();
        if(GUI.Button(new Rect(420,369,200,30),"结束并保存 / 退出")) Application.Quit();
        GUI.Label(new Rect(20,405,680,50),"先看追踪，再依次按 Grip / 扳机 / 摇杆及点击 / A / B。\n标签仅供记录：按键实测需核对原始变化与操作者确认，不能自动判通过。");
        GUI.Label(new Rect(20,460,680,50),string.IsNullOrEmpty(error)?folder:"写盘/连接状态: "+error);
        if(!restarting && !latest.xr_active && GUI.Button(new Rect(20,511,420,30),"已进入 Quest Link：重试 OpenXR 连接")) StartCoroutine(RetryXR());
    }
    void OnDestroy(){writing=false;if(writer!=null && !writer.Join(2000)) Debug.LogError("Quest diagnostic writer did not drain");}
}
