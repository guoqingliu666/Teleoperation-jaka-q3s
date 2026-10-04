using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;

// 只在显式诊断环境变量下记录实际渲染应用节奏；不产生关节状态或机器人命令。
[DefaultExecutionOrder(1000)]
public sealed class TwinDisplayTimingProbe : MonoBehaviour {
    [Serializable] public class Sample { public double host_probe_s; public string phase; public int applied; public float q0; }
    [Serializable] public class Report {
        public string scope="actual_Unity_Update_observation_no_robot";
        public int robot_commands=0; public bool headset_photon_timing_validated=false;
        public List<Sample> samples=new List<Sample>();
    }
    string path; TwinStateReceiver receiver; Report report=new Report(); int previous=-1; bool saved;
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Boot() {
        string p=Environment.GetEnvironmentVariable("V2_TWIN_DISPLAY_TIMING_PROBE");
        if(string.IsNullOrEmpty(p))return;
        new GameObject("V2_Display_Timing_Probe_NoSDK").AddComponent<TwinDisplayTimingProbe>().path=p;
    }
    void Start(){receiver=FindObjectOfType<TwinStateReceiver>();}
    void LateUpdate() {
        if(saved || receiver==null || receiver.received==previous || receiver.stale)return;
        previous=receiver.received;
        report.samples.Add(new Sample{host_probe_s=(double)System.Diagnostics.Stopwatch.GetTimestamp()/System.Diagnostics.Stopwatch.Frequency,
            phase=receiver.state,applied=receiver.received,q0=receiver.robot.q[0]});
        // 诊断样本有界；只读测试应在一分钟内结束。
        if(report.samples.Count>=10000)Save();
    }
    void Save(){if(saved || string.IsNullOrEmpty(path))return;saved=true;Directory.CreateDirectory(Path.GetDirectoryName(path));File.WriteAllText(path,JsonUtility.ToJson(report,true));}
    void OnApplicationQuit(){Save();}
    void OnDestroy(){Save();}
}
