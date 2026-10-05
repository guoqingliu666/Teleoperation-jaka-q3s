using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Explicit diagnostic run only: records the actual scene objects after UDP application.
public sealed class TwinTrackingProbe : MonoBehaviour {
    [Serializable] public class Sample {
        public string phase;public bool feedback_visible,target_visible,tcp2_visible;
        public Vector3 feedback_world,target_world,tcp2_world;public Vector3[] feedback_axes,target_axes,tcp2_axes;
    }
    [Serializable] public class Report {public string scope="actual_Unity_UDP_marker_objects";public List<Sample> samples=new List<Sample>();public int robot_commands=0;}
    string folder,lastPhase="";TwinStateReceiver receiver;Report report=new Report();
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Boot() {
        string path=Environment.GetEnvironmentVariable("V2_TRACKING_MARKER_PROBE");
        if(string.IsNullOrEmpty(path))return;
        new GameObject("Explicit_marker_validation_only").AddComponent<TwinTrackingProbe>().folder=path;
    }
    void Start(){receiver=FindObjectOfType<TwinStateReceiver>();Directory.CreateDirectory(folder);StartCoroutine(Observe());}
    IEnumerator Observe() {
        while(true) {
            yield return new WaitForSecondsRealtime(.12f);
            if(receiver==null || receiver.trackingMarkers==null)continue;
            string phase=receiver.state+(receiver.stale?"_STALE":"");
            if(!phase.StartsWith("MARKER_") || phase==lastPhase)continue;
            lastPhase=phase;var m=receiver.trackingMarkers;
            var s=new Sample{phase=phase,feedback_visible=m.FeedbackVisible,target_visible=m.TargetVisible};
            if(s.feedback_visible){s.feedback_world=m.FeedbackWorld;s.feedback_axes=(Vector3[])m.FeedbackAxes.Clone();}
            if(s.target_visible){s.target_world=m.TargetWorld;s.target_axes=(Vector3[])m.TargetAxes.Clone();}
            s.tcp2_visible=m.Tcp2Visible;
            if(s.tcp2_visible){s.tcp2_world=m.Tcp2World;s.tcp2_axes=(Vector3[])m.Tcp2Axes.Clone();}
            report.samples.Add(s);File.WriteAllText(Path.Combine(folder,"markers.json"),JsonUtility.ToJson(report,true));
            if(s.target_visible) {
                // Separate evidence camera: never lock or move the user's camera/XR anchor.
                var obj=new GameObject("Marker_evidence_camera_only");var camera=obj.AddComponent<Camera>();
                camera.CopyFrom(Camera.main);camera.enabled=false;camera.stereoTargetEye=StereoTargetEyeMask.None;
                Vector3 focus=(s.feedback_world+s.target_world)*.5f;
                camera.transform.position=focus+new Vector3(.38f,.28f,-.4f);camera.transform.LookAt(focus);
                camera.fieldOfView=42;camera.nearClipPlane=.01f;
                var target=new RenderTexture(1280,800,24);var image=new Texture2D(1280,800,TextureFormat.RGB24,false);
                var previous=RenderTexture.active;
                try {
                    camera.targetTexture=target;RenderTexture.active=target;camera.Render();
                    image.ReadPixels(new Rect(0,0,1280,800),0,0);image.Apply();
                    File.WriteAllBytes(Path.Combine(folder,phase+".png"),image.EncodeToPNG());
                } finally {camera.targetTexture=null;RenderTexture.active=previous;Destroy(image);Destroy(target);Destroy(obj);}
            }
        }
    }
}
