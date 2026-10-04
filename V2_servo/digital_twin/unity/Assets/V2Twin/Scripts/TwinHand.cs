using System;
using UnityEngine;
[Serializable] public class HandPart {public string name,mesh;public float[] xyz,rpy;}
[Serializable] public class HandJoint {public string name,type,parent,child;public float[] xyz,rpy,axis;public float lower,upper;}
[Serializable] public class HandAssembly {public float[] mount_xyz,mount_rpy;public HandPart[] rigid,links;public HandJoint[] joints;}
// Visual kinematics only. No EtherCAT, force simulation, enable or robot command.
public sealed class TwinHand : MonoBehaviour {
    public TextAsset assemblyAsset;
    public Transform[] pivots;
    public static Vector3 V(float[] a) {return a==null?Vector3.zero:new Vector3(a[0],a[1],a[2]);}
    public bool Apply(float[] angles) {
        if(angles==null || angles.Length!=6)return false;
        foreach(float q in angles)if(float.IsNaN(q)||float.IsInfinity(q))return false;
        var a=JsonUtility.FromJson<HandAssembly>(assemblyAsset.text);
        if(pivots==null || pivots.Length!=a.joints.Length)return false;
        string[] active={"finger11","finger12","finger21","finger31","finger41","finger51"};
        for(int i=0;i<a.joints.Length;i++) {
            var j=a.joints[i];float q=0;int index=Array.IndexOf(active,j.name);
            if(index>=0)q=angles[index];
            else if(j.name=="finger13")q=.004592f+.8808f*angles[1]+.469825f*angles[1]*angles[1];
            else for(int n=2;n<=5;n++)if(j.name=="finger"+n+"2")q=.011072f+.9206f*angles[n]+.200535f*angles[n]*angles[n];
            q=j.type=="fixed"?0:Mathf.Clamp(q,j.lower,j.upper);
            pivots[i].localRotation=TwinRobot.CadRpyToUnity(V(j.rpy))*Quaternion.AngleAxis(-q*Mathf.Rad2Deg,TwinRobot.CadToUnity(V(j.axis)));
        }
        return true;
    }
    void Awake(){Apply(new float[6]);}
}
