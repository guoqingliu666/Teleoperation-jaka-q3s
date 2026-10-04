using System;
using UnityEngine;

// All CAD transforms use metres, right-handed Z-up. Unity uses left-handed Y-up.
// B(x,y,z)=(x,z,y) has determinant -1, so axial joint angles change sign.
[Serializable] public class TwinJointConfig {
    public string name;
    public Vector3 origin_xyz_cad_m, origin_rpy_cad_rad, axis_local_cad;
    public float lower_rad, upper_rad;
}
[Serializable] public class TwinViewConfig {
    public string name;
    public Vector3 position_cad_m, look_at_cad_m;
}
[Serializable] public class TwinConfig {
    public int schema, status_udp_port, vr_input_udp_port, zed_serial;
    public string cad_source_sha256, s5_serial, zed_mount;
    public bool site_collision_approved, zed_extrinsic_verified;
    public Vector3 base_position_cad_m, base_rpy_cad_rad, shell_min_cad_m, shell_max_cad_m;
    public float[] initial_joint_sample_rad;
    public TwinJointConfig[] joints;
    public TwinViewConfig[] views;
}
public sealed class TwinRobot : MonoBehaviour {
    public TextAsset configAsset;
    public Transform[] jointTransforms = new Transform[6];
    public Transform link06;
    [NonSerialized] public TwinConfig config;
    [NonSerialized] public float[] q = new float[6];
    public static Vector3 CadToUnity(Vector3 p) { return new Vector3(p.x,p.z,p.y); }
    public static Quaternion CadRpyToUnity(Vector3 rpy) {
        // Apply intrinsic XYZ CAD rotation, mirrored into Unity's handedness.
        var rx=Quaternion.AngleAxis(-rpy.x*Mathf.Rad2Deg,Vector3.right);
        var ry=Quaternion.AngleAxis(-rpy.y*Mathf.Rad2Deg,Vector3.forward);
        var rz=Quaternion.AngleAxis(-rpy.z*Mathf.Rad2Deg,Vector3.up);
        return rz*ry*rx;
    }
    void Awake() {
        if(configAsset==null) { Debug.LogError("[V2Twin] twin_unity.json missing"); return; }
        config=JsonUtility.FromJson<TwinConfig>(configAsset.text);
        Apply(config.initial_joint_sample_rad);
    }
    public bool Apply(float[] positions) {
        if(config==null || positions==null || positions.Length!=6 || jointTransforms.Length!=6) return false;
        for(int i=0;i<6;i++) {
            if(jointTransforms[i]==null || float.IsNaN(positions[i]) || float.IsInfinity(positions[i])) return false;
            if(positions[i]<config.joints[i].lower_rad-.001f || positions[i]>config.joints[i].upper_rad+.001f) return false;
        }
        for(int i=0;i<6;i++) {
            q[i]=positions[i];
            jointTransforms[i].localRotation=CadRpyToUnity(config.joints[i].origin_rpy_cad_rad)*
                Quaternion.AngleAxis(-q[i]*Mathf.Rad2Deg,CadToUnity(config.joints[i].axis_local_cad));
        }
        return true;
    }
}
