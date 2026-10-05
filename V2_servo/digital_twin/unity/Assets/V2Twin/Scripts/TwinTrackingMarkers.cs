using System;
using UnityEngine;

[Serializable] public class TwinTrackingPacket {
    public int schema;
    public string frame,transform_layout,feedback_source,tool_model,model_sha256;
    public float[] feedback_T,target_T,tcp2_T;
    public bool target_valid,physical_feedback,solver_valid;
    public float position_error_mm,orientation_error_deg,sigma_min,joint_velocity_scale;
    public string[] warnings;
}

// Display only. Basis reflection is applied to each axis, not guessed Euler angles.
// Markers are diagnostic overlays, never collision shapes or a robot command source.
public sealed class TwinTrackingMarkers : MonoBehaviour {
    public TwinRobot robot;
    public string status="TCP/目标标记：等待带坐标来源的新鲜反馈";
    public bool FeedbackVisible {get{return feedback!=null && feedback.activeSelf;}}
    public bool TargetVisible {get{return goal!=null && goal.activeSelf;}}
    public Vector3 FeedbackWorld {get{return feedback.transform.position;}}
    public Vector3 TargetWorld {get{return goal.transform.position;}}
    public Vector3[] FeedbackAxes {get{return feedbackAxesEnds;}}
    public Vector3[] TargetAxes {get{return targetAxesEnds;}}
    public bool Tcp2Visible {get{return tcp2!=null && tcp2.activeSelf;}}
    public Vector3 Tcp2World {get{return tcp2.transform.position;}}
    public Vector3[] Tcp2Axes {get{return tcp2AxesEnds;}}
    GameObject feedback,goal; LineRenderer[] feedbackAxes,goalAxes;
    Vector3[] feedbackAxesEnds=new Vector3[3],targetAxesEnds=new Vector3[3];
    Material material; TextMesh feedbackLabel,goalLabel;
    GameObject tcp2;LineRenderer[] tcp2Axes;LineRenderer toolExtension;TextMesh tcp2Label;
    Vector3[] tcp2AxesEnds=new Vector3[3];
    const float AxisLength=.075f;
    void Ensure() {
        if(feedback!=null)return;
        material=new Material(Resources.Load<Shader>("TwinMarkers"));
        feedback=Make("SIM TCP1",new Color(.05f,1f,1f),false,out feedbackAxes,out feedbackLabel);
        goal=Make("HAND GOAL",new Color(1f,.15f,.85f),true,out goalAxes,out goalLabel);
        tcp2=Make("TCP2 REF",new Color(1f,.75f,.15f),false,out tcp2Axes,out tcp2Label);
        toolExtension=Line(tcp2.transform,new Color(.1f,.7f,1f),2,.002f);
        Clear();
    }
    LineRenderer Line(Transform parent,Color color,int count,float width) {
        var obj=new GameObject("diagnostic_line");obj.transform.SetParent(parent,false);
        var line=obj.AddComponent<LineRenderer>();line.sharedMaterial=material;
        line.startColor=color;line.endColor=color;line.startWidth=width;line.endWidth=width;
        line.positionCount=count;line.useWorldSpace=false;line.numCapVertices=3;return line;
    }
    GameObject Make(string name,Color color,bool wire,out LineRenderer[] axes,out TextMesh label) {
        var root=new GameObject(name+"_diagnostic_only");root.transform.SetParent(transform,false);
        if(!wire) {
            var ball=GameObject.CreatePrimitive(PrimitiveType.Sphere);ball.transform.SetParent(root.transform,false);
            ball.transform.localScale=Vector3.one*.018f;Destroy(ball.GetComponent<Collider>());
            var m=new Material(material);m.color=color;ball.GetComponent<Renderer>().material=m;
        } else for(int plane=0;plane<3;plane++) {
            var circle=Line(root.transform,color,49,.0018f);
            for(int i=0;i<=48;i++) {
                float a=i*Mathf.PI*2/48,x=Mathf.Cos(a)*.016f,y=Mathf.Sin(a)*.016f;
                circle.SetPosition(i,plane==0?new Vector3(x,y,0):plane==1?new Vector3(x,0,y):new Vector3(0,x,y));
            }
        }
        axes=new[]{Line(root.transform,Color.red,2,.0025f),Line(root.transform,Color.green,2,.0025f),Line(root.transform,Color.blue,2,.0025f)};
        var text=new GameObject(name+"_label");text.transform.SetParent(root.transform,false);
        label=text.AddComponent<TextMesh>();label.text=name;label.fontSize=36;label.characterSize=.0022f;
        label.anchor=TextAnchor.MiddleCenter;label.color=color;
        text.transform.localPosition=new Vector3(0,wire?.055f:.03f,0);
        return root;
    }
    static bool Finite(float v){return !float.IsNaN(v) && !float.IsInfinity(v);}
    static bool Rigid(float[] t) {
        if(t==null || t.Length!=16)return false;
        foreach(float v in t)if(!Finite(v))return false;
        if(Mathf.Abs(t[12])+Mathf.Abs(t[13])+Mathf.Abs(t[14])+Mathf.Abs(t[15]-1)>.0001f)return false;
        var x=new Vector3(t[0],t[4],t[8]);var y=new Vector3(t[1],t[5],t[9]);var z=new Vector3(t[2],t[6],t[10]);
        return Mathf.Abs(x.sqrMagnitude-1)<.001f && Mathf.Abs(y.sqrMagnitude-1)<.001f &&
            Mathf.Abs(z.sqrMagnitude-1)<.001f && Mathf.Abs(Vector3.Dot(x,y))<.001f &&
            Vector3.Dot(Vector3.Cross(x,y),z)>.999f;
    }
    public Vector3 BasePointToWorld(Vector3 p){return robot.transform.TransformPoint(TwinRobot.CadToUnity(p));}
    void Place(GameObject marker,LineRenderer[] axes,Vector3[] ends,float[] t) {
        Vector3 position=new Vector3(t[3],t[7],t[11]);Vector3 world=BasePointToWorld(position);
        marker.transform.position=world;marker.transform.rotation=Quaternion.identity;
        for(int i=0;i<3;i++) {
            ends[i]=BasePointToWorld(position+new Vector3(t[i],t[4+i],t[8+i])*AxisLength);
            axes[i].SetPosition(0,Vector3.zero);axes[i].SetPosition(1,ends[i]-world);
        }
        marker.SetActive(true);
    }
    public void Apply(TwinStatusPacket packet) {
        Ensure();var t=packet.tracking;
        if(robot==null || !packet.feedback_valid || !packet.connected || packet.mode!="simulation" ||
           t==null || t.schema!=1 || t.frame!="robot_base" || t.transform_layout!="row_major_4x4_m" ||
           t.feedback_source!="simulated_fk" || t.physical_feedback || !Rigid(t.feedback_T)) {Clear();return;}
        Place(feedback,feedbackAxes,feedbackAxesEnds,t.feedback_T);
        bool currentTool=t.tool_model=="TCP1_plus200mm_localZ_user_configured_shadow" && Rigid(t.tcp2_T);
        if(currentTool) {
            // Reject inconsistent TCP2 metadata instead of displaying an arbitrary offset.
            Vector3 p1=new Vector3(t.feedback_T[3],t.feedback_T[7],t.feedback_T[11]);
            Vector3 p2=new Vector3(t.tcp2_T[3],t.tcp2_T[7],t.tcp2_T[11]);
            Vector3 z2=new Vector3(t.tcp2_T[2],t.tcp2_T[6],t.tcp2_T[10]);
            currentTool=Vector3.Distance(p1-p2,z2*.2f)<.00001f;
            for(int row=0;row<3;row++)for(int col=0;col<3;col++)
                currentTool &= Mathf.Abs(t.feedback_T[row*4+col]-t.tcp2_T[row*4+col])<.00001f;
            if(currentTool) {
                Place(tcp2,tcp2Axes,tcp2AxesEnds,t.tcp2_T);
                toolExtension.SetPosition(0,Vector3.zero);toolExtension.SetPosition(1,FeedbackWorld-Tcp2World);
            }
        }
        if(!currentTool)tcp2.SetActive(false);
        bool target=t.target_valid && packet.active && Rigid(t.target_T);
        if(target)Place(goal,goalAxes,targetAxesEnds,t.target_T);else goal.SetActive(false);
        status="青球=模拟TCP；紫色线球=手柄映射目标；RGB=XYZ轴（辅助叠加显示，无碰撞体）\n"+
            (target?string.Format("误差 {0:F1}mm / {1:F1}°；关节速度比例 {2:F2}；σ {3:F4}",
                t.position_error_mm,t.orientation_error_deg,t.joint_velocity_scale,t.sigma_min):"目标已解绑/失效")+
            (currentTool?"；TCP1=TCP2局部+Z 200mm（用户配置影子模型，非实测TCP）":"；历史/未识别工具模型，非实测TCP");
        if(t.warnings!=null && t.warnings.Length>0)status+="；"+string.Join(",",t.warnings);
    }
    public void Clear() {
        if(feedback!=null)feedback.SetActive(false);if(goal!=null)goal.SetActive(false);
        if(tcp2!=null)tcp2.SetActive(false);
        status="TCP/目标标记失效或数据过期；不以历史目标/候选指令冒充实测反馈";
    }
    void LateUpdate() {
        if(Camera.main==null)return;
        if(feedbackLabel!=null)feedbackLabel.transform.rotation=Camera.main.transform.rotation;
        if(goalLabel!=null)goalLabel.transform.rotation=Camera.main.transform.rotation;
        if(tcp2Label!=null)tcp2Label.transform.rotation=Camera.main.transform.rotation;
    }
    void OnDestroy(){if(material!=null)Destroy(material);}
}
