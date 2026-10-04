using System;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;

/// <summary>既有Player的现场阶段提示。只读显示文件，不连接SDK、不发运动权限。</summary>
public sealed class QuestCommissioningHud : MonoBehaviour {
    public const string Version="20261003-mapping-readout";
    [Serializable] public sealed class Guidance {
        public int schema;
        public string scope,stage,title,instruction,detail,input_mode,session,control_scope;
        public double sent_wall_unix_s;
    }
    [Serializable] public sealed class Probe {
        public string version=Version,stage,title,instruction,input_mode,font_name;
        public bool stale,world_space_canvas,xr_active;
        public float body_required_height,body_available_height;
        public string display_only="no_robot_SDK_or_motion_permission";
    }
    string path,probePath,lastStage="",readError="";
    float nextPoll; Guidance latest; bool stale=true;
    Canvas canvas; Text heading,body; Image panel; Font font; Material overlayMaterial;
    Camera view; AudioSource sound; AudioClip cue;
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Boot() {
        if(!VrPoseUdpSender.CommissioningInputRequested || QuestReadOnlyDiagnostics.Requested)return;
        new GameObject("Quest_Commissioning_Stage_DisplayOnly").AddComponent<QuestCommissioningHud>();
    }
    void Start() {
        var dir=Environment.GetEnvironmentVariable("V2_TWIN_DATA_DIR");
        if(string.IsNullOrEmpty(dir))dir=Path.GetFullPath(Path.Combine(Application.dataPath,"../../../data"));
        path=Path.Combine(dir,"quest_live_guidance.json");
        probePath=Environment.GetEnvironmentVariable("V2_QUEST_GUIDANCE_PROBE");
        view=Camera.main;
        if(view!=null && FindObjectOfType<AudioListener>()==null)view.gameObject.AddComponent<AudioListener>();
        font=Font.CreateDynamicFontFromOSFont(new[]{"Microsoft YaHei","SimHei","Arial"},36);
        var overlayShader=Resources.Load<Shader>("QuestStageOverlay");
        if(overlayShader==null)Debug.LogError("[QuestGuidance] overlay shader missing; stage may be occluded");
        else overlayMaterial=new Material(overlayShader);
        var root=new GameObject("Quest_Stage_WorldSpace",typeof(RectTransform),typeof(Canvas),typeof(CanvasGroup));
        root.transform.SetParent(transform,false);
        canvas=root.GetComponent<Canvas>();canvas.renderMode=RenderMode.WorldSpace;canvas.worldCamera=view;
        canvas.sortingOrder=200;
        var rect=root.GetComponent<RectTransform>();rect.sizeDelta=new Vector2(1100,260);
        root.transform.localScale=Vector3.one*.00065f;
        root.GetComponent<CanvasGroup>().blocksRaycasts=false;
        var background=new GameObject("Panel",typeof(RectTransform),typeof(Image));background.transform.SetParent(root.transform,false);
        var br=background.GetComponent<RectTransform>();br.anchorMin=Vector2.zero;br.anchorMax=Vector2.one;br.offsetMin=br.offsetMax=Vector2.zero;
        panel=background.GetComponent<Image>();panel.raycastTarget=false;
        if(overlayMaterial!=null)panel.material=overlayMaterial;
        heading=MakeText(root.transform,"Heading",new Vector2(20,-12),new Vector2(1060,54),40);
        body=MakeText(root.transform,"Instruction",new Vector2(20,-72),new Vector2(1060,180),29);
        sound=gameObject.AddComponent<AudioSource>();sound.spatialBlend=0;sound.volume=.35f;
        // 进入实际H3时两声提示音。此声音仅辅助识别阶段，不能当成控制器运动确认。
        const int rate=24000;var samples=new float[(int)(rate*.38f)];
        for(int i=0;i<samples.Length;i++) {
            float t=(float)i/rate,local=t<.15f?t:t-.21f;
            if(local>=0 && local<.15f) samples[i]=.35f*Mathf.Sin(2*Mathf.PI*(t<.15f?660:880)*local)*Mathf.Sin(Mathf.PI*local/.15f);
        }
        cue=AudioClip.Create("Quest_H3_live_two_tones",samples.Length,1,rate,false);cue.SetData(samples,0);
        Debug.Log("[QuestGuidance] START version="+Version+" file="+path+" world_space_display_only");
    }
    Text MakeText(Transform parent,string name,Vector2 offset,Vector2 size,int fontSize) {
        var go=new GameObject(name,typeof(RectTransform),typeof(Text));go.transform.SetParent(parent,false);
        var r=go.GetComponent<RectTransform>();r.anchorMin=r.anchorMax=r.pivot=new Vector2(0,1);r.anchoredPosition=offset;r.sizeDelta=size;
        var text=go.GetComponent<Text>();text.font=font;text.fontSize=fontSize;text.color=Color.white;
        text.alignment=TextAnchor.UpperLeft;text.horizontalOverflow=HorizontalWrapMode.Wrap;text.verticalOverflow=VerticalWrapMode.Overflow;text.raycastTarget=false;
        if(overlayMaterial!=null)text.material=overlayMaterial;
        return text;
    }
    void Update() {
        if(Time.unscaledTime<nextPoll)return;nextPoll=Time.unscaledTime+.2f;
        try {
            if(File.Exists(path)) {
                var p=JsonUtility.FromJson<Guidance>(File.ReadAllText(path,System.Text.Encoding.UTF8));
                if(p!=null && p.schema==1 && p.scope=="operator_display_only_no_motion_permission")latest=p;
            }
            readError="";
        } catch(Exception e){readError=e.Message;}
        var age=latest==null?double.PositiveInfinity:DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()/1000.0-latest.sent_wall_unix_s;
        stale=age<-.25 || age>1.5 || !string.IsNullOrEmpty(readError);
        string stage=stale?"WAIT_STALE":latest.stage;
        if(stage!=lastStage) {
            Debug.Log("[QuestGuidance] STAGE "+stage+(latest==null?"":" | "+latest.title));
            if(stage=="LIVE")sound.PlayOneShot(cue);
            if(!string.IsNullOrEmpty(probePath))ScreenCapture.CaptureScreenshot(Path.ChangeExtension(probePath,"."+stage+".png"));
            lastStage=stage;
        }
        bool live=!stale && stage=="LIVE", error=stage=="ERROR" || stage=="STOPPING";
        panel.color=live?new Color(.015f,.24f,.07f,.96f):error?new Color(.33f,.015f,.015f,.96f):new Color(.10f,.12f,.20f,.96f);
        heading.text=stale?"阶段提示已过期 · 请松开Grip":latest.title;
        body.text=stale?"等待启动器/控制台的新提示。显示过期不代表机器人已停止；核对控制台，异常用物理急停。":
            latest.instruction+"\n"+latest.detail+"\n"+(string.IsNullOrEmpty(latest.control_scope)?"仅J1 0..+0.1°":latest.control_scope)+" | 硬件急停独立 | 此面板只显示阶段";
        // 映射/指令/反馈同屏显示；面板随文字高度增长，不裁掉真正的机器人反馈。
        float bodyHeight=Mathf.Max(180,body.preferredHeight+8);
        body.rectTransform.sizeDelta=new Vector2(1060,bodyHeight);
        canvas.GetComponent<RectTransform>().sizeDelta=new Vector2(1100,bodyHeight+84);
        if(!string.IsNullOrEmpty(probePath)) try {
            File.WriteAllText(probePath,JsonUtility.ToJson(new Probe{stage=stage,title=heading.text,
                instruction=body.text,input_mode=latest==null?"":latest.input_mode,stale=stale,
                world_space_canvas=canvas.renderMode==RenderMode.WorldSpace,xr_active=XRSettings.isDeviceActive,
                body_required_height=body.preferredHeight,body_available_height=body.rectTransform.rect.height,
                font_name=font==null?"missing":font.name}));
        }catch(Exception e){Debug.LogWarning("[QuestGuidance] probe write: "+e.Message);}
    }
    void LateUpdate() {
        if(view==null)view=Camera.main;
        if(view==null || canvas==null)return;
        // 跟随现有相机位置；不覆写相机、鼠标或OpenXR头部追踪。
        canvas.transform.position=view.transform.TransformPoint(new Vector3(0,-.25f,1.05f));
        canvas.transform.rotation=view.transform.rotation;
    }
    void OnGUI() {
        if(canvas==null || XRSettings.isDeviceActive)return; // 桌面另外显示大字；VR使用真实世界空间Canvas。
        var titleStyle=new GUIStyle(GUI.skin.label){font=font,fontSize=23,wordWrap=true};
        var bodyStyle=new GUIStyle(GUI.skin.label){font=font,fontSize=18,wordWrap=true};
        float width=Mathf.Min(730,Screen.width-20),contentWidth=width-30;
        string title=stale?"阶段提示过期 / 松开Grip":latest.title;
        float titleHeight=titleStyle.CalcHeight(new GUIContent(title),contentWidth);
        float textHeight=bodyStyle.CalcHeight(new GUIContent(body.text),contentWidth);
        float height=titleHeight+textHeight+28;
        float top=Mathf.Max(10,Mathf.Min(260,Screen.height-height-10));
        var oldColor=GUI.color;GUI.color=panel.color;
        GUI.DrawTexture(new Rect(10,top,width,height),Texture2D.whiteTexture);GUI.color=oldColor;
        GUI.Label(new Rect(25,top+8,contentWidth,titleHeight),title,titleStyle);
        GUI.Label(new Rect(25,top+titleHeight+16,contentWidth,textHeight),body.text,bodyStyle);
    }
    void OnDestroy(){if(cue!=null)Destroy(cue);if(font!=null)Destroy(font);if(overlayMaterial!=null)Destroy(overlayMaterial);}
}
