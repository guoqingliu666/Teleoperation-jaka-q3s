using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.Build.Reporting;
using UnityEditor.XR.Management;
using UnityEditor.XR.Management.Metadata;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using UnityEngine.XR.Management;

public static class V2TwinBuilder {
    const string Root="Assets/V2Twin";
    static string TwinPath { get { return Path.GetFullPath(Path.Combine(Application.dataPath,"../..")); } }
    static string Generated { get { return Root+"/Generated"; } }
    static string Report { get { return Path.Combine(TwinPath,"data/unity_build_report.json"); } }
    static TwinConfig config;
    static TextAsset configAsset;
    static void Folders() {
        Directory.CreateDirectory(Path.Combine(Application.dataPath,"V2Twin/Generated/Meshes"));
        Directory.CreateDirectory(Path.Combine(Application.dataPath,"V2Twin/Scenes"));
        Directory.CreateDirectory(Path.Combine(Application.dataPath,"V2Twin/Prefabs"));
    }
    static void ReadConfig() {
        AssetDatabase.Refresh();
        configAsset=AssetDatabase.LoadAssetAtPath<TextAsset>(Root+"/Data/twin_unity.json");
        if(configAsset==null) throw new Exception("twin_unity.json missing");
        config=JsonUtility.FromJson<TwinConfig>(configAsset.text);
        if(config==null || config.joints.Length!=6 || config.views.Length<3 || config.site_collision_approved)
            throw new Exception("twin configuration is invalid or unexpectedly site-approved");
    }
    static void ConfigureOpenXR() {
        const string path=Root+"/Generated/XRGeneralSettingsPerBuildTarget.asset";
        var store=AssetDatabase.LoadAssetAtPath<XRGeneralSettingsPerBuildTarget>(path);
        if(store==null) {
            store=ScriptableObject.CreateInstance<XRGeneralSettingsPerBuildTarget>();
            AssetDatabase.CreateAsset(store,path);
        }
        EditorBuildSettings.AddConfigObject(XRGeneralSettings.settingsKey,store,true);
        if(!store.HasManagerSettingsForBuildTarget(BuildTargetGroup.Standalone))
            store.CreateDefaultManagerSettingsForBuildTarget(BuildTargetGroup.Standalone);
        var settings=store.SettingsForBuildTarget(BuildTargetGroup.Standalone);
        settings.InitManagerOnStart=true;
        if(!settings.Manager.activeLoaders.Any(loader=>loader!=null && loader.GetType().FullName=="UnityEngine.XR.OpenXR.OpenXRLoader")) {
            if(!XRPackageMetadataStore.AssignLoader(settings.Manager,"UnityEngine.XR.OpenXR.OpenXRLoader",BuildTargetGroup.Standalone))
                throw new Exception("OpenXR Standalone loader assignment failed");
        }
        EditorUtility.SetDirty(settings);EditorUtility.SetDirty(store);AssetDatabase.SaveAssets();
        var openxr=UnityEngine.XR.OpenXR.OpenXRSettings.GetSettingsForBuildTargetGroup(BuildTargetGroup.Standalone);
        if(openxr==null) throw new Exception("OpenXR Standalone settings missing");
        int profiles=0;
        foreach(var feature in openxr.GetFeatures<UnityEngine.XR.OpenXR.Features.OpenXRInteractionFeature>()) {
            if(feature.GetType().Name=="OculusTouchControllerProfile" || feature.GetType().Name=="MetaQuestTouchPlusControllerProfile") {
                feature.enabled=true;EditorUtility.SetDirty(feature);profiles++;
                Debug.Log("[V2Twin] enabled controller profile "+feature.GetType().Name);
            }
        }
        if(profiles!=2) throw new Exception("Expected Oculus Touch + Meta Touch Plus profiles");
        EditorUtility.SetDirty(openxr);AssetDatabase.SaveAssets();
        Debug.Log("[V2Twin] OpenXR Standalone loader assigned");
    }
    static Mesh ImportMesh(string name) {
        string assetPath=Generated+"/Meshes/"+name+".asset";
        Mesh cached=AssetDatabase.LoadAssetAtPath<Mesh>(assetPath);
        if(cached!=null) AssetDatabase.DeleteAsset(assetPath); // source .v2mesh may have changed on rebuild
        using(var file=File.OpenRead(Path.Combine(TwinPath,"meshes/visual/"+name+".v2mesh")))
        using(var reader=new BinaryReader(file)) {
            if(new string(reader.ReadChars(4))!="V2M1") throw new Exception("mesh magic "+name);
            int nv=(int)reader.ReadUInt32(), nf=(int)reader.ReadUInt32();
            if(nv<3 || nf<1 || nv>4000000 || nf>4000000) throw new Exception("mesh dimensions "+name);
            var vertices=new Vector3[nv];var triangles=new int[nf*3];
            for(int i=0;i<nv;i++) vertices[i]=TwinRobot.CadToUnity(new Vector3(reader.ReadSingle(),reader.ReadSingle(),reader.ReadSingle()));
            for(int i=0;i<nf;i++) {
                int a=(int)reader.ReadUInt32(),b=(int)reader.ReadUInt32(),c=(int)reader.ReadUInt32();
                if(a>=nv || b>=nv || c>=nv) throw new Exception("mesh index "+name);
                triangles[3*i]=a;triangles[3*i+1]=c;triangles[3*i+2]=b; // reflected basis changes winding.
            }
            var mesh=new Mesh{name=name,indexFormat=IndexFormat.UInt32};
            mesh.vertices=vertices;mesh.triangles=triangles;mesh.RecalculateNormals();mesh.RecalculateBounds();
            AssetDatabase.CreateAsset(mesh,assetPath);return mesh;
        }
    }
    static Material Material(string name,Color color,bool transparent=false) {
        string path=Generated+"/"+name+".mat";
        var existing=AssetDatabase.LoadAssetAtPath<Material>(path);
        var shader=Shader.Find(transparent?"V2Twin/TransparentDoubleSided":"V2Twin/OpaqueDoubleSided");
        if(shader==null) throw new Exception("V2 double-sided shader missing; prevent broken backface preview");
        var material=existing==null?new Material(shader){name=name}:existing;
        material.shader=shader;material.color=color;
        material.renderQueue=transparent?3000:-1;
        if(existing==null)AssetDatabase.CreateAsset(material,path);
        else EditorUtility.SetDirty(material);
        return material;
    }
    static GameObject Visual(string name,Transform parent,Material material) {
        var obj=new GameObject(name);obj.transform.SetParent(parent,false);
        obj.AddComponent<MeshFilter>().sharedMesh=ImportMesh(name);
        var renderer=obj.AddComponent<MeshRenderer>();renderer.sharedMaterial=material;
        if(name=="cabinet_shell") {renderer.shadowCastingMode=ShadowCastingMode.Off;renderer.receiveShadows=false;}
        return obj;
    }
    static void LodVisual(string name,Transform parent,Material material) {
        var group=new GameObject(name+"_LOD_group");group.transform.SetParent(parent,false);
        var high=Visual(name,group.transform,material).GetComponent<MeshRenderer>();
        var low=Visual(name+"_lod1",group.transform,material).GetComponent<MeshRenderer>();
        var lod=group.AddComponent<LODGroup>();
        lod.SetLODs(new[]{new LOD(.28f,new[]{high}),new LOD(.015f,new[]{low})});lod.RecalculateBounds();
    }
    static void BuildHand(Transform link06) {
        var text=AssetDatabase.LoadAssetAtPath<TextAsset>(Root+"/Data/hand_assembly.json");
        if(text==null)throw new Exception("Run digital_twin/hand_assets.py first");
        var a=JsonUtility.FromJson<HandAssembly>(text.text);var mat=Material("DH116",new Color(.18f,.20f,.24f,1));
        foreach(var part in a.rigid)Visual("DH_"+part.name,link06,mat);
        var handRoot=new GameObject("DH116EC_R1_visual_unverified_contact").transform;handRoot.SetParent(link06,false);
        handRoot.localPosition=TwinRobot.CadToUnity(TwinHand.V(a.mount_xyz));handRoot.localRotation=TwinRobot.CadRpyToUnity(TwinHand.V(a.mount_rpy));
        var links=new Dictionary<string,Transform>{{"base_link",handRoot}};var pivots=new Transform[a.joints.Length];
        for(int i=0;i<a.joints.Length;i++) {
            var j=a.joints[i];var p=new GameObject("DH_"+j.name).transform;p.SetParent(links[j.parent],false);
            p.localPosition=TwinRobot.CadToUnity(TwinHand.V(j.xyz));p.localRotation=TwinRobot.CadRpyToUnity(TwinHand.V(j.rpy));
            links[j.child]=p;pivots[i]=p;
        }
        foreach(var part in a.links) {
            var v=Visual("DH_"+part.name,links[part.name],mat).transform;
            v.localPosition=TwinRobot.CadToUnity(TwinHand.V(part.xyz));v.localRotation=TwinRobot.CadRpyToUnity(TwinHand.V(part.rpy));
        }
        var hand=handRoot.gameObject.AddComponent<TwinHand>();hand.assemblyAsset=text;hand.pivots=pivots;hand.Apply(new float[6]);
    }
    [MenuItem("V2 Twin/Build workcell scene")]
    public static void Build() {
        try {
            Folders();ReadConfig();ConfigureOpenXR();
            var scene=EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
            var shellMat=Material("cabinet_glass",new Color(.55f,.73f,.8f,.08f),true);
            var fixtureMat=Material("fixtures",new Color(.55f,.61f,.66f,1));
            var neighborMat=Material("legacy_neighbor_unverified",new Color(.95f,.55f,.18f,1));
            var armMat=Material("left_s5",new Color(.10f,.57f,.70f,1));
            var staticRoot=new GameObject("Cabinet_CAD_preview_not_collision_approved");
            Visual("cabinet_shell",staticRoot.transform,shellMat);
            LodVisual("workcell_fixtures",staticRoot.transform,fixtureMat);
            LodVisual("legacy_neighbor_placeholder",staticRoot.transform,neighborMat);
            PrefabUtility.SaveAsPrefabAsset(staticRoot,Root+"/Prefabs/Cabinet_CAD_preview.prefab");
            var armRoot=new GameObject("Leftmost_JAKA_S5_only_actuated");
            armRoot.transform.position=TwinRobot.CadToUnity(config.base_position_cad_m);
            armRoot.transform.rotation=TwinRobot.CadRpyToUnity(config.base_rpy_cad_rad);
            Transform parent=armRoot.transform;
            var pivots=new Transform[6];
            Visual("Link_00",parent,armMat);
            for(int i=0;i<6;i++) {
                var spec=config.joints[i];var pivot=new GameObject(spec.name).transform;
                pivot.SetParent(parent,false);pivot.localPosition=TwinRobot.CadToUnity(spec.origin_xyz_cad_m);
                pivot.localRotation=TwinRobot.CadRpyToUnity(spec.origin_rpy_cad_rad);
                Visual("Link_0"+(i+1),pivot,armMat);pivots[i]=pivot;parent=pivot;
            }
            var robot=armRoot.AddComponent<TwinRobot>();robot.configAsset=configAsset;robot.jointTransforms=pivots;robot.link06=parent;
            robot.config=config;
            if(!robot.Apply(config.initial_joint_sample_rad)) throw new Exception("initial sample violates joint limits");
            BuildHand(parent);
            PrefabUtility.SaveAsPrefabAsset(armRoot,Root+"/Prefabs/Leftmost_S5_6DOF.prefab");
            new GameObject("Third_arm_location_unknown_NO_GEOMETRY_NO_COLLISION").transform.SetParent(staticRoot.transform,false);
            var cameraObj=new GameObject("TwinView_Camera");var camera=cameraObj.AddComponent<Camera>();
            camera.tag="MainCamera";camera.fieldOfView=70;camera.nearClipPlane=.015f;camera.farClipPlane=20;
            camera.stereoTargetEye=StereoTargetEyeMask.Both;
            camera.backgroundColor=new Color(.07f,.12f,.17f);
            var listener=cameraObj.AddComponent<AudioListener>();
            cameraObj.transform.position=TwinRobot.CadToUnity(config.views[0].position_cad_m);
            cameraObj.transform.LookAt(TwinRobot.CadToUnity(config.views[0].look_at_cad_m));
            var views=cameraObj.AddComponent<TwinCameraModes>();views.robot=robot;views.viewCamera=camera;
            var receiver=new GameObject("GUI_readonly_feedback_UDP_5007").AddComponent<TwinStateReceiver>();
            receiver.robot=robot;receiver.port=config.status_udp_port;
            var probe=new GameObject("Runtime_probe_only_with_flag").AddComponent<TwinRuntimeProbe>();
            probe.robot=robot;probe.receiver=receiver;probe.cameraModes=views;
            var sender=new GameObject("QDU_Quest_pose_sender_to_V2_UDP_5006").AddComponent<VrPoseUdpSender>();
            sender.enabled=false; // activate explicitly only after Quest input source checks.
            receiver.vrSender=sender;
            var light=new GameObject("Inside_cabinet_light").AddComponent<Light>();light.type=LightType.Directional;
            light.intensity=1.25f;light.transform.rotation=Quaternion.Euler(55,-35,0);
            RenderSettings.ambientLight=new Color(.6f,.63f,.67f);
            EditorSceneManager.SaveScene(scene,Root+"/Scenes/Workcell.unity");
            EditorBuildSettings.scenes=new[]{new EditorBuildSettingsScene(Root+"/Scenes/Workcell.unity",true)};
            AssetDatabase.SaveAssets();
            File.WriteAllText(Report,"{\"result\":\"built\",\"unity_editor\":\""+Application.unityVersion+
                "\",\"scene\":\"Assets/V2Twin/Scenes/Workcell.unity\",\"moving_arms\":1,\"joint_count\":6,\"site_collision_approved\":false}");
            Debug.Log("[V2Twin] BUILD_OK scene, 2 prefabs, 6 joints, 4 view modes, read-only status UDP");
        } catch(Exception e) { Debug.LogException(e); throw; }
    }
    [MenuItem("V2 Twin/Validate saved scene")]
    public static void Validate() {
        var scene=EditorSceneManager.OpenScene(Root+"/Scenes/Workcell.unity");
        var robot=UnityEngine.Object.FindObjectOfType<TwinRobot>();
        var receiver=UnityEngine.Object.FindObjectOfType<TwinStateReceiver>();
        var view=UnityEngine.Object.FindObjectOfType<TwinCameraModes>();
        var xr=XRGeneralSettingsPerBuildTarget.XRGeneralSettingsForBuildTarget(BuildTargetGroup.Standalone);
        if(!scene.IsValid() || robot==null || robot.jointTransforms.Length!=6 || robot.link06==null ||
            receiver==null || view==null || Camera.main==null || xr==null || xr.Manager==null ||
            !xr.Manager.activeLoaders.Any(loader=>loader!=null && loader.GetType().FullName=="UnityEngine.XR.OpenXR.OpenXRLoader"))
            throw new Exception("saved scene or OpenXR loader missing components");
        robot.config=JsonUtility.FromJson<TwinConfig>(robot.configAsset.text);
        if(!robot.Apply(robot.config.initial_joint_sample_rad)) throw new Exception("joint sample application failed");
        var hand=UnityEngine.Object.FindObjectOfType<TwinHand>();
        if(hand==null || !hand.Apply(new float[6]))throw new Exception("complete DH116 visual missing");
        RenderPreview(Camera.main,robot.config.views[0],"unity_inside_operator.png");
        RenderPreview(Camera.main,robot.config.views[1],"unity_inside_fixed.png");
        File.WriteAllText(Path.Combine(TwinPath,"data/unity_validate_report.json"),
            "{\"result\":\"validated\",\"joint_count\":6,\"camera_modes\":4,\"read_only_receiver\":true,\"rendered_views\":2,\"openxr_standalone_loader\":true}");
        Debug.Log("[V2Twin] VALIDATE_OK saved scene readback");
    }
    static void RenderPreview(Camera camera,TwinViewConfig view,string filename) {
        camera.transform.position=TwinRobot.CadToUnity(view.position_cad_m);
        camera.transform.LookAt(TwinRobot.CadToUnity(view.look_at_cad_m));
        var target=new RenderTexture(1280,720,24);
        var image=new Texture2D(1280,720,TextureFormat.RGB24,false);
        try {
            camera.targetTexture=target;RenderTexture.active=target;camera.Render();
            image.ReadPixels(new Rect(0,0,1280,720),0,0);image.Apply();
            File.WriteAllBytes(Path.Combine(TwinPath,"figures/"+filename),image.EncodeToPNG());
        } finally {
            camera.targetTexture=null;RenderTexture.active=null;
            UnityEngine.Object.DestroyImmediate(target);UnityEngine.Object.DestroyImmediate(image);
        }
    }
    [MenuItem("V2 Twin/Build Windows player")]
    public static void BuildWindows() {
        ConfigureOpenXR(); // Rebuild existing scene; no CAD regeneration.
        string location=Path.Combine(TwinPath,"unity/Build/V2Twin.exe");
        Directory.CreateDirectory(Path.GetDirectoryName(location));
        PlayerSettings.companyName="V2ServoResearch";PlayerSettings.productName="V2Twin";
        var options=new BuildPlayerOptions{scenes=new[]{Root+"/Scenes/Workcell.unity"},
            locationPathName=location,target=BuildTarget.StandaloneWindows64,
            options=BuildOptions.Development};
        var report=BuildPipeline.BuildPlayer(options);
        if(report.summary.result!=BuildResult.Succeeded) throw new Exception("Windows player build failed: "+report.summary.result);
        File.WriteAllText(Path.Combine(TwinPath,"data/unity_player_build.json"),
            "{\"result\":\"succeeded\",\"size_bytes\":"+report.summary.totalSize+"}");
        Debug.Log("[V2Twin] PLAYER_BUILD_OK "+location);
    }
}
