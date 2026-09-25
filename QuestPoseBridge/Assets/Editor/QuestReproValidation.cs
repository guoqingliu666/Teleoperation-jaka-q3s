using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class QuestReproValidation
{
    // 自动化修补 Player 时只编译/构建，不调用 Camera.Render；后者在 Windows
    // -batchmode/-nographics 组合下会触发 Unity 2022 的原生渲染崩溃。
    public static void BuildOnly()
    {
        ValidateMeasuredFeedback();
        string root = Directory.GetParent(Application.dataPath).Parent.FullName;
        PlayerSettings.productName = "JAKA S5 VR 数字孪生 - 只读反馈";
        PlayerSettings.runInBackground = true;
        PlayerSettings.resizableWindow = true;
        PlayerSettings.defaultScreenWidth = 1280;
        PlayerSettings.defaultScreenHeight = 800;
        string player = Path.Combine(root, "Player", "QuestPosePreview.exe");
        string[] commandArgs = Environment.GetCommandLineArgs();
        for (int i=0; i+1<commandArgs.Length; i++)
            if (commandArgs[i] == "-staged-player") player = Path.GetFullPath(commandArgs[i+1]);
        Directory.CreateDirectory(Path.GetDirectoryName(player));
        BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
            scenes = new[] { "Assets/Scenes/SampleScene.unity" },
            locationPathName = player,
            target = BuildTarget.StandaloneWindows64,
            options = BuildOptions.Development
        });
        if (report.summary.result != BuildResult.Succeeded)
            throw new Exception("Player build failed: " + report.summary.result);
        File.WriteAllText(Path.Combine(root, "Validation", "unity_build_only.txt"),
            "PASS: Windows Player build without synthetic camera rendering.\n");
        Debug.Log("QUEST_REPRO_BUILD_ONLY_PASS " + player);
    }

    // 只在编辑器内注入显示测试数据，不登录SDK、不发送手柄或运动命令。
    // 不用Camera.Render，验证同一个ApplyState入口能逐帧应用、拒绝旧源和迟到包。
    public static void ValidateMeasuredFeedback()
    {
        GameObject holder = new GameObject("离线反馈验证");
        try
        {
            JakaS5RobotVisualizer view = holder.AddComponent<JakaS5RobotVisualizer>();
            view.Initialize(holder.transform);
            var packet = new JakaS5RobotVisualizer.RobotStatePacket {
                schema = "quest_jaka_robot_state.v1", feedback_source = "readonly_bridge",
                connected = true, robot_simulated = true, joints_rad = new float[6], tool_id = 1
            };
            for (int i = 0; i < 60; ++i)
            {
                packet.joints_rad[0] = i * 0.001f;
                packet.sample_time_ns = (DateTime.UtcNow.Ticks - 621355968000000000L) * 100L;
                view.ApplyStateForValidation(packet);
                if (view.AppliedPacketCount != i + 1) throw new Exception("中间反馈帧未应用");
            }
            Quaternion before = view.JointPivots[0].localRotation;
            packet.joints_rad[0] = 1f;
            packet.feedback_source = "motion_session";
            view.ApplyStateForValidation(packet);
            if (view.AppliedPacketCount != 60 || Quaternion.Angle(before, view.JointPivots[0].localRotation) > 0.001f)
                throw new Exception("旧运动来源污染显示");
            packet.feedback_source = "readonly_bridge";
            packet.sample_time_ns -= 1000000000L;
            view.ApplyStateForValidation(packet);
            if (view.AppliedPacketCount != 60) throw new Exception("迟到包被当成新反馈");
            var binding = new JakaTargetBinding.Binding {
                binding_id="offline-one", anchor_tcp=new float[]{400,100,300,0,0,0},
                reference_m=new float[]{0,1,0}, mapping=new float[]{0,0,1,-1,0,0,0,1,0},
                center_tcp=new float[]{400,100,300,0,0,0}, radius_mm=100, position_only=true
            };
            var envelope = new JakaTargetBinding.Packet {
                schema="quest_jaka_target_binding.v1", binding=binding,
                sent_time_ns=(DateTime.UtcNow.Ticks-621355968000000000L)*100L
            };
            view.ApplyBindingForDisplay(envelope);
            var hand = new VrPoseUdpSender.ControllerSample {
                connected=true, tracked=true, pose_valid=true, grip=1,
                position_m=VrPoseUdpSender.Vec3.From(new Vector3(0,1,0)),
                rotation_xyzw=VrPoseUdpSender.Quat.From(Quaternion.identity)
            };
            view.ApplyHandForDisplay(hand);
            Vector3 firstTarget = view.TargetMarker.localPosition;
            Quaternion firstJoint = view.JointPivots[0].localRotation;
            // 没有任何新SDK帧，移动本地手柄仍逐帧更新黄色目标；真实关节不动。
            hand.position_m.y=1.04f;
            view.ApplyHandForDisplay(hand);
            if (Mathf.Abs(Vector3.Distance(firstTarget,view.TargetMarker.localPosition)-.04f)>.0001f
                || Quaternion.Angle(firstJoint,view.JointPivots[0].localRotation)>.001f)
                throw new Exception("本地目标显示依赖SDK或污染真实关节");
            hand.grip=0;
            view.ApplyHandForDisplay(hand);
            System.Threading.Thread.Sleep(20); // 编辑器DateTime.UtcNow分辨率可能低于渲染节拍。
            envelope.sent_time_ns=(DateTime.UtcNow.Ticks-621355968000000000L)*100L;
            view.ApplyBindingForDisplay(envelope);
            hand.grip=1;
            view.ApplyHandForDisplay(hand);
            if(view.TargetMarker.gameObject.activeSelf) throw new Exception("旧绑定在重新握持时复活");
            binding.binding_id="offline-two";
            System.Threading.Thread.Sleep(20);
            envelope.sent_time_ns=(DateTime.UtcNow.Ticks-621355968000000000L)*100L;
            view.ApplyBindingForDisplay(envelope);
            view.ApplyHandForDisplay(hand);
            if(!view.TargetMarker.gameObject.activeSelf) throw new Exception("新绑定不能恢复显示");
            // 姿态只读绑定：位置保持，黄色工具轴随手柄旋转；不会改变实测关节。
            binding.binding_id="offline-rotation";
            binding.position_only=false;
            binding.rotation_enabled=true;
            binding.rotation_limit_deg=10f;
            binding.reference_rotation_xyzw=new float[]{0,0,0,1};
            System.Threading.Thread.Sleep(20);
            envelope.sent_time_ns=(DateTime.UtcNow.Ticks-621355968000000000L)*100L;
            view.ApplyBindingForDisplay(envelope);
            Vector3 rotationPosition=view.TargetMarker.localPosition;
            Quaternion rotationBefore=view.TargetMarker.localRotation;
            hand.rotation_xyzw=VrPoseUdpSender.Quat.From(Quaternion.AngleAxis(6f,Vector3.up));
            float[] rotationTarget=JakaTargetBinding.Target(binding,new Vector3(0,1,0),Quaternion.AngleAxis(6f,Vector3.up));
            view.ApplyHandForDisplay(hand);
            float markerMove=Vector3.Distance(rotationPosition,view.TargetMarker.localPosition);
            float markerAngle=Quaternion.Angle(rotationBefore,view.TargetMarker.localRotation);
            float jointAngle=Quaternion.Angle(firstJoint,view.JointPivots[0].localRotation);
            // TCP坐标保持；掌面可视点因沿工具Z轴偏置，旋转时会绕TCP移动，这是正确几何关系。
            float tcpMove=Vector3.Distance(new Vector3(rotationTarget[0],rotationTarget[1],rotationTarget[2]),
                                           new Vector3(binding.anchor_tcp[0],binding.anchor_tcp[1],binding.anchor_tcp[2]));
            if(tcpMove>.0001f || markerAngle<4f || jointAngle>.001f)
                throw new Exception($"姿态只读影子未独立旋转或污染实测关节: palmMove={markerMove}, tcpMove={tcpMove}, marker={markerAngle}, joint={jointAngle}");
            Debug.Log("QUEST_LOCAL_TARGET_PASS: independent target, release latch, rebind; no SDK/no physical commands");
            Debug.Log("QUEST_MEASURED_FEEDBACK_VALIDATION_PASS: 60 intermediate frames; old source/stale rejected");
        }
        finally { UnityEngine.Object.DestroyImmediate(holder); }
    }

    public static void ValidateAndBuild()
    {
        string root = Directory.GetParent(Application.dataPath).Parent.FullName;
        string output = Path.Combine(root, "Validation");
        Directory.CreateDirectory(output);
        if (!AssetDatabase.IsValidFolder("Assets/Resources"))
            AssetDatabase.CreateFolder("Assets", "Resources");
        if (AssetDatabase.LoadAssetAtPath<Material>("Assets/Resources/QuestPreviewBase.mat") == null)
        {
            AssetDatabase.CreateAsset(new Material(Shader.Find("Standard")), "Assets/Resources/QuestPreviewBase.mat");
            AssetDatabase.SaveAssets();
        }
        EditorSceneManager.OpenScene("Assets/Scenes/SampleScene.unity");
        var sender = UnityEngine.Object.FindObjectOfType<VrPoseUdpSender>();
        if (sender == null) throw new Exception("PoseSender missing");
        // 仅编辑器验收注入模拟姿态和机器人反馈，正式 Player 不包含本文件。
        var preview = sender.GetComponent<QuestPosePreview>();
        if (preview == null) preview = sender.gameObject.AddComponent<QuestPosePreview>();
        preview.Initialize();
        var packet = new VrPoseUdpSender.PosePacket {
            version = 2, sequence = 1,
            head = Sample(0, 1.6f, -1),
            left = Sample(-0.3f, 1.2f, -0.3f),
            right = Sample(0.3f, 1.2f, -0.3f)
        };
        preview.ApplyPacket(packet);
        Check(Vector3.Distance(preview.PreviewCamera.transform.position, new Vector3(0, 1.6f, -1)) < 0.001f, "HMD camera pose");
        Check(preview.Robot != null && preview.Robot.ModelReady, "JAKA S5 runtime model loaded");
        Check(preview.Robot.JointPivots.Length == 6, "Six URDF joint pivots");
        Check(preview.Robot.Hand.ModelReady && preview.Robot.Hand.LinkCount == 17
            && preview.Robot.Hand.JointCount == 16, "Complete CAD couplers and articulated DH116 loaded");
        Check(GameObject.Find("Near cyan cube") == null && GameObject.Find("Far orange pillar") == null,
            "Legacy cyan/orange blocks removed");
        Quaternion oldJoint2 = preview.Robot.JointPivots[1].localRotation;
        var robotPacket = new JakaS5RobotVisualizer.RobotStatePacket {
            schema = "quest_jaka_robot_state.v1", sequence = 1,
            connected = true, powered_on = true, enabled = true, armed = true, robot_simulated = true,
            servo_active = true, tool_id = 1,
            // 使用现场截图中的一组真实关节姿态做渲染夹具，不向控制器发送。
            joints_rad = new[] { 0.0222f, 1.5603f, -1.1872f, -1.2438f, 4.7094f, -0.9134f },
            tcp_pose_mm_rad = new[] { 334.019f, 122.106f, 520.300f, 2.659f, 0.534f, 2.375f },
            target_tcp_mm_rad = new[] { 350f, 140f, 540f, 2.659f, 0.534f, 2.375f },
            error = "", hand_simulated = true, hand_feedback_valid = true,
            hand_angles_deg = new float[6], hand_status = "OFFLINE VALIDATION"
        };
        preview.Robot.ApplyStateForValidation(robotPacket);
        preview.ApplyPacket(packet); // 让截图中的 HUD 同步显示刚注入的机器人状态。
        Check(preview.Robot.AppliedPacketCount == 1, "Robot feedback packet applied");
        Check(GameObject.Find("手柄目标原点_明黄色球_半径2cm") != null,
            "Bright yellow 2 cm-radius target sphere created");
        Check(preview.Robot.TargetMarker != null && preview.Robot.TargetMarker.gameObject.activeSelf,
            "Long thick target axes visible whenever a candidate target exists");
        Vector3 rawTool1Position = JakaS5RobotVisualizer.RosVectorToUnity(new Vector3(
            robotPacket.target_tcp_mm_rad[0], robotPacket.target_tcp_mm_rad[1],
            robotPacket.target_tcp_mm_rad[2])) * 0.001f;
        Check(Mathf.Abs(Vector3.Distance(preview.Robot.TargetMarker.localPosition, rawTool1Position)
              - JakaS5RobotVisualizer.TargetMarkerExtraToolZM) < 0.002f,
            "Target marker is flange +200 mm (Tool1 +54 mm display offset)");
        Check(Quaternion.Angle(oldJoint2, preview.Robot.JointPivots[1].localRotation) > 20f,
            "Measured joint angle drives model");
        string fixture = JsonUtility.ToJson(packet);
        File.WriteAllText(Path.Combine(output, "unity_v2_fixture.json"), fixture);
        File.WriteAllText(Path.Combine(output, "jaka_robot_state_fixture.json"), JsonUtility.ToJson(robotPacket));
        Camera camera = preview.PreviewCamera;
        // 单独拍摄整个装配链，验收图明确使用模拟关节，不连接现场机器人。
        camera.transform.position = new Vector3(-1.0f, 1.35f, 0.05f);
        camera.transform.LookAt(preview.Robot.RobotRoot.position + new Vector3(-0.15f, 0.36f, 0));
        Capture(camera, Path.Combine(output, "完整数字孪生_张手.png"));
        Quaternion handOpen = preview.Robot.Hand.JointPivot("finger21").localRotation;
        robotPacket.hand_angles_deg = new[] { 15f, 22f, 55f, 55f, 55f, 55f };
        preview.Robot.ApplyStateForValidation(robotPacket);
        Check(Quaternion.Angle(handOpen, preview.Robot.Hand.JointPivot("finger21").localRotation) > 45f,
            "DH116 active encoder angle drives articulated finger");
        Capture(camera, Path.Combine(output, "完整数字孪生_闭手.png"));
        preview.ApplyPacket(packet);
        RenderTexture render = new RenderTexture(1440, 900, 24);
        camera.stereoTargetEye = StereoTargetEyeMask.None;
        camera.targetTexture = render;
        camera.Render();
        RenderTexture.active = render;
        Texture2D png = new Texture2D(1440, 900, TextureFormat.RGB24, false);
        png.ReadPixels(new Rect(0, 0, 1440, 900), 0, 0);
        png.Apply();
        File.WriteAllBytes(Path.Combine(output, "preview_synthetic.png"), png.EncodeToPNG());
        camera.targetTexture = null;
        RenderTexture.active = null;
        UnityEngine.Object.DestroyImmediate(render);
        UnityEngine.Object.DestroyImmediate(png);
        // 不保存测试对象；构建重新加载原始场景，运行时才创建可视化。
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        PlayerSettings.productName = "JAKA S5 VR 数字孪生 - 只读反馈";
        PlayerSettings.runInBackground = true;
        PlayerSettings.resizableWindow = true;
        PlayerSettings.defaultScreenWidth = 1280;
        PlayerSettings.defaultScreenHeight = 800;
        string player = Path.Combine(root, "Player", "QuestPosePreview.exe");
        Directory.CreateDirectory(Path.GetDirectoryName(player));
        BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
            scenes = new[] { "Assets/Scenes/SampleScene.unity" },
            locationPathName = player,
            target = BuildTarget.StandaloneWindows64,
            options = BuildOptions.Development
        });
        if (report.summary.result != BuildResult.Succeeded)
            throw new Exception("Player build failed: " + report.summary.result);
        File.WriteAllText(Path.Combine(output, "unity_validation.txt"),
            "PASS: camera pose, JAKA S5 STL loading, six URDF joints, measured-joint animation, v2 serialization.\n"
            + "PASS: complete CAD couplers/flange, DH116 17 links / 16 joints, finger animation, open/closed renders.\n"
            + "PASS: legacy cyan/orange controller blocks removed.\n"
            + "PASS: target marker uses a bright yellow 2 cm-radius sphere, long/thick axes, and flange +200 mm placement.\n"
            + "PASS: Windows Player build.\nSynthetic preview only; headset motion still requires human acceptance.\n");
        Debug.Log("QUEST_REPRO_VALIDATION_PASS " + player);
    }

    private static void Capture(Camera camera, string path)
    {
        var render = new RenderTexture(1600, 1100, 24);
        camera.stereoTargetEye = StereoTargetEyeMask.None;
        camera.targetTexture = render; camera.Render(); RenderTexture.active = render;
        var png = new Texture2D(1600, 1100, TextureFormat.RGB24, false);
        png.ReadPixels(new Rect(0, 0, 1600, 1100), 0, 0); png.Apply();
        File.WriteAllBytes(path, png.EncodeToPNG());
        camera.targetTexture = null; RenderTexture.active = null;
        UnityEngine.Object.DestroyImmediate(render); UnityEngine.Object.DestroyImmediate(png);
    }

    private static VrPoseUdpSender.ControllerSample Sample(float x, float y, float z)
    {
        return new VrPoseUdpSender.ControllerSample {
            connected = true, tracked = true, pose_valid = true,
            position_m = new VrPoseUdpSender.Vec3 { x = x, y = y, z = z },
            rotation_xyzw = new VrPoseUdpSender.Quat { w = 1 },
            linear_velocity_m_s = new VrPoseUdpSender.Vec3(),
            angular_velocity_rad_s = new VrPoseUdpSender.Vec3(),
            thumbstick = new VrPoseUdpSender.Vec2()
        };
    }

    private static void Check(bool value, string test)
    {
        if (!value) throw new Exception("FAILED: " + test);
        Debug.Log("PASS: " + test);
    }
}
