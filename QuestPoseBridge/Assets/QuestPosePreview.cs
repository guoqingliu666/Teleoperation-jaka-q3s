using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

/// <summary>
/// VR 输入与 JAKA S5 只读数字孪生场景。
/// 手柄数据仍使用原始 XR 坐标；机器人模型只接收 Python 广播的真实关节反馈，
/// 本程序没有 JAKA SDK，也没有任何控制机器人的网络出口。
/// </summary>
public sealed class QuestPosePreview : MonoBehaviour
{
    private VrPoseUdpSender sender;
    private Camera view;
    private GameObject environment;
    private JakaS5RobotVisualizer robot;
    private TextMesh hud;
    private GameObject rightHandToolAxes;
    private readonly List<Material> materials = new List<Material>();
    private string status = "Waiting for XR samples";
    private float nextTextUpdate;
    public Camera PreviewCamera => view;
    public JakaS5RobotVisualizer Robot => robot;

    private void OnEnable()
    {
        Initialize();
        Application.onBeforeRender += BeforeRender;
    }

    public void Initialize()
    {
        if (environment != null) return;
        sender = GetComponent<VrPoseUdpSender>();
        view = Camera.main;
        if (view == null)
        {
            GameObject cameraObject = new GameObject("Main Camera", typeof(Camera));
            cameraObject.tag = "MainCamera";
            view = cameraObject.GetComponent<Camera>();
        }
        view.nearClipPlane = 0.05f;
        view.farClipPlane = 50;
        view.transform.SetPositionAndRotation(new Vector3(0, 1.6f, -1), Quaternion.identity);
        environment = new GameObject("READ_ONLY_XR_PREVIEW");
        Material floor = MakeMaterial(new Color(0.10f, 0.15f, 0.21f));
        Material grid = MakeMaterial(new Color(0.25f, 0.37f, 0.43f));
        Material white = MakeMaterial(new Color(0.9f, 0.95f, 1));
        Box("Reference floor (y=0, not a calibrated physical floor)", new Vector3(0, -0.04f, 0), new Vector3(10, 0.05f, 10), floor);
        for (int i = -10; i <= 10; i++)
        {
            Box("Grid X", new Vector3(i * 0.5f, -0.011f, 0), new Vector3(0.008f, 0.005f, 10), grid);
            Box("Grid Z", new Vector3(0, -0.011f, i * 0.5f), new Vector3(10, 0.005f, 0.008f), grid);
        }
        // 原先的青色/橙色方块只用于验证手柄追踪，现已由实际 JAKA S5 网格取代。
        Box("Forward axis", new Vector3(0, 0.005f, 1), new Vector3(0.025f, 0.012f, 2), white);
        Label("World title", "JAKA S5 + DH116  |  DIGITAL TWIN", new Vector3(-1.1f, 2.25f, 3), 0.017f, environment.transform);
        Label("World axes", "+Z FORWARD    |    GRID = 0.5 m\nDisplay placement is NOT robot/room calibration", new Vector3(-1.1f, 0.15f, 3), 0.019f, environment.transform);
        robot = GetComponent<JakaS5RobotVisualizer>();
        if (robot == null) robot = gameObject.AddComponent<JakaS5RobotVisualizer>();
        robot.Initialize(environment.transform);
        rightHandToolAxes = CreateRightHandToolAxes();
        rightHandToolAxes.SetActive(false);
        // 头锁定状态面板随摄像机移动，可在头显内直接看到掉线/失追踪。
        // 面板缩到左上侧，避免像旧版一样挡住机械臂主体；桌面镜像仍有 OnGUI 大字诊断。
        Box("HUD background", new Vector3(-0.32f, 0.405f, 0.98f), new Vector3(0.62f, 0.15f, 0.005f), floor, view.transform);
        hud = Label("XR status HUD", status, new Vector3(-0.60f, 0.465f, 0.95f), 0.0025f, view.transform);
        hud.gameObject.layer = 0;
        // 运行时几何没有烘焙光照探针，补充环境光与柔和反向光，避免掌心成为黑色剪影。
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight = new Color(0.56f, 0.59f, 0.65f);
        var fill = new GameObject("Assembly fill light", typeof(Light));
        fill.transform.SetParent(environment.transform, false);
        fill.transform.localRotation = Quaternion.Euler(25, -100, 0);
        fill.GetComponent<Light>().type = LightType.Directional;
        fill.GetComponent<Light>().intensity = 0.55f;
        Application.runInBackground = true;
        Application.targetFrameRate = 90;
    }

    private Material MakeMaterial(Color color)
    {
        // Resources 中的材质保留 Player 所需 shader；仅 Shader.Find 会被构建裁剪。
        Material template = Resources.Load<Material>("QuestPreviewBase");
        Material material = template != null ? new Material(template) : new Material(Shader.Find("Standard"));
        material.color = color;
        materials.Add(material);
        return material;
    }

    private GameObject Box(string label, Vector3 position, Vector3 scale, Material material, Transform parent = null)
    {
        GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.name = label;
        box.transform.SetParent(parent != null ? parent : environment.transform, false);
        box.transform.localPosition = position;
        box.transform.localScale = scale;
        box.GetComponent<Renderer>().sharedMaterial = material;
        // 这些只是视觉标记，不启用碰撞或物理交互。
        box.GetComponent<Collider>().enabled = false;
        return box;
    }

    private TextMesh Label(string name, string text, Vector3 position, float size, Transform parent)
    {
        GameObject label = new GameObject(name, typeof(TextMesh));
        label.transform.SetParent(parent, false);
        label.transform.localPosition = position;
        TextMesh mesh = label.GetComponent<TextMesh>();
        mesh.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        label.GetComponent<Renderer>().sharedMaterial = mesh.font.material;
        mesh.text = text;
        mesh.fontSize = 48;
        mesh.characterSize = size;
        mesh.anchor = TextAnchor.UpperLeft;
        mesh.color = Color.white;
        return mesh;
    }

    private void LateUpdate()
    {
        if (sender != null) ApplyPacket(sender.LatestPacket);
    }

    private void BeforeRender()
    {
        // 在渲染前再次更新 HMD，降低转头到画面变化的延迟。
        ApplyHead(VrPoseUdpSender.ReadController(InputDevices.GetDeviceAtXRNode(XRNode.Head)));
    }

    private static bool Usable(VrPoseUdpSender.ControllerSample sample)
    {
        return sample != null && sample.connected && sample.tracked && sample.pose_valid;
    }

    private static void SetPose(Transform target, VrPoseUdpSender.ControllerSample sample)
    {
        var p = sample.position_m;
        var q = sample.rotation_xyzw;
        target.SetPositionAndRotation(new Vector3(p.x, p.y, p.z), new Quaternion(q.x, q.y, q.z, q.w));
    }

    private void ApplyHead(VrPoseUdpSender.ControllerSample head)
    {
        if (view != null && Usable(head)) SetPose(view.transform, head);
    }

    public void ApplyPacket(VrPoseUdpSender.PosePacket packet)
    {
        if (packet == null)
        {
            status = "NO SAMPLES - start the PoseSender";
        }
        else
        {
            ApplyHead(packet.head);
            bool rightUsable = Usable(packet.right);
            rightHandToolAxes.SetActive(rightUsable);
            if (rightUsable) SetPose(rightHandToolAxes.transform, packet.right);
            robot?.ApplyHandForDisplay(packet.right);
            status = "VR INPUT + JAKA S5 DIGITAL TWIN (READ ONLY)\n" + Describe("HMD", packet.head) + "\n"
                + Describe("LEFT", packet.left) + "\n" + Describe("RIGHT", packet.right)
                + "\n" + (sender != null ? sender.TransportStatus : "Validation preview")
                + "\n" + (robot != null ? robot.Status : "JAKA model unavailable");
        }
        if (!Application.isPlaying || Time.unscaledTime >= nextTextUpdate)
        {
            hud.text = status;
            nextTextUpdate = Time.unscaledTime + 0.1f;
        }
    }

    private GameObject CreateRightHandToolAxes()
    {
        GameObject marker = new GameObject("右手柄实时工具坐标（原始XR空间）");
        marker.transform.SetParent(environment.transform, false);
        AddAxis(marker.transform, "X", Color.red, Vector3.right);
        AddAxis(marker.transform, "Y", Color.green, Vector3.up);
        AddAxis(marker.transform, "Z", Color.blue, Vector3.forward);
        GameObject origin = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        origin.name = "右手柄实时原点_青白球";
        origin.transform.SetParent(marker.transform, false);
        origin.transform.localScale = Vector3.one * 0.03f;
        origin.GetComponent<Renderer>().sharedMaterial = MakeMaterial(new Color(0.35f, 1.0f, 1.0f));
        origin.GetComponent<Collider>().enabled = false;
        return marker;
    }

    private static void AddAxis(Transform parent, string name, Color color, Vector3 direction)
    {
        GameObject axis = new GameObject(name + "轴", typeof(LineRenderer));
        axis.transform.SetParent(parent, false);
        LineRenderer line = axis.GetComponent<LineRenderer>();
        line.useWorldSpace = false;
        line.positionCount = 2;
        line.SetPosition(0, Vector3.zero);
        line.SetPosition(1, direction * 0.12f);
        line.startWidth = line.endWidth = 0.008f;
        line.startColor = line.endColor = color;
        line.material = new Material(Shader.Find("Sprites/Default"));
    }

    private static string Describe(string label, VrPoseUdpSender.ControllerSample s)
    {
        if (s == null || !s.connected) return label + ": DISCONNECTED";
        if (!Usable(s)) return label + ": CONNECTED / NOT TRACKED";
        return $"{label}: TRACKED  ({s.position_m.x:F2}, {s.position_m.y:F2}, {s.position_m.z:F2}) m"
            + $"  T:{s.trigger:F2} G:{s.grip:F2}";
    }

    private void OnGUI()
    {
        GUI.Box(new Rect(10, 10, 1100, 190), "");
        GUI.Label(new Rect(20, 18, 1080, 180), status);
    }

    private void OnDisable()
    {
        Application.onBeforeRender -= BeforeRender;
    }

    private void OnDestroy()
    {
        if (environment != null) Destroy(environment);
        if (hud != null) Destroy(hud.gameObject);
        foreach (Material material in materials) Destroy(material);
    }
}
