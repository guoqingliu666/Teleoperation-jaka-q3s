using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// JAKA S5 的只读 VR 数字孪生。
///
/// 架构说明：
/// 1. Python/JAKA SDK 仍是唯一的真机通信与安全控制端；
/// 2. Python 把控制器“实际反馈”的六个关节角广播到本机 UDP 5006；
/// 3. Unity 只收数据并驱动模型，绝不向机器人发送任何命令；
/// 4. 手柄目标只画成小坐标轴，机械臂实体模型只跟随实测关节角。
///
/// 因此，逆解失败、伺服未启动或真机没有运动时，VR 模型不会冒充真机已经运动。
/// </summary>
public sealed class JakaS5RobotVisualizer : MonoBehaviour
{
    [Serializable]
    public sealed class RobotStatePacket
    {
        public string schema;
        public string feedback_source;
        public int sequence;
        public long sent_time_ns;
        public long sample_time_ns;
        public float feedback_hz;
        public float query_ms;
        public bool connected;
        public bool robot_simulated;
        public bool powered_on;
        public bool enabled;
        public bool armed;
        public bool servo_starting;
        public bool servo_active;
        public int tool_id;
        public float[] joints_rad;
        public float[] tcp_pose_mm_rad;
        public float[] target_tcp_mm_rad;
        public string error;
        public bool hand_connected;
        public bool hand_feedback_valid;
        public bool hand_simulated;
        public float[] hand_angles_deg;
        public string hand_status;
    }

    [Header("Python 只读反馈")]
    [SerializeField] private int listenPort = 5006;

    [Header("VR 中的模型摆放（不是物理标定）")]
    // 这里只决定“在 VR 展示窗里摆在哪里”，不是手眼/世界坐标标定。抬高 0.65 m
    // 是为了让佩戴者正视时能看到完整机械臂，而不是必须低头找地面。
    [SerializeField] private Vector3 displayBasePosition = new Vector3(0f, 0.65f, 1.0f);
    [SerializeField] private float displayScale = 1f;

    // 以下参数逐项抄自厂商 jaka_s5.urdf。不要为了“看起来顺眼”修改运动学尺寸。
    private static readonly Vector3[] JointOriginRos =
    {
        new Vector3(0f, -0.00022535f, 0.12015f),
        new Vector3(0f, 0f, 0f),
        new Vector3(0.43f, 0f, 0f),
        new Vector3(0.3685f, 0f, -0.114f),
        new Vector3(0f, -0.1135f, 0f),
        new Vector3(0.00026137f, 0.1175f, 0.00025782f),
    };

    private static readonly Vector3[] JointRpyRos =
    {
        Vector3.zero,
        new Vector3(1.5708f, 0f, 0f),
        Vector3.zero,
        Vector3.zero,
        new Vector3(1.5708f, 0f, 0f),
        new Vector3(-1.5708f, 0f, 0f),
    };

    private Transform robotRoot;
    private readonly Transform[] jointPivots = new Transform[6];
    private readonly Quaternion[] jointFixedRotations = new Quaternion[6];
    private readonly Quaternion[] jointTargetRotations = new Quaternion[6];
    private bool haveJointTargets;
    private readonly List<Material> ownedMaterials = new List<Material>();
    private GameObject targetAxes;
    private GameObject measuredTcpAxes;
    private bool localBindingMode;
    private JakaTargetBinding.Binding localBinding;
    private string releasedBindingId;
    private long lastBindingStamp;
    private float bindingUpdateRealtime;
    private string bindingStatus = "黄色=手柄目标；等待②建立绑定";
    // 现场已确认：控制器 Tool 1 位于法兰 Tool 0 的局部 +Z 146 mm；操作者希望
    // VR 醒目标记位于法兰局部 +Z 200 mm。因此只给“显示标记”再加 54 mm，
    // 不修改控制器的 Tool 1，也不改变发送给 JAKA 的目标位姿。
    private const float ControllerTool1FromFlangeM = 0.146f;
    private const float RequestedMarkerFromFlangeM = 0.200f;
    public const float TargetMarkerExtraToolZM = RequestedMarkerFromFlangeM - ControllerTool1FromFlangeM;
    private UdpClient receiver;
    private Thread receiveThread;
    private volatile bool receiveRunning;
    private readonly object packetLock = new object();
    private string pendingJson;
    private string pendingBindingJson;
    private string receiveError = "";
    private float lastPacketRealtime = float.NegativeInfinity;
    private RobotStatePacket latestState;

    public bool ModelReady { get; private set; }
    public int AppliedPacketCount { get; private set; }
    public Transform RobotRoot => robotRoot;
    public Transform[] JointPivots => jointPivots;
    public Transform TargetMarker => targetAxes != null ? targetAxes.transform : null;
    public RobotStatePacket LatestState => latestState;
    public Dh116AssemblyVisualizer Hand { get; private set; }

    public string Status
    {
        get
        {
            if (!ModelReady) return "JAKA S5模型：加载失败或尚未完成";
            if (!string.IsNullOrEmpty(receiveError)) return "JAKA反馈UDP错误：" + receiveError;
            if (latestState == null) return "JAKA S5数字孪生：等待Python反馈（UDP 5006）";
            float age = Time.realtimeSinceStartup - lastPacketRealtime;
            if (age > 0.5f) return $"JAKA反馈已过期 {age:F2}s；模型保持最后实测姿态";
            string motion = latestState.servo_active ? "伺服模式" : "实测反馈";
            string fault = string.IsNullOrEmpty(latestState.error) ? "" : " | 错误=" + latestState.error;
            return (latestState.robot_simulated ? "JAKA模拟模型（非真机）：" : "JAKA实测模型：")
                + $"连接={latestState.connected} 上电={latestState.powered_on} "
                + $"使能={latestState.enabled} Tool={latestState.tool_id} {motion}{fault}\n"
                + $"采样={latestState.feedback_hz:F1}Hz 渲染={1f/Mathf.Max(Time.smoothDeltaTime,0.001f):F0}FPS 读取={latestState.query_ms:F1}ms 帧龄={age*1000:F0}ms\n"
                + bindingStatus + "\n"
                + (Hand != null ? Hand.Status : "DH116：模型尚未加载");
        }
    }

    private void OnEnable()
    {
        StartReceiver();
    }

    /// <summary>由 QuestPosePreview 创建好 VR 环境后调用一次。</summary>
    public void Initialize(Transform parent)
    {
        if (ModelReady) return;
        BuildRobot(parent);
    }

    private void Update()
    {
        string json = null;
        string bindingJson = null;
        lock (packetLock)
        {
            bindingJson = pendingBindingJson;
            pendingBindingJson = null;
            if (pendingJson != null)
            {
                json = pendingJson;
                pendingJson = null;
            }
        }
        if (bindingJson != null)
        {
            try { ApplyBindingForDisplay(JsonUtility.FromJson<JakaTargetBinding.Packet>(bindingJson)); }
            catch (Exception exception) { bindingStatus = "目标绑定错误：" + exception.Message; }
        }
        if (json != null)
        {
            try
            {
                RobotStatePacket packet = JsonUtility.FromJson<RobotStatePacket>(json);
                if (packet == null || packet.schema != "quest_jaka_robot_state.v1")
                    throw new InvalidDataException("数据契约不是 quest_jaka_robot_state.v1");
                ApplyState(packet);
                receiveError = "";
            }
            catch (Exception exception)
            {
                receiveError = exception.Message;
            }
        }
        SmoothMeasuredJoints();
    }

    private void BuildRobot(Transform parent)
    {
        robotRoot = new GameObject("JAKA S5 实测数字孪生（未做空间标定）").transform;
        robotRoot.SetParent(parent, false);
        robotRoot.localPosition = displayBasePosition;
        robotRoot.localRotation = Quaternion.identity;
        robotRoot.localScale = Vector3.one * displayScale;

        Material white = MakeMaterial(new Color(0.83f, 0.85f, 0.90f));
        Material red = MakeMaterial(new Color(0.72f, 0.055f, 0.035f));
        Material dark = MakeMaterial(new Color(0.18f, 0.20f, 0.23f));
        Material[] linkMaterials = { dark, red, white, red, white, red, dark };

        Transform currentLink = NewLink("Link_00", robotRoot, linkMaterials[0]);
        for (int jointIndex = 0; jointIndex < 6; jointIndex++)
        {
            Transform pivot = new GameObject($"joint_{jointIndex + 1}（URDF轴）").transform;
            pivot.SetParent(currentLink, false);
            pivot.localPosition = RosVectorToUnity(JointOriginRos[jointIndex]);
            jointFixedRotations[jointIndex] = RosRotationToUnity(JointRpyRos[jointIndex]);
            pivot.localRotation = jointFixedRotations[jointIndex];
            jointTargetRotations[jointIndex] = jointFixedRotations[jointIndex];
            jointPivots[jointIndex] = pivot;
            currentLink = NewLink($"Link_{jointIndex + 1:00}", pivot, linkMaterials[jointIndex + 1]);
        }

        Hand = currentLink.gameObject.AddComponent<Dh116AssemblyVisualizer>();
        Hand.Initialize();
        targetAxes = CreateTargetMarker(robotRoot);
        targetAxes.SetActive(false);
        measuredTcpAxes = CreateAxes("蓝色原点=实测Tool1 TCP（非手柄目标）", robotRoot, .10f, .008f);
        GameObject measuredOrigin = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        measuredOrigin.transform.SetParent(measuredTcpAxes.transform, false);
        measuredOrigin.transform.localScale = Vector3.one * .025f;
        measuredOrigin.GetComponent<Renderer>().sharedMaterial = MakeMaterial(new Color(.1f,.65f,1f));
        Collider measuredCollider = measuredOrigin.GetComponent<Collider>();
        if (Application.isPlaying) Destroy(measuredCollider); else DestroyImmediate(measuredCollider);
        measuredTcpAxes.SetActive(false);
        ModelReady = true;
    }

    private Transform NewLink(string name, Transform parent, Material material)
    {
        GameObject link = new GameObject(name);
        link.transform.SetParent(parent, false);
        string path = Path.Combine(Application.streamingAssetsPath, "JakaS5", name + ".STL");
        Mesh mesh = LoadBinaryStl(path);
        MeshFilter filter = link.AddComponent<MeshFilter>();
        MeshRenderer renderer = link.AddComponent<MeshRenderer>();
        filter.sharedMesh = mesh;
        renderer.sharedMaterial = material;
        return link.transform;
    }

    /// <summary>
    /// 读取 SolidWorks/URDF 导出的二进制 STL。顶点保持厂家米制尺寸；由于 ROS 到 Unity
    /// 的轴变换包含一次镜像，三角形绕序必须反转，否则模型表面会被背面剔除。
    /// </summary>
    public static Mesh LoadBinaryStl(string path)
    {
        if (!File.Exists(path)) throw new FileNotFoundException("缺少JAKA网格", path);
        using (FileStream stream = File.OpenRead(path))
        using (BinaryReader reader = new BinaryReader(stream))
        {
            reader.ReadBytes(80);
            uint triangleCount = reader.ReadUInt32();
            if (84L + triangleCount * 50L != stream.Length)
                throw new InvalidDataException("当前只支持厂家提供的二进制STL：" + path);
            int vertexCount = checked((int)triangleCount * 3);
            Vector3[] vertices = new Vector3[vertexCount];
            int[] triangles = new int[vertexCount];
            for (int triangle = 0; triangle < triangleCount; triangle++)
            {
                reader.ReadSingle(); reader.ReadSingle(); reader.ReadSingle(); // STL 法向量，统一重算。
                int first = triangle * 3;
                for (int corner = 0; corner < 3; corner++)
                {
                    Vector3 ros = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
                    vertices[first + corner] = RosVectorToUnity(ros);
                }
                reader.ReadUInt16();
                triangles[first] = first;
                triangles[first + 1] = first + 2;
                triangles[first + 2] = first + 1;
            }
            Mesh mesh = new Mesh { name = Path.GetFileNameWithoutExtension(path) + "_runtime" };
            if (vertexCount > 65535) mesh.indexFormat = IndexFormat.UInt32;
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            mesh.UploadMeshData(true);
            return mesh;
        }
    }

    private Material MakeMaterial(Color color)
    {
        Material template = Resources.Load<Material>("QuestPreviewBase");
        Material material = template != null ? new Material(template) : new Material(Shader.Find("Standard"));
        material.color = color;
        ownedMaterials.Add(material);
        return material;
    }

    /// <summary>应用真实反馈。公开验证入口也走同一函数，避免测试另一套假逻辑。</summary>
    public void ApplyStateForValidation(RobotStatePacket packet)
    {
        ApplyState(packet);
        SnapMeasuredJoints();
    }

    private void ApplyState(RobotStatePacket packet)
    {
        // ①只读与②单连接会话均可送实测关节；仅拒绝旧版 motion_session 残留包。
        if (packet.feedback_source == "motion_session") return;
        if (packet.sample_time_ns > 0)
        {
            long nowNs = (DateTime.UtcNow.Ticks - 621355968000000000L) * 100L;
            if (nowNs - packet.sample_time_ns > 150000000L || nowNs < packet.sample_time_ns)
                return; // 不为迟到的SDK返回值补一个“新鲜”到达时间。
        }
        latestState = packet;
        lastPacketRealtime = Time.realtimeSinceStartup;
        AppliedPacketCount++;
        Hand?.ApplyFeedback(packet);
        if (packet.joints_rad != null && packet.joints_rad.Length == 6)
        {
            for (int index = 0; index < 6; index++)
            {
                // ROS→Unity 轴映射的行列式为 -1，所以绕 ROS +Z 的正角对应 Unity -Y。
                float degrees = packet.joints_rad[index] * Mathf.Rad2Deg;
                jointTargetRotations[index] = jointFixedRotations[index]
                    * Quaternion.AngleAxis(-degrees, Vector3.up);
            }
            haveJointTargets = true;
        }
        // 目标标记是“操作者选中的候选终点”，并不等价于伺服已经启动。
        // 因此选点或冻结阶段也要显示；真机模型仍只使用实测 joints_rad。
        bool hasTcp = packet.tcp_pose_mm_rad != null && packet.tcp_pose_mm_rad.Length == 6;
        measuredTcpAxes.SetActive(hasTcp && packet.connected);
        if (hasTcp) PositionMarker(measuredTcpAxes, packet.tcp_pose_mm_rad, false);
        // 当前②的目标直接进入独立绑定通道；①的空target不得覆盖它。
        if (!localBindingMode)
        {
            bool showTarget = packet.target_tcp_mm_rad != null && packet.target_tcp_mm_rad.Length == 6;
            targetAxes.SetActive(showTarget);
            if (showTarget) PositionMarker(targetAxes, packet.target_tcp_mm_rad, true);
        }
    }

    private void PositionMarker(GameObject marker, float[] pose, bool palmOffset)
    {
        Vector3 raw = RosVectorToUnity(new Vector3(pose[0], pose[1], pose[2])) * .001f;
        Quaternion rotation = RosRotationToUnity(new Vector3(pose[3], pose[4], pose[5]));
        marker.transform.localPosition = raw + (palmOffset ? rotation * RosVectorToUnity(new Vector3(0,0,TargetMarkerExtraToolZM)) : Vector3.zero);
        marker.transform.localRotation = rotation;
    }

    public void ApplyBindingForDisplay(JakaTargetBinding.Packet packet)
    {
        long now = (DateTime.UtcNow.Ticks - 621355968000000000L)*100L;
        if (packet == null || packet.schema != "quest_jaka_target_binding.v1"
            || packet.sent_time_ns <= lastBindingStamp || packet.sent_time_ns > now
            || now-packet.sent_time_ns > 500000000L) return;
        if (packet.binding != null && !JakaTargetBinding.Valid(packet.binding))
            throw new InvalidDataException("目标显示契约无效");
        lastBindingStamp = packet.sent_time_ns;
        bindingUpdateRealtime = Time.realtimeSinceStartup;
        localBindingMode = true;
        if (packet.binding != null && packet.binding.binding_id == releasedBindingId) return;
        localBinding = packet.binding;
        if (localBinding == null) targetAxes.SetActive(false);
    }

    /// <summary>每渲染帧调用；不等待Python、不改变真实关节。丢追踪/松Grip立即取消本地绑定。</summary>
    public void ApplyHandForDisplay(VrPoseUdpSender.ControllerSample hand)
    {
        if (!localBindingMode || localBinding == null) return;
        if (hand == null || !hand.connected || !hand.tracked || !hand.pose_valid || hand.grip <= .55f || hand.secondary_button)
        {
            releasedBindingId = localBinding.binding_id;
            localBinding = null;
            targetAxes.SetActive(false);
            bindingStatus = "目标显示已释放；蓝色原点=实测TCP";
            return;
        }
        Quaternion handRotation = Quaternion.identity;
        if (hand.rotation_xyzw != null)
            handRotation = new Quaternion(hand.rotation_xyzw.x,hand.rotation_xyzw.y,
                                          hand.rotation_xyzw.z,hand.rotation_xyzw.w);
        else if (localBinding.rotation_enabled)
        {
            targetAxes.SetActive(false);
            bindingStatus = "手柄姿态无效；姿态影子保持关闭";
            return;
        }
        float[] target = JakaTargetBinding.Target(localBinding,
            new Vector3(hand.position_m.x,hand.position_m.y,hand.position_m.z), handRotation);
        PositionMarker(targetAxes, target, true);
        targetAxes.SetActive(true);
        float age = Time.realtimeSinceStartup-bindingUpdateRealtime;
        bindingStatus = age > .3f
            ? $"黄色仅本地显示；②更新已过期{age:F1}s，运动状态未知；蓝色=实测TCP"
            : (localBinding.rotation_enabled
                ? "黄色=本地手柄姿态影子（零运动）；蓝色=实测TCP"
                : "黄色=本地手柄位置目标（姿态保持）；蓝色=实测TCP");
    }

    private void SmoothMeasuredJoints()
    {
        if (!haveJointTargets || Time.realtimeSinceStartup - lastPacketRealtime > 0.15f) return;
        // 每个渲染帧平滑已测量姿态，15ms响应；无新数据则保持，不外推猜测真机运动。
        float blend = 1f - Mathf.Exp(-Time.unscaledDeltaTime / 0.015f);
        for (int index = 0; index < 6; index++)
            jointPivots[index].localRotation = Quaternion.Slerp(
                jointPivots[index].localRotation, jointTargetRotations[index], blend);
    }

    private void SnapMeasuredJoints()
    {
        if (!haveJointTargets) return;
        for (int index = 0; index < 6; index++)
            jointPivots[index].localRotation = jointTargetRotations[index];
    }

    /// <summary>ROS(x前,y左,z上) → Unity(x右,y上,z前)。</summary>
    public static Vector3 RosVectorToUnity(Vector3 ros) => new Vector3(-ros.y, ros.z, ros.x);

    /// <summary>用 B*R*B^-1 做完整基变换，避免直接交换欧拉角导致复合姿态错误。</summary>
    public static Quaternion RosRotationToUnity(Vector3 rpy)
    {
        float cr = Mathf.Cos(rpy.x), sr = Mathf.Sin(rpy.x);
        float cp = Mathf.Cos(rpy.y), sp = Mathf.Sin(rpy.y);
        float cy = Mathf.Cos(rpy.z), sy = Mathf.Sin(rpy.z);
        Matrix4x4 ros = Matrix4x4.identity;
        ros.m00 = cy * cp; ros.m01 = cy * sp * sr - sy * cr; ros.m02 = cy * sp * cr + sy * sr;
        ros.m10 = sy * cp; ros.m11 = sy * sp * sr + cy * cr; ros.m12 = sy * sp * cr - cy * sr;
        ros.m20 = -sp;    ros.m21 = cp * sr;                ros.m22 = cp * cr;
        Matrix4x4 basis = Matrix4x4.identity;
        basis.m00 = 0; basis.m01 = -1; basis.m02 = 0;
        basis.m10 = 0; basis.m11 = 0;  basis.m12 = 1;
        basis.m20 = 1; basis.m21 = 0;  basis.m22 = 0;
        Matrix4x4 unity = basis * ros * basis.transpose;
        Vector3 forward = new Vector3(unity.m02, unity.m12, unity.m22);
        Vector3 up = new Vector3(unity.m01, unity.m11, unity.m21);
        return Quaternion.LookRotation(forward, up);
    }

    private static GameObject CreateAxes(string name, Transform parent, float length, float width)
    {
        GameObject axes = new GameObject(name);
        axes.transform.SetParent(parent, false);
        AddAxis(axes.transform, "X", Color.red, Vector3.right, length, width);
        AddAxis(axes.transform, "Y", Color.green, Vector3.up, length, width);
        AddAxis(axes.transform, "Z", Color.blue, Vector3.forward, length, width);
        return axes;
    }

    private GameObject CreateTargetMarker(Transform parent)
    {
        // 18 cm 长、12 mm 粗的三轴比旧版更容易在 Quest 中看清；球的直径 4 cm，
        // 即用户要求的半径 2 cm。球只表示目标，不代表机器人已经到达。
        GameObject marker = CreateAxes("手柄目标：法兰上方200mm（不是实测模型）", parent, 0.18f, 0.012f);
        GameObject origin = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        origin.name = "手柄目标原点_明黄色球_半径2cm";
        origin.transform.SetParent(marker.transform, false);
        origin.transform.localScale = Vector3.one * 0.04f;
        origin.GetComponent<Renderer>().sharedMaterial = MakeMaterial(new Color(1.0f, 0.82f, 0.0f));
        Collider collider = origin.GetComponent<Collider>();
        if (collider != null)
        {
            if (Application.isPlaying) Destroy(collider);
            else DestroyImmediate(collider);
        }
        return marker;
    }

    private static void AddAxis(Transform parent, string name, Color color, Vector3 end, float length, float width)
    {
        GameObject axis = new GameObject(name + "轴", typeof(LineRenderer));
        axis.transform.SetParent(parent, false);
        LineRenderer line = axis.GetComponent<LineRenderer>();
        line.useWorldSpace = false;
        line.positionCount = 2;
        line.SetPosition(0, Vector3.zero);
        line.SetPosition(1, end * length);
        line.startWidth = line.endWidth = width;
        line.startColor = line.endColor = color;
        line.material = new Material(Shader.Find("Sprites/Default"));
    }

    private void StartReceiver()
    {
        if (receiveThread != null) return;
        string[] args = Environment.GetCommandLineArgs();
        for (int index = 0; index + 1 < args.Length; index++)
            if (args[index] == "-robot-state-port" && int.TryParse(args[index + 1], out int requested)
                && requested >= 1024 && requested <= 65535) listenPort = requested;
        try
        {
            receiver = new UdpClient(new IPEndPoint(IPAddress.Loopback, listenPort));
            receiver.Client.ReceiveTimeout = 300;
            receiveRunning = true;
            receiveThread = new Thread(ReceiveLoop) { IsBackground = true, Name = "JAKA-VR-state-UDP" };
            receiveThread.Start();
        }
        catch (Exception exception)
        {
            receiveError = exception.Message;
        }
    }

    private void ReceiveLoop()
    {
        IPEndPoint source = new IPEndPoint(IPAddress.Loopback, 0);
        while (receiveRunning)
        {
            try
            {
                byte[] bytes = receiver.Receive(ref source);
                if (!IPAddress.IsLoopback(source.Address)) continue;
                string json = Encoding.UTF8.GetString(bytes);
                lock (packetLock)
                {
                    if (json.Contains("\"quest_jaka_target_binding.v1\"")) pendingBindingJson = json;
                    else pendingJson = json;
                }
            }
            catch (SocketException exception)
            {
                if (exception.SocketErrorCode != SocketError.TimedOut && receiveRunning)
                    receiveError = exception.SocketErrorCode.ToString();
            }
            catch (ObjectDisposedException) { return; }
            catch (Exception exception) { receiveError = exception.Message; }
        }
    }

    private void OnDisable()
    {
        receiveRunning = false;
        receiver?.Close();
        receiver = null;
        if (receiveThread != null && receiveThread.IsAlive) receiveThread.Join(500);
        receiveThread = null;
    }

    private void OnDestroy()
    {
        foreach (Material material in ownedMaterials) Destroy(material);
    }
}
