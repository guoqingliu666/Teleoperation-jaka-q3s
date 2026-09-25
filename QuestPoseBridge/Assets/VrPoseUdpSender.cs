using System;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

/// <summary>
/// Reads the left and right OpenXR controller states through Unity XR and sends
/// them to a local Python process as UTF-8 JSON over UDP.
///
/// Coordinate convention: Unity tracking space, metres, +X right, +Y up,
/// +Z forward. Rotations are quaternions in x/y/z/w order.
/// </summary>
public sealed class VrPoseUdpSender : MonoBehaviour
{
    [Header("UDP destination (Python runs on the same PC during Quest Link)")]
    [SerializeField] private string host = "127.0.0.1";
    [SerializeField] private int port = 5005;

    [Header("Diagnostics")]
    [SerializeField] private bool logStatusChanges = true;
    [SerializeField, Range(10, 120)] private int sendRateHz = 60;

    public PosePacket LatestPacket { get; private set; }
    public int SentPackets { get; private set; }
    public string TransportStatus { get; private set; } = "Starting";
    private double nextSendTime;
    private double nextWarningTime;

    private UdpClient udp;
    private InputDevice headDevice;
    private InputDevice leftDevice;
    private InputDevice rightDevice;
    private int sequence;
    private bool? previousLeftConnected;
    private bool? previousRightConnected;

    private void OnEnable()
    {
        // 验收可使用独立端口，防止与另一份 Unity/GUI 混流；正常启动仍为 5005。
        string[] args = Environment.GetCommandLineArgs();
        for (int i = 0; i + 1 < args.Length; i++)
            if (args[i] == "-pose-port" && int.TryParse(args[i + 1], out int requestedPort)
                && requestedPort >= 1024 && requestedPort <= 65535) port = requestedPort;
        // 可视化只读取 XR 数据，不拥有任何机器人接口。
        if (GetComponent<QuestPosePreview>() == null)
            gameObject.AddComponent<QuestPosePreview>();
        previousLeftConnected = previousRightConnected = null;
        nextSendTime = 0;
        sequence = SentPackets = 0;
        try
        {
            udp = new UdpClient();
            udp.Connect(host, port);
            Debug.Log($"[VrPoseUdpSender] Sending to udp://{host}:{port}");
        }
        catch (Exception exception)
        {
            Debug.LogError($"[VrPoseUdpSender] Could not open UDP sender: {exception.Message}");
            enabled = false;
        }
    }

    private void Update()
    {
        EnsureDevice(ref headDevice, XRNode.Head);
        EnsureDevice(ref leftDevice, XRNode.LeftHand);
        EnsureDevice(ref rightDevice, XRNode.RightHand);

        ControllerSample head = ReadController(headDevice);
        ControllerSample left = ReadController(leftDevice);
        ControllerSample right = ReadController(rightDevice);

        if (logStatusChanges)
        {
            LogConnectionChange("left", left.connected, ref previousLeftConnected);
            LogConnectionChange("right", right.connected, ref previousRightConnected);
        }

        PosePacket packet = new PosePacket
        {
            // Version 2 adds the HMD pose.  The Python receiver accepts both
            // versions, while v2 lets the operator lock the VR forward axis
            // before arming robot motion.
            version = 2,
            sequence = sequence,
            unity_time_s = Time.realtimeSinceStartupAsDouble,
            head = head,
            left = left,
            right = right
        };

        LatestPacket = packet;
        // 每帧采样，按固定上限发送，避免空场景几千 FPS 淹没 Python。
        double now = Time.realtimeSinceStartupAsDouble;
        if (now < nextSendTime) return;
        nextSendTime = now + 1.0 / Mathf.Clamp(sendRateHz, 10, 120);
        sequence++;

        byte[] bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));

        try
        {
            udp.Send(bytes, bytes.Length);
            SentPackets++;
            TransportStatus = $"UDP {host}:{port} | sent {SentPackets} (not an ACK)";
        }
        catch (SocketException exception)
        {
            // Python 尚未启动/重启时不永久关闭采集，下一帧自动重试。
            TransportStatus = "UDP retry: " + exception.SocketErrorCode;
            if (now >= nextWarningTime)
            {
                Debug.LogWarning($"[VrPoseUdpSender] {TransportStatus}; start Python demo receiver.");
                nextWarningTime = now + 3;
            }
        }
    }

    private static void EnsureDevice(ref InputDevice device, XRNode node)
    {
        if (!device.isValid)
        {
            device = InputDevices.GetDeviceAtXRNode(node);
        }
    }

    public static ControllerSample ReadController(InputDevice device)
    {
        ControllerSample sample = new ControllerSample
        {
            connected = device.isValid,
            position_m = new Vec3(),
            rotation_xyzw = new Quat(),
            linear_velocity_m_s = new Vec3(),
            angular_velocity_rad_s = new Vec3(),
            thumbstick = new Vec2()
        };

        if (!device.isValid)
        {
            return sample;
        }

        bool trackedAvailable = device.TryGetFeatureValue(CommonUsages.isTracked, out bool tracked);
        bool positionAvailable = device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position);
        bool rotationAvailable = device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation);

        sample.tracked = trackedAvailable && tracked;
        sample.pose_valid = sample.tracked && positionAvailable && rotationAvailable;

        if (positionAvailable)
        {
            sample.position_m = Vec3.From(position);
        }

        if (rotationAvailable)
        {
            sample.rotation_xyzw = Quat.From(rotation);
        }

        if (device.TryGetFeatureValue(CommonUsages.deviceVelocity, out Vector3 linearVelocity))
        {
            sample.linear_velocity_m_s = Vec3.From(linearVelocity);
        }

        if (device.TryGetFeatureValue(CommonUsages.deviceAngularVelocity, out Vector3 angularVelocity))
        {
            sample.angular_velocity_rad_s = Vec3.From(angularVelocity);
        }

        device.TryGetFeatureValue(CommonUsages.trigger, out sample.trigger);
        device.TryGetFeatureValue(CommonUsages.grip, out sample.grip);

        if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 thumbstick))
        {
            sample.thumbstick = Vec2.From(thumbstick);
        }

        device.TryGetFeatureValue(CommonUsages.primaryButton, out sample.primary_button);
        device.TryGetFeatureValue(CommonUsages.secondaryButton, out sample.secondary_button);
        device.TryGetFeatureValue(CommonUsages.menuButton, out sample.menu_button);
        device.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out sample.thumbstick_click);

        return sample;
    }

    private static void LogConnectionChange(string side, bool connected, ref bool? previous)
    {
        if (!previous.HasValue || previous.Value != connected)
        {
            Debug.Log($"[VrPoseUdpSender] {side} controller connected={connected}");
            previous = connected;
        }
    }

    private void OnDisable()
    {
        udp?.Close();
        udp = null;
        LatestPacket = null;
    }

    [Serializable]
    public sealed class PosePacket
    {
        public int version;
        public int sequence;
        public double unity_time_s;
        public ControllerSample head;
        public ControllerSample left;
        public ControllerSample right;
    }

    [Serializable]
    public sealed class ControllerSample
    {
        public bool connected;
        public bool tracked;
        public bool pose_valid;
        public Vec3 position_m;
        public Quat rotation_xyzw;
        public Vec3 linear_velocity_m_s;
        public Vec3 angular_velocity_rad_s;
        public float trigger;
        public float grip;
        public Vec2 thumbstick;
        public bool primary_button;
        public bool secondary_button;
        public bool menu_button;
        public bool thumbstick_click;
    }

    [Serializable]
    public sealed class Vec2
    {
        public float x;
        public float y;

        public static Vec2 From(Vector2 value)
        {
            return new Vec2 { x = value.x, y = value.y };
        }
    }

    [Serializable]
    public sealed class Vec3
    {
        public float x;
        public float y;
        public float z;

        public static Vec3 From(Vector3 value)
        {
            return new Vec3 { x = value.x, y = value.y, z = value.z };
        }
    }

    [Serializable]
    public sealed class Quat
    {
        public float x;
        public float y;
        public float z;
        public float w;

        public static Quat From(Quaternion value)
        {
            return new Quat { x = value.x, y = value.y, z = value.z, w = value.w };
        }
    }
}
