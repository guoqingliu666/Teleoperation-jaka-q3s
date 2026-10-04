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
    public static bool CommissioningInputRequested => Array.IndexOf(Environment.GetCommandLineArgs(), "--quest-j1-input") >= 0;
    public ControllerSample LastRight { get; private set; }
    public int SentPackets { get; private set; }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void StartCommissioningInput()
    {
        if (!CommissioningInputRequested) return;
        // 这里只开启本机输入流，不拥有机器人SDK，也不授予运动权限。
        // 只读诊断优先；即使两个参数同时存在，也不能发送控制输入。
        if (QuestReadOnlyDiagnostics.Requested) {
            Debug.Log("[VrPoseUdpSender] QUEST_J1_INPUT_BLOCKED_READONLY");
            return;
        }
        Application.runInBackground = true;
        foreach (var sender in FindObjectsOfType<VrPoseUdpSender>(true)) sender.enabled = true;
        Debug.Log("[VrPoseUdpSender] QUEST_J1_INPUT_ENABLED_NO_ROBOT_SDK");
    }
    [Header("UDP destination (Python runs on the same PC during Quest Link)")]
    [SerializeField] private string host = "127.0.0.1";
    [SerializeField] private int port = 5006;

    [Header("Diagnostics")]
    [SerializeField] private bool logStatusChanges = true;

    private UdpClient udp;
    private InputDevice headDevice;
    private InputDevice leftDevice;
    private InputDevice rightDevice;
    private int sequence;
    private bool? previousLeftConnected;
    private bool? previousRightConnected;

    private void OnEnable()
    {
        if (QuestReadOnlyDiagnostics.Requested) { enabled = false; return; }
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
        LastRight = right;

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
            sequence = sequence++,
            unity_time_s = Time.realtimeSinceStartupAsDouble,
            head = head,
            left = left,
            right = right
        };

        byte[] bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));

        try
        {
            udp.Send(bytes, bytes.Length);
            SentPackets++;
        }
        catch (Exception exception)
        {
            Debug.LogError($"[VrPoseUdpSender] UDP send failed: {exception.Message}");
            enabled = false;
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
            device_name = device.isValid ? device.name : "",
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
        sample.tracking_available = trackedAvailable;
        sample.position_available = positionAvailable;
        sample.rotation_available = rotationAvailable;
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

        sample.trigger_available = device.TryGetFeatureValue(CommonUsages.trigger, out sample.trigger);
        sample.grip_available = device.TryGetFeatureValue(CommonUsages.grip, out sample.grip);

        if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 thumbstick))
        {
            sample.thumbstick = Vec2.From(thumbstick);
            sample.thumbstick_available = true;
        }

        sample.primary_available = device.TryGetFeatureValue(CommonUsages.primaryButton, out sample.primary_button);
        sample.secondary_available = device.TryGetFeatureValue(CommonUsages.secondaryButton, out sample.secondary_button);
        sample.menu_available = device.TryGetFeatureValue(CommonUsages.menuButton, out sample.menu_button);
        sample.stick_click_available = device.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out sample.thumbstick_click);

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
    }

    [Serializable]
    private sealed class PosePacket
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
        public string device_name;
        public bool tracking_available, position_available, rotation_available;
        public bool trigger_available, grip_available, thumbstick_available;
        public bool primary_available, secondary_available, menu_available, stick_click_available;
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
