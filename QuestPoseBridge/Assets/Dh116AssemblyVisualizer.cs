using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

/// <summary>
/// 完整末端：真实 CAD 快换 A/B/连接法兰 + 厂家 DH116 活动关节树。
/// 只读显示，不引用硬件 SDK。6 个主动关节跟随实测角度，5 个耦合关节按厂家
/// MJCF 多项式估计，其余为固定连接。被动关节不是独立测量值。
/// CAD 配准用于显示，绝不能作为已经完成 TCP/碰撞/世界坐标标定的证明。
/// </summary>
public sealed class Dh116AssemblyVisualizer : MonoBehaviour
{
    [Serializable] public sealed class Link { public string name, mesh; public float[] xyz, rpy; }
    [Serializable] public sealed class Rigid { public string name, mesh; }
    [Serializable] public sealed class Joint {
        public string name, type, parent, child;
        public float[] xyz, rpy, axis;
        public float lower, upper;
    }
    [Serializable] public sealed class Assembly {
        public string schema;
        public float[] mount_xyz, mount_rpy;
        public Rigid[] rigid; public Link[] links; public Joint[] joints;
    }
    private Assembly description;
    private readonly Dictionary<string, Transform> nodes = new Dictionary<string, Transform>();
    private readonly Dictionary<string, Transform> pivots = new Dictionary<string, Transform>();
    private readonly Dictionary<string, Quaternion> rotations = new Dictionary<string, Quaternion>();
    private readonly List<Material> materials = new List<Material>();
    private float lastValidFeedback = float.NegativeInfinity;
    private string status = "DH116：未连接，显示厂家参考姿态（非实测）";
    public bool ModelReady { get; private set; }
    public int LinkCount => nodes.Count;
    public int JointCount => pivots.Count;
    public string Status => Time.realtimeSinceStartup - lastValidFeedback > 0.5f && !float.IsNegativeInfinity(lastValidFeedback)
        ? "DH116：反馈过期，保持最后姿态（不能据此判断实物）" : status;
    public Transform JointPivot(string name) => pivots[name];

    private static Vector3 V(float[] a) => new Vector3(a[0], a[1], a[2]);
    private static Vector3 Position(float[] a) => JakaS5RobotVisualizer.RosVectorToUnity(V(a));
    private static Quaternion Rotation(float[] a) => JakaS5RobotVisualizer.RosRotationToUnity(V(a));

    public void Initialize()
    {
        if (ModelReady) return;
        string folder = Path.Combine(Application.streamingAssetsPath, "CompleteEndEffector");
        description = JsonUtility.FromJson<Assembly>(File.ReadAllText(Path.Combine(folder, "assembly.json")));
        if (description.schema != "jaka_dh116_assembly.v1") throw new InvalidDataException("末端模型契约不匹配");
        Material metal = Material(new Color(0.66f, 0.69f, 0.73f));
        Material dark = Material(new Color(0.12f, 0.14f, 0.17f));
        foreach (Rigid part in description.rigid)
            MeshObject(part.name, Path.Combine(folder, part.mesh), transform, part.name == "quick_change_a" ? dark : metal);
        Transform hand = new GameObject("DH116右手（被动关节为估计）").transform;
        hand.SetParent(transform, false);
        hand.localPosition = Position(description.mount_xyz);
        hand.localRotation = Rotation(description.mount_rpy);
        foreach (Link link in description.links)
        {
            Transform node = new GameObject(link.name).transform;
            node.SetParent(hand, false);
            nodes.Add(link.name, node);
            Transform mesh = MeshObject(link.name + "_visual", Path.Combine(folder, link.mesh), node,
                link.name.EndsWith("3_link") || link.name == "finger14_link" ? dark : metal);
            mesh.localPosition = Position(link.xyz); mesh.localRotation = Rotation(link.rpy);
        }
        foreach (Joint joint in description.joints)
        {
            Transform pivot = new GameObject(joint.name).transform;
            pivot.SetParent(nodes[joint.parent], false);
            pivot.localPosition = Position(joint.xyz);
            Quaternion rotation = Rotation(joint.rpy);
            pivot.localRotation = rotation;
            nodes[joint.child].SetParent(pivot, false);
            nodes[joint.child].localPosition = Vector3.zero;
            nodes[joint.child].localRotation = Quaternion.identity;
            pivots.Add(joint.name, pivot); rotations.Add(joint.name, rotation);
        }
        ModelReady = true;
        ApplyAngles(new float[6]);
    }

    public void ApplyFeedback(JakaS5RobotVisualizer.RobotStatePacket packet)
    {
        // 有手柄数据不等于有实物手反馈；不允许把 Trigger 目标假装成编码器读数。
        bool valid = packet.hand_feedback_valid && packet.hand_angles_deg != null && packet.hand_angles_deg.Length == 6;
        if (valid) foreach (float value in packet.hand_angles_deg) if (float.IsNaN(value) || float.IsInfinity(value)) valid = false;
        if (valid && (packet.hand_connected || packet.hand_simulated))
        {
            ApplyAngles(packet.hand_angles_deg);
            lastValidFeedback = Time.realtimeSinceStartup;
            status = packet.hand_simulated ? "DH116：模拟开合（不是实物反馈）" : "DH116：6轴实测角度 + 被动关节估计";
        }
        else status = "DH116：无有效实测反馈，模型保持参考/最后姿态";
        if (!string.IsNullOrEmpty(packet.hand_status)) status += " | " + packet.hand_status;
    }

    private void ApplyAngles(float[] degrees)
    {
        string[] active = { "finger11", "finger12", "finger21", "finger31", "finger41", "finger51" };
        var angles = new Dictionary<string, float>();
        for (int i = 0; i < 6; i++) angles[active[i]] = degrees[i] * Mathf.Deg2Rad;
        float thumb = angles["finger12"];
        angles["finger13"] = 0.004592f + 0.8808f * thumb + 0.469825f * thumb * thumb;
        foreach (int finger in new[] { 2, 3, 4, 5 })
        {
            float q = angles[$"finger{finger}1"];
            angles[$"finger{finger}2"] = 0.011072f + 0.9206f * q + 0.200535f * q * q;
        }
        foreach (Joint joint in description.joints)
        {
            float angle = angles.ContainsKey(joint.name) ? angles[joint.name] : 0f;
            if (joint.type == "revolute") angle = Mathf.Clamp(angle, joint.lower, joint.upper);
            // ROS→Unity 为反射变换，旋转轴是伪向量，角度需要取负。
            pivots[joint.name].localRotation = rotations[joint.name]
                * Quaternion.AngleAxis(-angle * Mathf.Rad2Deg, Position(joint.axis));
        }
    }

    private Material Material(Color color)
    {
        Material template = Resources.Load<Material>("QuestPreviewBase");
        Material result = template != null ? new Material(template) : new Material(Shader.Find("Standard"));
        result.color = color; materials.Add(result); return result;
    }
    private static Transform MeshObject(string name, string path, Transform parent, Material material)
    {
        GameObject obj = new GameObject(name, typeof(MeshFilter), typeof(MeshRenderer));
        obj.transform.SetParent(parent, false);
        obj.GetComponent<MeshFilter>().sharedMesh = JakaS5RobotVisualizer.LoadBinaryStl(path);
        obj.GetComponent<MeshRenderer>().sharedMaterial = material;
        return obj.transform;
    }
    private void OnDestroy() { foreach (Material material in materials) Destroy(material); }
}
