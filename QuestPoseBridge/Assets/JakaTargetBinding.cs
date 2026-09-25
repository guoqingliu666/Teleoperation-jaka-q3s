using System;
using UnityEngine;

/// <summary>
/// 只负责显示的相对绑定。矩阵来自Python同一离合锚点；Unity每帧使用本地XR位置。
/// 不做逆解、不调用SDK、不向机器人发送命令；黄色点不能当成机器人实测位置。
/// </summary>
public static class JakaTargetBinding
{
    [Serializable] public sealed class Binding
    {
        public string binding_id;
        public float[] anchor_tcp;
        public float[] reference_m;
        public float[] reference_rotation_xyzw;
        public float[] mapping;
        public float[] center_tcp;
        public float radius_mm;
        public bool position_only;
        public bool rotation_enabled;
        public float rotation_limit_deg;
    }
    [Serializable] public sealed class Packet
    {
        public string schema;
        public long sent_time_ns;
        public Binding binding;
    }

    private static bool Finite(float[] v, int count)
    {
        if (v == null || v.Length != count) return false;
        foreach (float x in v) if (float.IsNaN(x) || float.IsInfinity(x)) return false;
        return true;
    }

    public static bool Valid(Binding b)
    {
        if (b == null) return false;
        bool rotationValid = !b.rotation_enabled
            || (Finite(b.reference_rotation_xyzw, 4)
                && b.rotation_limit_deg >= 1f && b.rotation_limit_deg <= 10f);
        return !string.IsNullOrEmpty(b.binding_id)
            && Finite(b.anchor_tcp, 6) && Finite(b.reference_m, 3)
            && Finite(b.mapping, 9) && Finite(b.center_tcp, 6)
            && b.radius_mm >= 20f && b.radius_mm <= 200f && rotationValid;
    }

    public static float[] Target(Binding b, Vector3 handMetres, Quaternion handRotation)
    {
        if (!Valid(b) || float.IsNaN(handMetres.sqrMagnitude) || float.IsInfinity(handMetres.sqrMagnitude))
            throw new ArgumentException("显示绑定无效");
        var p = (float[])b.anchor_tcp.Clone();
        if (b.position_only)
        {
            Vector3 delta = handMetres - new Vector3(b.reference_m[0], b.reference_m[1], b.reference_m[2]);
            for (int i = 0; i < 3; i++)
                p[i] += 1000f * (b.mapping[3*i]*delta.x + b.mapping[3*i+1]*delta.y + b.mapping[3*i+2]*delta.z);
            Vector3 center = new Vector3(b.center_tcp[0], b.center_tcp[1], b.center_tcp[2]);
            Vector3 limited = center + Vector3.ClampMagnitude(new Vector3(p[0],p[1],p[2])-center, b.radius_mm);
            p[0]=limited.x; p[1]=limited.y; p[2]=limited.z;
        }
        if (b.rotation_enabled)
        {
            Quaternion reference = new Quaternion(b.reference_rotation_xyzw[0], b.reference_rotation_xyzw[1],
                                                  b.reference_rotation_xyzw[2], b.reference_rotation_xyzw[3]);
            Quaternion sourceDelta = handRotation * Quaternion.Inverse(reference);
            Matrix4x4 map = Matrix4x4.identity;
            for (int row=0; row<3; row++) for (int col=0; col<3; col++)
                map[row,col] = b.mapping[3*row+col];
            Matrix4x4 mapped = map * Matrix4x4.Rotate(sourceDelta) * map.transpose;
            Quaternion mappedQ = Quaternion.LookRotation(
                new Vector3(mapped.m02,mapped.m12,mapped.m22),
                new Vector3(mapped.m01,mapped.m11,mapped.m21));
            mappedQ = Quaternion.RotateTowards(Quaternion.identity, mappedQ, b.rotation_limit_deg);
            Matrix4x4 composed = Matrix4x4.Rotate(mappedQ) * RpyMatrix(p[3],p[4],p[5]);
            Vector3 rpy = MatrixRpy(composed);
            p[3] = Unwrap(rpy.x,p[3]); p[4] = Unwrap(rpy.y,p[4]); p[5] = Unwrap(rpy.z,p[5]);
        }
        return p;
    }

    private static float Unwrap(float value, float near)
        => value + Mathf.Round((near-value)/(2f*Mathf.PI))*2f*Mathf.PI;

    private static Matrix4x4 RpyMatrix(float rx, float ry, float rz)
    {
        float cx=Mathf.Cos(rx), sx=Mathf.Sin(rx), cy=Mathf.Cos(ry), sy=Mathf.Sin(ry), cz=Mathf.Cos(rz), sz=Mathf.Sin(rz);
        Matrix4x4 m=Matrix4x4.identity;
        m.m00=cz*cy; m.m01=cz*sy*sx-sz*cx; m.m02=cz*sy*cx+sz*sx;
        m.m10=sz*cy; m.m11=sz*sy*sx+cz*cx; m.m12=sz*sy*cx-cz*sx;
        m.m20=-sy;   m.m21=cy*sx;            m.m22=cy*cx;
        return m;
    }

    private static Vector3 MatrixRpy(Matrix4x4 m)
    {
        float ry=Mathf.Asin(Mathf.Clamp(-m.m20,-1f,1f));
        if (Mathf.Abs(Mathf.Cos(ry))>1e-7f)
            return new Vector3(Mathf.Atan2(m.m21,m.m22),ry,Mathf.Atan2(m.m10,m.m00));
        return new Vector3(Mathf.Atan2(-m.m12,m.m11),ry,0f);
    }
}
