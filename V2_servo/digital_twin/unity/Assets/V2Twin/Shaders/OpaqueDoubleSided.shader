Shader "V2Twin/OpaqueDoubleSided" {
 Properties { _Color ("Color", Color) = (1,1,1,1) }
 SubShader {
  Tags { "RenderType"="Opaque" } LOD 200 Cull Off
  CGPROGRAM
  #pragma surface surf Standard fullforwardshadows
  struct Input { float2 uv_MainTex; };
  fixed4 _Color;
  void surf(Input IN, inout SurfaceOutputStandard o) {
   o.Albedo=_Color.rgb; o.Alpha=1; o.Metallic=0.15; o.Smoothness=0.35;
  }
  ENDCG
 }
 Fallback "Diffuse"
}
