Shader "V2Twin/TransparentDoubleSided" {
 Properties { _Color ("Color", Color) = (1,1,1,.08) }
 SubShader {
  Tags { "Queue"="Transparent" "RenderType"="Transparent" } LOD 200 Cull Off ZWrite Off
  CGPROGRAM
  #pragma surface surf Standard alpha:fade
  struct Input { float2 uv_MainTex; };
  fixed4 _Color;
  void surf(Input IN, inout SurfaceOutputStandard o) {
   o.Albedo=_Color.rgb; o.Alpha=_Color.a; o.Metallic=0; o.Smoothness=0.2;
  }
  ENDCG
 }
 Fallback "Transparent/Diffuse"
}
