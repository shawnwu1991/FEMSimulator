
attribute highp vec4 aVertex;
attribute highp vec3 aNormal;

uniform highp mat4 uProjViewModelmatrix;
uniform highp mat4 uViewModelMatrix;
uniform highp mat4 uProjMatrix;
uniform highp mat4 uViewMatrix;
uniform highp mat4 uModelMatrix;

uniform highp mat3 uNormalMatrix;

uniform highp vec4 uLightPos;

varying mediump vec3 vNormal;
varying mediump vec3 vHalfVector;
varying mediump vec3 vLightDir;

void main(void)
{
    gl_Position = uProjViewModelmatrix * aVertex;
    highp vec4 eyeLightPos = uViewMatrix * uLightPos;
    highp vec4 eyeVertex = uViewModelMatrix * aVertex;

    vNormal = (uViewModelMatrix * vec4(aNormal, 0.0)).xyz;

    //if (eyeLightPos.w == 0.0)
    //{
        vLightDir = normalize(eyeLightPos.xyz);
    //}
    //else
    //{
    //    vLightDir = normalize(eyeLightPos.xyz/eyeLightPos.w - eyeVertex.xyz/eyeVertex.w);
    //}
    vHalfVector = normalize(-normalize(eyeVertex.xyz) + vLightDir);
}