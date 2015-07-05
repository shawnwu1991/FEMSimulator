
uniform highp mat4 uProjViewModelmatrix;
uniform highp mat4 uViewModelMatrix;
uniform highp mat4 uProjMatrix;
uniform highp mat4 uViewMatrix;
uniform highp mat4 uModelMatrix;

uniform highp mat3 uNormalMatrix;

attribute highp vec4 aVertex;

void main(void)
{
    gl_Position = uProjViewModelmatrix * aVertex;
}