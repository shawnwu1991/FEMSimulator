
attribute highp vec4 vertex;

uniform mediump mat4 projViewModelmatrix;
uniform mediump vec4 color;

varying mediump vec4 rm_color;

void main(void)
{
    if (vertex.x < 0.05 && vertex.x > -0.05)
        rm_color = vec4(1.0, 0.0, 0.0, 1.0);
    else if (vertex.z < 0.05 && vertex.z > -0.05)
        rm_color = vec4(0.0, 0.0, 1.0, 1.0);
    else
        rm_color = color;
    gl_Position = projViewModelmatrix * vertex;
}