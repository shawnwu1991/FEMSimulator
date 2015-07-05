#pragma once

#ifndef _Inlined_Shader_Code_h_
#define _Inlined_Shader_Code_h_

#define ShaderLine(x) x "\n"

#define ShaderProjViewModelMatrixHeader \
    ShaderLine("uniform highp mat4 uProjViewModelmatrix;") \
    ShaderLine("uniform highp mat4 uViewModelMatrix;") \
    ShaderLine("uniform highp mat4 uProjMatrix;") \
    ShaderLine("uniform highp mat4 uViewMatrix;") \
    ShaderLine("uniform highp mat4 uModelMatrix;") \
    ShaderLine("uniform highp mat3 uNormalMatrix;") 

#define ShaderPrecisionFloat ShaderLine("precision mediump float;")

#define ShaderLightPositionHeader ShaderLine("uniform highp vec4 uLightPos;")
#define ShaderLightMaterialHeader \
    ShaderLine("struct LightMaterial") \
    ShaderLine("{") \
    ShaderLine("    mediump vec4 ambient;") \
    ShaderLine("    mediump vec4 diffuse;") \
    ShaderLine("    mediump vec4 specular;") \
    ShaderLine("};") \
    ShaderLine("uniform LightMaterial uLightMaterial;") 

#endif