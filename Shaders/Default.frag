precision mediump float;

struct LightMaterial
{
    mediump vec4 ambient;
    mediump vec4 diffuse;
    mediump vec4 specular;
};
uniform LightMaterial uLightMaterial;

struct ObjectMaterial
{
    mediump vec4 diffuse;
    mediump vec4 specular;
    mediump float shininess;
};
uniform ObjectMaterial uObjectMaterial;

varying mediump vec3 vNormal;
varying mediump vec3 vHalfVector;
varying mediump vec3 vLightDir;

void main(void)
{    
    mediump vec3 n,l,halfV;  
    mediump float NdotL,NdotHV;  

    ///* The ambient term will always be present */  
    vec4 color = uObjectMaterial.diffuse * uLightMaterial.ambient;

    ///* a fragment shader can't write a varying variable, hence we need 
    //   a new variable to store the normalized interpolated normal */  
    n = normalize(vNormal);  
    l = normalize(vLightDir);

    ///* compute the dot product between normal and ldir */  
    NdotL = dot(n, l);
        
    if (NdotL > 0.0)  
    {  
        color += uObjectMaterial.diffuse * uLightMaterial.diffuse * NdotL;  
        halfV = normalize(vHalfVector);  

        NdotHV = max(dot(n,halfV),0.0);  

        color += uObjectMaterial.specular *  
                uLightMaterial.specular *  
                pow(NdotHV, uObjectMaterial.shininess);  
    }
  
    color = clamp(color, 0.0, 1.0);
    color.w = uObjectMaterial.diffuse.w;
    gl_FragColor = color; 
}