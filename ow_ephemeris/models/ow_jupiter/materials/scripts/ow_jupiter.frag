#version 130

uniform sampler2D diffuseMap;
uniform float exposureMultiplier;
uniform float gammaCorrection;

// vectors in view-space
in vec3 vsNormal;
in vec3 vsVecToSun;

in vec2 uv;

out vec4 outputCol;

void main()
{
  vec3 color = texture2D(diffuseMap, uv).rgb;

  float light = clamp(dot(normalize(vsNormal), vsVecToSun) + 0.05, 0.0, 1.0);
  //light = pow(light, 0.5);

  vec3 exposedClr = color * light * exposureMultiplier;
  vec3 gammaClr = pow(exposedClr, vec3(gammaCorrection));
  outputCol = vec4(gammaClr, 1.0);  
}
