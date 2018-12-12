#version 130

uniform samplerCube cubemap;
uniform float exposureMultiplier;
uniform float gammaCorrection;

in vec3 wsNormal;
in vec3 wsVecToEye;
in vec3 vsPos;
in vec3 vsNormal;

out vec4 outputCol;

void main()
{
  vec3 wsNormalNormalized = normalize(wsNormal);
  vec3 vsNormalNormalized = normalize(vsNormal);
  vec3 vsVecToEye = normalize(-vsPos);

  // Gazebo is z-up but Ogre is y-up. Must rotate before cube texture lookup.
  vec3 wsNormal_gazebo2ogre = vec3(wsNormalNormalized.x, wsNormalNormalized.z, -wsNormalNormalized.y);
  // Using textureLod() to "blur" texture as a substitute for a real irradiance environment map.
  vec3 diffuseColor = textureLod(cubemap, wsNormal_gazebo2ogre, 5.0).rgb;

  vec3 reflectvec = reflect(-normalize(wsVecToEye), wsNormalNormalized);
  vec3 reflectvec_gazebo2ogre = vec3(reflectvec.x, reflectvec.z, -reflectvec.y);
  vec3 specularColor = texture(cubemap, reflectvec_gazebo2ogre).rgb;

  float fresnel = 1.0 - dot(vsVecToEye, vsNormalNormalized);
  vec3 color = mix(diffuseColor, specularColor, fresnel);

  outputCol = vec4(color, 1.0);  
}
