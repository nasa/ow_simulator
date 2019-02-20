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
  // OpenGL cubemaps are arranged using RenderMan's left-handed coordinate system
  // resulting in the entire map being mirrored when rendered looking out from
  // the center, so we also negate y to correct our cube texture lookups.
  vec3 wsNormal_gazebo2ogre_and_mirrored = vec3(wsNormalNormalized.x, wsNormalNormalized.z, wsNormalNormalized.y);
  // Can also use textureLod() to "blur" texture as a cheap substitute for a real irradiance environment map.
  vec3 diffuseColor = texture(cubemap, wsNormal_gazebo2ogre_and_mirrored).rgb;

  //vec3 reflectvec = reflect(-normalize(wsVecToEye), wsNormalNormalized);
  //vec3 reflectvec_gazebo2ogre_and_mirrored = vec3(reflectvec.x, reflectvec.z, reflectvec.y);
  //vec3 specularColor = texture(cubemap, reflectvec_gazebo2ogre_and_mirrored).rgb;

  //float fresnel = 1.0 - dot(vsVecToEye, vsNormalNormalized);
  //vec3 color = mix(diffuseColor, specularColor, fresnel);

  outputCol = vec4(diffuseColor, 1.0);  
}
