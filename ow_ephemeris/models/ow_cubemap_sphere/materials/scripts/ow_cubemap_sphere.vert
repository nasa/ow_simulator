#version 130

// params bound by Ogre
in vec4 position;
in vec3 normal;

uniform mat4 worldMatrix;
uniform mat4 worldViewMatrix;
uniform mat4 worldViewProjMatrix;
uniform mat4 inverseViewMatrix;
uniform mat4 inverseTransposeWorldMatrix;
uniform mat4 inverseTransposeWorldViewMatrix;

out vec3 wsNormal;
out vec3 wsVecToEye;
out vec3 vsPos;
out vec3 vsNormal;

void main()
{
  vsPos = vec3(worldViewMatrix * position);

  vec3 wsPos = vec3(worldMatrix * position);
  wsNormal = mat3(inverseTransposeWorldMatrix) * normal;
  wsVecToEye = vec3(inverseViewMatrix[3]) - wsPos;
  vsNormal = mat3(inverseTransposeWorldViewMatrix) * normal;

  gl_Position = worldViewProjMatrix * position;
}
