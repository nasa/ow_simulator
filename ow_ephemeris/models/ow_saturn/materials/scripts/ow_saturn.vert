#version 130

// params bound by ogre
in vec4 position;
in vec2 uv0;
in vec3 normal;

uniform mat4 worldViewProjMatrix;
uniform mat4 inverseTransposeWorldViewMatrix;
uniform mat4 inverseTransposeViewMatrix;

uniform vec4 sunPosition;

// vectors in view-space
out vec3 vsNormal;
out vec3 vsVecToSun;

out vec2 uv;

//*********************************************************
void main()
{
  vsNormal = mat3(inverseTransposeWorldViewMatrix) * normal;

  vsVecToSun = normalize(mat3(inverseTransposeViewMatrix) * sunPosition.xyz);

  uv = uv0;

  gl_Position = worldViewProjMatrix * position;
}
