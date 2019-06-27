#version 130

// variable prefixes to indicate coordinate systems:
// os = object space
// ws = world space
// vs = view space
// ls = light space

// params bound by Ogre
in vec4 position;
in vec3 normal;
in vec2 uv0;

uniform mat4 worldMatrix;
uniform mat4 worldViewMatrix;
uniform mat4 worldViewProjMatrix;
uniform mat4 inverseViewMatrix;
uniform mat4 inverseTransposeWorldMatrix;
uniform mat4 inverseTransposeWorldViewMatrix;

// spotlights
uniform vec4 wsSpotlightPos0;
uniform vec4 wsSpotlightPos1;
uniform vec4 wsSpotlightDir0;
uniform vec4 wsSpotlightDir1;
uniform vec4 spotlightAtten0;
uniform vec4 spotlightParams0;
out vec4 spotlightTexCoord0;
out vec4 spotlightTexCoord1;

// Shadow parameters
uniform mat4 texViewProjMatrix0;
uniform mat4 texViewProjMatrix1;
uniform mat4 texViewProjMatrix2;
out vec4 lsPos0;
out vec4 lsPos1;
out vec4 lsPos2;

// output
out vec3 wsPos;
out vec3 wsNormal;
out vec3 wsVecToEye;
out vec2 wsHeightmapUV;
out vec3 vsPos;
//out vec3 vsNormal;
out mat3 normalMatrix;
//out vec3 vsVecToSun;

mat4 perspectiveProjection(float halfFOVy, float near, float far)
{
  float f = 1.0 / tan(halfFOVy);

  // The first element should be divided by aspect ratio, but we're assuming 1.0.
  return mat4(
    f, 0.0, 0.0, 0.0,
    0.0, f, 0.0, 0.0,
    0.0, 0.0, (far + near) / (near - far), -1.0,
    0.0, 0.0, (2.0 * far * near) / (near - far), 0.0
  );
}

mat4 makeInverseViewMatrix(vec4 pos, vec3 forward)
{
  // Ogre uses y-up system, but Gazebo uses z-up system
  vec3 right = normalize(cross(normalize(forward), vec3(0.0, 0.0, 1.0)));
  vec3 up = normalize(cross(right, normalize(forward)));

  mat3 rotmat = mat3(
    right, up, -forward
  );
  vec4 posXformed = vec4(-pos.xyz * rotmat, 1.0);

  // The first element should be divided by aspect ratio, but we're assuming 1.0.
  return mat4(
    vec4(right.x, up.x, -forward.x, 0.0),
    vec4(right.y, up.y, -forward.y, 0.0),
    vec4(right.z, up.z, -forward.z, 0.0),
    posXformed
  );
}

void main()
{
  vsPos = vec3(worldViewMatrix * position);

  wsPos = vec3(worldMatrix * position);
  wsNormal = mat3(inverseTransposeWorldMatrix) * normal;
  wsVecToEye = vec3(inverseViewMatrix[3]) - wsPos;
  wsHeightmapUV = uv0;

  normalMatrix = mat3(inverseTransposeWorldViewMatrix);

  //vsNormal = normalMatrix * normal;
  //vsVecToSun = normalMatrix * normalize(wsSunPosition.xyz);

  // PSSM shadows
  vec4 worldPosition = worldMatrix * position;
  lsPos0 = texViewProjMatrix0 * worldPosition;
  lsPos1 = texViewProjMatrix1 * worldPosition;
  lsPos2 = texViewProjMatrix2 * worldPosition;

  // spotlight texture coordinates
  mat4 biasMat = mat4(
    0.5, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.0,
    0.5, 0.5, 0.5, 1.0
  );
  mat4 spotlightProjMat = perspectiveProjection(acos(spotlightParams0.y), 0.01, spotlightAtten0.x);
  mat4 spotlightViewMat0 = makeInverseViewMatrix(wsSpotlightPos0, wsSpotlightDir0.xyz);
  mat4 spotlightViewMat1 = makeInverseViewMatrix(wsSpotlightPos1, wsSpotlightDir1.xyz);
  spotlightTexCoord0 = biasMat * spotlightProjMat * spotlightViewMat0 * worldPosition;
  spotlightTexCoord1 = biasMat * spotlightProjMat * spotlightViewMat1 * worldPosition;

  gl_Position = worldViewProjMatrix * position;
}
