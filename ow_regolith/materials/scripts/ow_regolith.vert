#version 130

// variable prefixes or sub-names to indicate data requirements:
// os = object space
// ws = world space
// vs = view space
// ls = light space
// vec = vector of any length
// dir = unit vector (short for "direction")

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
uniform vec3 landerUpVec;  // Should be set after launch and updated if lander changes orientation
uniform vec4 wsLightPos[3];
uniform vec4 wsLightDir[3];
uniform vec4 spotlightParams[3];
uniform vec4 spotlightAtten0;
out vec4 spotlightTexCoord[2];

// Shadow parameters
uniform mat4 texViewProjMatrix[3];
out vec4 lsPos[3];

// output
out vec3 wsPos;
out vec3 wsNormal;
out vec3 wsVecToEye;
out vec2 wsHeightmapUV;
out vec3 vsPos;
//out vec3 vsNormal;
out mat3 normalMatrix;
//out vec3 vsVecToSun;

// halfFOVy must be in radians. near and far must be positive.
mat4 perspectiveProjection(float halfFOVy, float near, float far)
{
  float f = 1.0 / tan(halfFOVy);

  // The first element should be divided by aspect ratio, but we're assuming 1.0.
  return mat4(
    f, 0.0, 0.0, 0.0,
    0.0, f, 0.0, 0.0,
    0.0, 0.0, -(far + near) / (far - near), -1.0,
    0.0, 0.0, (-2.0 * far * near) / (far - near), 0.0
  );
}

mat4 makeInverseViewMatrix(vec4 pos, vec3 forward)
{
  // Ogre uses y-up system, but Gazebo uses z-up system
  vec3 right = normalize(cross(normalize(forward), landerUpVec));
  vec3 up = normalize(cross(right, normalize(forward)));

  mat3 rotmat = mat3(
    right, up, -forward
  );
  vec4 posXformed = vec4(-pos.xyz * rotmat, 1.0);

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
  vec4 worldPos = worldMatrix * position;
  for (int i=0; i<lsPos.length(); i++)
    lsPos[i] = texViewProjMatrix[i] * worldPos;

  // spotlight texture coordinates
  // Ogre's texture_viewproj_matrix and spotlight_viewproj_matrix both use a
  // view matrix made from spotlight direction and an arbitrary up-vector, so
  // texcoords computed from these spin in strange ways as spotlights rotate.
  // For this reason, we construct our own transformations from worldPos to
  // texcoords to project spotlight textures using the lander's up vector. This
  // method assumes that spotlights have been yawed and pitched but not rolled.
  mat4 biasMat = mat4(  // Convert tex coords from {-1, 1} to {0, 1}.
    0.5, 0.0, 0.0, 0.0,
    0.0, -0.5, 0.0, 0.0,  // It is not known why element 1 needs to be negative to correctly project a texture.
    0.0, 0.0, 1.0, 0.0,
    0.5, 0.5, 0.0, 1.0
  );
  for (int i=0; i<spotlightTexCoord.length(); i++) {
    mat4 spotlightProjMat = perspectiveProjection(acos(spotlightParams[i+1].y), 0.01, spotlightAtten0.x);
    mat4 spotlightViewMat = makeInverseViewMatrix(wsLightPos[i+1], wsLightDir[i+1].xyz);
    spotlightTexCoord[i] = biasMat * spotlightProjMat * spotlightViewMat * worldPos;
  }

  gl_Position = worldViewProjMatrix * position;
}
