#version 130

// variable prefixes or sub-names to indicate data requirements:
// os = object space
// ws = world space
// vs = view space
// ls = light space
// vec = vector of any length
// dir = unit vector (short for "direction")

in vec3 wsPos;
in vec3 wsNormal;
in vec3 wsVecToEye;
in vec2 wsHeightmapUV;
in vec3 vsPos;
//in vec3 vsNormal;
in mat3 normalMatrix;
//in vec3 vsVecToSun;

uniform vec4 materialDiffuse;
uniform vec4 materialSpecular;

// required by Gazebo to modify UVs for split DEMs
uniform mat4 uvTransform;

uniform vec4 wsSunPosition;
uniform vec3 sunIntensity;  // lux (visual spectrum) or watts per square meter (some other spectrum)
uniform float sunVisibility;  // fraction of sun that is visible in the range {0.0, 1.0}

// Shadow parameters
uniform vec4 pssmSplitPoints;
uniform sampler2DShadow shadowMap0;
uniform sampler2DShadow shadowMap1;
uniform sampler2DShadow shadowMap2;
uniform float inverseShadowmapSize[3];
in vec4 lsPos[3];

uniform float spotlightIntensityScale[2];
uniform vec4 vsLightPos[3];
uniform vec4 vsLightDir[3];
uniform vec4 spotlightParams[3];
uniform vec4 spotlightColor0;
uniform vec4 spotlightAtten0;
in vec4 spotlightTexCoord[2];

uniform samplerCube irradianceMap;
uniform sampler2D spotlightMap;

// output
out vec4 outputCol;

// Blend normals using Reoriented Normal Mapping.
// Using RNM method from http://blog.selfshadow.com/publications/blending-in-detail
// but modified to assume normals are already unpacked from texture. Also using
// detailStrength to modulate detail normals strength.
vec3 blendNormals(vec3 baseNormal, vec3 detailNormal, float detailStrength)
{
  vec3 newDetailNormal = mix(vec3(0.0, 0.0, 1.0), detailNormal, detailStrength);
  vec3 t = baseNormal + vec3(0.0, 0.0, 1.0);
  vec3 u = newDetailNormal * vec3(-1.0, -1.0, 1.0);
  return normalize((t / t.z) * dot(t, u) - u);
}

float calcDepthShadow(sampler2DShadow shadowMap, vec4 uv, float invShadowMapSize)
{
  // Remove shadow outside shadow maps so that all that area appears lit
  if (uv.z < 0.0 || uv.z > 1.0)
    return 1.0;

  // Debug code that shows a checkerboard pattern where shadow maps are projected
  //float checker = (mod(floor(uv.x * 40.0) + floor(uv.y * 40.0), 2.0) < 1.0) ? 0.5 : 1.0;
  //return texture(shadowMap, uv.xyz) * checker;

  float shadow = 0.0;

  // 9-sample poisson disk blur
  vec2 poissonDisk[9] = vec2[](
    vec2( 0.0, 0.0 ), 
    vec2( -0.987330103927, 0.127316674408 ), 
    vec2( -0.168435664837, -0.923511462813 ), 
    vec2( 0.637490968702, 0.633257405393 ), 
    vec2( 0.887653811523, -0.295636257708 ), 
    vec2( 0.516231382947, 0.0664456533132 ), 
    vec2( -0.408070991576, -0.332409120252 ), 
    vec2( -0.491072397165, 0.263378713033 ), 
    vec2( 0.0606228609526, 0.851023996335 )
  );
  for (int i = 0; i < poissonDisk.length(); i++)
  {
    vec4 newUV = uv;
    newUV.xy += poissonDisk[i] * invShadowMapSize;
    newUV = newUV / newUV.w;
    shadow += texture(shadowMap, newUV.xyz);
  }
  shadow /= poissonDisk.length();

  return smoothstep(0.0, 1.0, shadow);
}

float calcPSSMDepthShadow(
  sampler2DShadow shadowMap0, sampler2DShadow shadowMap1, sampler2DShadow shadowMap2,
  vec4 lsPos0, vec4 lsPos1, vec4 lsPos2,
  float invShadowmapSize0, float invShadowmapSize1, float invShadowmapSize2,
  vec4 pssmSplitPoints, float camDepth, float depthBias)
{
  float shadow = 1.0;
  vec4 bias = vec4(0.0, 0.0, depthBias, 0.0);
  // calculate shadow
  if (camDepth <= pssmSplitPoints.x)
  {
    shadow = calcDepthShadow(shadowMap0, lsPos0 + bias, invShadowmapSize0);
  }
  else if (camDepth <= pssmSplitPoints.y)
  {
    shadow = calcDepthShadow(shadowMap1, lsPos1 + bias, invShadowmapSize1);
  }
  else if (camDepth <= pssmSplitPoints.z)
  {
    shadow = calcDepthShadow(shadowMap2, lsPos2 + bias, invShadowmapSize2);
  }
  return shadow;
}

float calcPSSMDepthShadowDebug(
  sampler2DShadow shadowMap0, sampler2DShadow shadowMap1, sampler2DShadow shadowMap2,
  vec4 lsPos0, vec4 lsPos1, vec4 lsPos2,
  float invShadowmapSize0, float invShadowmapSize1, float invShadowmapSize2,
  vec4 pssmSplitPoints, float camDepth, float depthBias)
{
  float shadow = 1.0;
  vec4 bias = vec4(0.0, 0.0, depthBias, 0.0);
  // calculate shadow
  shadow = calcDepthShadow(shadowMap0, lsPos0 + bias, invShadowmapSize0);
  return shadow;
}

// vsVecToLight must not be normalized.
// vsNegLightDir and vsDirToEye must be normalized.
void spotlight(in vec3 vsVecToLight,
               in vec3 vsNegLightDir,  // Light is pointed in this direction
               in vec4 attenParams,
               in vec4 spotParams,
               in vec3 color,
               in vec4 texCoord,
               in vec3 vsDirToEye,
               in vec3 vsNormal,
               in float specularPower,
               inout vec3 diffuse,
               inout vec3 specular)
{
  // If this is not a spotlight or a spotlight isn't applied to this object
  // because it is too far away, spotParams will be set to 1,0,0,1
  if (spotParams == vec4(1, 0, 0, 1))
  {
    return;
  }

  float lightD = length(vsVecToLight);
  vec3 vsDirToLight = vsVecToLight / lightD;
  vec3 vsNegLightDirNorm = normalize(vsNegLightDir);

  // For realism, we are only using squared component in attenuation. A spotlight
  // is really an area light, but it behaves almost exactly like a point light
  // at about 3*diameter away from the light.
  float atten = 1.0 / (/*attenParams.y + attenParams.z * lightD +*/ attenParams.w * lightD * lightD);

  // Even though we are projecting textures, we use this spot cone calculation
  // to avoid artifacts to the side of the light
  float rho = dot(vsNegLightDirNorm, vsDirToLight);
  float spotT = clamp((rho - spotParams.y) / (spotParams.x - spotParams.y), 0.0, 1.0);
  // We don't need a falloff exponent to soften the spot edge because we are projecting a texture
  //spotT = pow(spotT, spotParams.z);

  vec3 texColor = textureProj(spotlightMap, texCoord).rgb;

  vec3 finalColor = max(texColor * color * (atten * spotT), vec3(0.0));
  diffuse += max(dot(vsDirToLight, vsNormal), 0.0) * finalColor;
  vec3 reflectvec = reflect(-vsDirToEye, vsNormal);
  float spotspec = pow(max(dot(vsDirToLight, reflectvec), 0.0), specularPower);
  specular += finalColor * spotspec;
}

// wsDirToSun, wsDirToEye, wsNormal, and wsDetailNormalHeight.xyz must be normalized
void lighting(vec3 wsDirToSun, vec3 wsDirToEye, vec3 wsNormal, vec4 wsDetailNormalHeight, out vec3 diffuse, out vec3 specular)
{
  const float specular_power = 100.0;

  // shadows
  // Compute shadow lookup bias using formula from
  // http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-16-shadow-mapping/
  // Should be able to bake this bias into the shadow map using constant_bias
  // and slope_scale_bias in IRGShadowParametersPlugins, but it doesn't work.
  float constantBias = vsPos.z * 0.00001 - 0.00002; // Normally a constant, but this works better with PSSM
  float cosTheta = clamp(dot(wsNormal, wsDirToSun), 0.0, 1.0);
  float slopeScaleBias = clamp(0.000004 * tan(acos(cosTheta)), 0.0, 0.001);
  float bias = constantBias - slopeScaleBias;
  float shadow = calcPSSMDepthShadow(shadowMap0, shadowMap1, shadowMap2,
                                     lsPos[0], lsPos[1], lsPos[2],
                                     inverseShadowmapSize[0], inverseShadowmapSize[1], inverseShadowmapSize[2],
                                     pssmSplitPoints, -vsPos.z, bias);

  // Only the highest parts of bumps should be lit when sun is at glancing angles
  // This removes a great deal of impossible light in shaded areas and hides shadow artifacts.
  float surfaceDot = dot(wsNormal, wsDirToSun);
  float heightMultiplier = clamp((wsDetailNormalHeight.w * 5.0 + 5.0) - (10.0 - surfaceDot * 50.0), 0.0, 1.0);

  float visibility = sunVisibility * heightMultiplier * shadow;

  // directional light diffuse
  float sundiffuse = max(dot(wsDetailNormalHeight.xyz, wsDirToSun), 0.0);
  diffuse += sunIntensity * (sundiffuse * visibility);

  // directional light specular
  vec3 reflectvec = reflect(-wsDirToEye, wsDetailNormalHeight.xyz);
  float sunspec = pow(max(dot(wsDirToSun, reflectvec), 0.0), specular_power);
  specular += sunIntensity * (sunspec * visibility);

  // irradiance diffuse (area light source simulation)
  // Gazebo is z-up but Ogre is y-up. Must rotate before cube texture lookup.
  // OpenGL cubemaps are arranged using RenderMan's left-handed coordinate system
  // resulting in the entire map being mirrored when rendered looking out from
  // the center, so we also negate y to correct our cube texture lookups.
  vec3 wsNormal_gazebo2ogre_and_mirrored = vec3(wsDetailNormalHeight.x, wsDetailNormalHeight.z, wsDetailNormalHeight.y);
  diffuse += texture(irradianceMap, wsNormal_gazebo2ogre_and_mirrored).rgb * wsDetailNormalHeight.w;

  // irradiance specular (area light source simulation)
  //vec3 reflectvec_gazebo2ogre_and_mirrored = vec3(reflectvec.x, reflectvec.z, reflectvec.y);
  // TODO: Use a specular map and use textureLod() to correlate roughness with a specific mipmap level
  //specular += texture(irradianceMap, reflectvec_gazebo2ogre_and_mirrored).rgb;

  // lander lights
  // These lander lights are computed in view space due to legacy code while
  // sunlight is computed in world space. This is awkward and a bit confusing,
  // but it seems premature to make this consistent before implementing multiple
  // materials and lighting that is more advanced than the Lambertian diffuse +
  // Phong specular model we are currently using.
  vec3 vsDirToEye = normalize(normalMatrix * wsDirToEye);
  vec3 vsDetailNormal = normalize(normalMatrix * wsDetailNormalHeight.xyz);
  for (int i=0; i<2; i++) {
    spotlight(vsLightPos[i+1].xyz - vsPos, -vsLightDir[i+1].xyz, spotlightAtten0,
              spotlightParams[1+1], spotlightColor0.rgb * spotlightIntensityScale[i],
              spotlightTexCoord[i], vsDirToEye, vsDetailNormal, specular_power,
              diffuse, specular);
  }
}

void main()
{
  vec3 wsNormalNormalized = normalize(wsNormal);
  vec3 diffuse = vec3(0);
  vec3 specular = vec3(0);
  lighting(normalize(wsSunPosition.xyz), normalize(wsVecToEye), wsNormalNormalized, vec4(wsNormalNormalized, 1.0), diffuse, specular);

  // material color
  diffuse *= materialDiffuse.rgb;
  specular *= materialSpecular.rgb;

  outputCol = vec4(diffuse + specular, 1.0);
}
