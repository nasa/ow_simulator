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
in mat3 normalMatrix;

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
// We only need this measurement for one shadow map because Ogre uses the same
// dimensions for all shadow maps.
float inverseShadowMapSize = 1.0 / float(textureSize(shadowMap0, 0).x);
in vec4 lsPos[3];

uniform vec4 wsLightPos[3];
uniform vec4 wsLightDir[3];
uniform vec4 lightDiffuseColor[3];
uniform vec4 lightAtten[3];
uniform vec4 spotlightParams[3];
in vec4 spotlightTexCoord[2];

uniform samplerCube irradianceMap;
uniform sampler2D spotlightMap;

// output
out vec4 outputCol;

const float PI = 3.14159265358979;

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

float calcDepthShadow(sampler2DShadow shadowMap, vec4 uv, float invMapSize)
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
    newUV.xy += poissonDisk[i] * invMapSize;
    newUV = newUV / newUV.w;
    shadow += texture(shadowMap, newUV.xyz);
  }
  shadow /= poissonDisk.length();

  return smoothstep(0.0, 1.0, shadow);
}

float calcPSSMDepthShadow(
  sampler2DShadow shadowMap0, sampler2DShadow shadowMap1, sampler2DShadow shadowMap2,
  vec4 lsPos0, vec4 lsPos1, vec4 lsPos2,
  vec4 pssmSplitPoints, float camDepth, float depthBias)
{
  float shadow = 1.0;
  vec4 bias = vec4(0.0, 0.0, depthBias, 0.0);
  // calculate shadow
  if (camDepth <= pssmSplitPoints.x)
  {
    shadow = calcDepthShadow(shadowMap0, lsPos0 + bias, inverseShadowMapSize);
  }
  else if (camDepth <= pssmSplitPoints.y)
  {
    shadow = calcDepthShadow(shadowMap1, lsPos1 + bias, inverseShadowMapSize);
  }
  else if (camDepth <= pssmSplitPoints.z)
  {
    shadow = calcDepthShadow(shadowMap2, lsPos2 + bias, inverseShadowMapSize);
  }
  return shadow;
}

float calcPSSMDepthShadowDebug(
  sampler2DShadow shadowMap0, sampler2DShadow shadowMap1, sampler2DShadow shadowMap2,
  vec4 lsPos0, vec4 lsPos1, vec4 lsPos2,
  vec4 pssmSplitPoints, float camDepth, float depthBias)
{
  float shadow = 1.0;
  vec4 bias = vec4(0.0, 0.0, depthBias, 0.0);
  // calculate shadow
  shadow = calcDepthShadow(shadowMap0, lsPos0 + bias, inverseShadowMapSize);
  return shadow;
}

// wsVecToLight must not be normalized.
// wsLightDir must be normalized.
vec3 spotlight(in vec3 wsVecToLight,
               in vec3 wsLightDir,
               in vec4 attenParams,
               in vec4 spotParams,
               in vec3 color,
               in vec4 texCoord,
               in int index)
{
  // If this is not a spotlight or a spotlight isn't applied to this object
  // because it is too far away, spotParams will be set to 1,0,0,1
  if (spotParams == vec4(1, 0, 0, 1))
  {
    return vec3(0, 0, 0);
  }
  else
  {
    float lightD = length(wsVecToLight);
    vec3 wsDirToLight = wsVecToLight / lightD;

    // For realism, we are only using squared component in attenuation. A spotlight
    // is really an area light, but it behaves almost exactly like a point light
    // at about 3*diameter away from the light.
    float atten = 1.0 / (/*attenParams.y + attenParams.z * lightD +*/ attenParams.w * lightD * lightD);

    // In a typical spotlight implementation, spotT would ordinarily zero-out
    // all light outside a circle based on the spotlight cone angles:
    // float rho = dot(-wsLightDir, wsDirToLight);
    // float spotT = clamp((rho - spotParams.y) / (spotParams.x - spotParams.y), 0.0, 1.0);
    // spotT = pow(spotT, spotParams.z);
    // But we are projecting a square texture and instead cut off light outside
    // valid texcoords {0.0, 1.0}. This avoids cutting off the texture corners.
    // The math is a bit complicated, but it results in a spotT value of 1.0
    // where the texture is projected and 0.0 outside the texture. A simpler
    // implementation would use conditionals but it would likely be slower.
    vec2 normalizedTexCoord = texCoord.xy / texCoord.w;
    // Arbitrarily large multiplier for saturating values at 0 or 1
    const float saturator = 1000000.0;
    // 0 where texcoord < 0 and 1 where texcoord > 0
    vec2 spotT0 = clamp(normalizedTexCoord * saturator, 0.0, 1.0);
    // 0 where texcoord > 1 and 1 where texcoord < 1
    vec2 spotT1 = clamp((vec2(1.0) - normalizedTexCoord) * saturator, 0.0, 1.0);
    // 0 behind light and 1 in front of light
    float spotTZ = clamp(dot(-wsLightDir, wsDirToLight) * saturator, 0.0, 1.0);
    float spotT = spotT0.x * spotT0.y * spotT1.x * spotT1.y * spotTZ;

    vec3 texColor = textureProj(spotlightMap, texCoord).rgb;

    // Attenuation and spot cone get baked into final light color. This is how
    // spotlights get generalized so they can be stored in lights array.
    return max(texColor * color * (atten * spotT), vec3(0.0));
  }
}

// Incoming vectors L, V, and N, must be normalized
vec3 brdf_oren_nayar(vec3 L, vec3 V, vec3 N, float roughness, vec3 albedo) {
  // Adapted from https://mimosa-pudica.net/improved-oren-nayar.html
  float r2 = roughness * roughness;
  float LdotV = dot(L, V);
  float NdotL = dot(N, L);
  float NdotV = dot(N, V);
  float s = LdotV - NdotL * NdotV;
  float t = mix(1.0, max(NdotL, NdotV), step(0.0, s));
  vec3  A = vec3(1.0) - vec3(0.5 * r2 / (r2 + 0.33)) + albedo * vec3(0.17 * r2 / (r2 + 0.13));
  float B = (0.45 * r2 / (r2 + 0.09));
  return albedo * max(0.0, NdotL) * (A + vec3(B * s / t)) / PI;
}

// wsDirToSun, wsDirToEye, wsNormal, and wsDetailNormalHeight.xyz must be normalized
void lighting(vec3 wsDirToSun, vec3 wsDirToEye, vec3 wsNormal, vec4 wsDetailNormalHeight,
              vec3 albedo, out vec3 diffuse, out vec3 specular)
{
  const int NUM_LIGHTS = 3;
  struct LightData {
    vec3 wsVecToLight;
    vec3 color;
  };
  LightData lights[NUM_LIGHTS];

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
                                     pssmSplitPoints, -vsPos.z, bias);

  // Only the highest parts of bumps should be lit when sun is at glancing angles
  // This removes a great deal of impossible light in shaded areas and hides shadow artifacts.
  float surfaceDot = dot(wsNormal, wsDirToSun);
  float heightMultiplier = clamp((wsDetailNormalHeight.w * 5.0 + 5.0) - (10.0 - surfaceDot * 50.0), 0.0, 1.0);

  // sunlight
  lights[0].wsVecToLight = wsDirToSun;
  lights[0].color = sunIntensity * (sunVisibility * heightMultiplier * shadow);

  // lander lights
  for (int i=0; i<2; i++) {
    vec3 wsVecToLight = wsLightPos[i+1].xyz - wsPos;
    lights[i+1].wsVecToLight = wsVecToLight;
    lights[i+1].color = spotlight(wsVecToLight, wsLightDir[i+1].xyz, lightAtten[i+1],
                                  spotlightParams[1+1], lightDiffuseColor[i+1].rgb,
                                  spotlightTexCoord[i], i+1);
  }

  const float roughness = 0.3;
  const float specPower = 60.0;
  for (int i = 0; i < NUM_LIGHTS; i++)
  {
    vec3 wsDirToLight = normalize(lights[i].wsVecToLight);
    float nDotL = dot(wsNormal, wsDirToLight);
    if (nDotL > 0.0)
    {
      diffuse += lights[i].color * brdf_oren_nayar(wsDirToLight, normalize(wsDirToEye),
                                                   wsDetailNormalHeight.xyz, roughness, albedo);

      vec3 reflectVec = reflect(-wsDirToLight, wsDetailNormalHeight.xyz);
      float specShape = pow( max(dot(reflectVec, normalize(wsDirToEye)), 0.0), specPower);
      specular += lights[i].color * clamp(specShape, 0.0, 1.0);
    }
  }

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
}

void main()
{
  vec3 wsNormalNormalized = normalize(wsNormal);
  vec3 diffuse = vec3(0);
  vec3 specular = vec3(0);
  lighting(normalize(wsSunPosition.xyz), normalize(wsVecToEye), wsNormalNormalized, vec4(wsNormalNormalized, 1.0), materialDiffuse.rgb, diffuse, specular);
  specular *= materialSpecular.rgb;

  outputCol = vec4(diffuse + specular, 1.0);
}
