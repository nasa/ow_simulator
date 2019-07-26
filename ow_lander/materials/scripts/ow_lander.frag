#version 130

// variable prefixes to indicate coordinate systems:
// os = object space
// ws = world space
// vs = view space
// ls = light space

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

// Shadow parameters
uniform vec4 pssmSplitPoints;
uniform sampler2DShadow shadowMap0;
uniform sampler2DShadow shadowMap1;
uniform sampler2DShadow shadowMap2;
uniform float inverseShadowmapSize0;
uniform float inverseShadowmapSize1;
uniform float inverseShadowmapSize2;
in vec4 lsPos0;
in vec4 lsPos1;
in vec4 lsPos2;

uniform vec4 vsSpotlightPos0;
uniform vec4 vsSpotlightPos1;
uniform vec4 vsSpotlightDir0;
uniform vec4 vsSpotlightDir1;
uniform vec4 spotlightColor0;
uniform vec4 spotlightAtten0;
uniform vec4 spotlightParams0;
in vec4 spotlightTexCoord0;
in vec4 spotlightTexCoord1;

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
  for (int i = 0; i < 9; i++)
  {
    vec4 newUV = uv;
    newUV.xy += poissonDisk[i] * invShadowMapSize;
    newUV = newUV / newUV.w;
    shadow += texture(shadowMap, newUV.xyz);
  }
  shadow /= 9.0;

  return smoothstep(0.0, 1.0, shadow);
}

float calcPSSMDepthShadow(
  sampler2DShadow shadowMap0, sampler2DShadow shadowMap1, sampler2DShadow shadowMap2,
  vec4 lsPos0, vec4 lsPos1, vec4 lsPos2,
  float invShadowmapSize0, float invShadowmapSize1, float invShadowmapSize2,
  vec4 pssmSplitPoints, float camDepth)
{
  float shadow = 1.0;
  // calculate shadow
  if (camDepth <= pssmSplitPoints.x)
  {
    shadow = calcDepthShadow(shadowMap0, lsPos0, invShadowmapSize0);
  }
  else if (camDepth <= pssmSplitPoints.y)
  {
    shadow = calcDepthShadow(shadowMap1, lsPos1, invShadowmapSize1);
  }
  else
  {
    shadow = calcDepthShadow(shadowMap2, lsPos2, invShadowmapSize2);
  }
  return shadow;
}

float calcPSSMDepthShadowDebug(
  sampler2DShadow shadowMap0, sampler2DShadow shadowMap1, sampler2DShadow shadowMap2,
  vec4 lsPos0, vec4 lsPos1, vec4 lsPos2,
  float invShadowmapSize0, float invShadowmapSize1, float invShadowmapSize2,
  vec4 pssmSplitPoints, float camDepth)
{
  float shadow = 1.0;
  // calculate shadow
  shadow = calcDepthShadow(shadowMap0, lsPos0, invShadowmapSize0);
  return shadow;
}

// vsVecToLight must not be normalized.
// vsNegLightDir must be normalized.
void spotlight(in vec3 vsVecToLight,
               in vec3 vsNegLightDir,
               in vec4 attenParams,
               in vec3 spotParams,
               in vec3 color,
               in vec4 texCoord,
               //in int index,
               in vec3 vsVecToEye,
               in vec3 vsNormal,
               in float specularPower,
               inout vec3 diffuse,
               inout vec3 specular)
{
  float lightD = length(vsVecToLight);
  vec3 vsVecToLightNorm = vsVecToLight / lightD;
  vec3 vsNegLightDirNorm = normalize(vsNegLightDir);

  // For realism, we are only using squared component in attenuation. A spotlight
  // is really an area light, but it behaves almost exactly like a point light
  // at about 3*diameter away from the light.
  float atten = 1.0 / (/*attenParams.y + attenParams.z * lightD +*/ attenParams.w * lightD * lightD);

  // Even though we are projecting textures, we use this spot cone calculation
  // to avoid artifacts to the side of the light
  float rho = dot(vsNegLightDirNorm, vsVecToLightNorm);
  float spotT = clamp((rho - spotParams.y) / (spotParams.x - spotParams.y), 0.0, 1.0);
  // We don't need a falloff exponent to soften the spot edge because we are projecting a texture
  //spotT = pow(spotT, spotParams.z);

  vec3 texColor = textureProj(spotlightMap, texCoord).rgb;

  vec3 finalColor = max(texColor * color * (atten * spotT), vec3(0.0));
  diffuse += max(dot(vsVecToLightNorm, vsNormal), 0.0) * finalColor;
  vec3 reflectvec = reflect(-vsVecToEye, vsNormal);
  float spotspec = pow(max(dot(vsVecToLightNorm, reflectvec), 0.0), specularPower);
  specular += finalColor * spotspec;
}

void lighting(vec3 wsVecToSun, vec3 wsVecToEye, vec3 wsNormal, vec4 wsDetailNormalHeight, out vec3 diffuse, out vec3 specular)
{
  const float specular_power = 100.0;

  // shadows
  float shadow = calcPSSMDepthShadow(shadowMap0, shadowMap1, shadowMap2,
                                     lsPos0, lsPos1, lsPos2,
                                     inverseShadowmapSize0, inverseShadowmapSize1, inverseShadowmapSize2,
                                     pssmSplitPoints, -vsPos.z);

  // Only the highest parts of bumps should be lit when sun is at glancing angles
  // This removes a great deal of impossible light in shaded areas and hides shadow artifacts.
  float surfaceDot = dot(wsNormal, wsVecToSun);
  float heightMultiplier = clamp((wsDetailNormalHeight.w * 5.0 + 5.0) - (10.0 - surfaceDot * 50.0), 0.0, 1.0);

  // directional light diffuse
  float sundiffuse = max(dot(wsDetailNormalHeight.xyz, wsVecToSun), 0.0);
  diffuse += sunIntensity * (sundiffuse * heightMultiplier * shadow);

  // directional light specular
  vec3 reflectvec = reflect(-wsVecToEye, wsDetailNormalHeight.xyz);
  float sunspec = pow(max(dot(wsVecToSun, reflectvec), 0.0), specular_power);
  specular += sunIntensity * (sunspec * heightMultiplier * shadow);

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
  vec3 vsVecToEye = normalMatrix * wsVecToEye;
  vec3 vsDetailNormal = normalMatrix * wsDetailNormalHeight.xyz;
  spotlight(vsSpotlightPos0.xyz - vsPos, -vsSpotlightDir0.xyz, spotlightAtten0,
            spotlightParams0.xyz, spotlightColor0.rgb, spotlightTexCoord0,
            vsVecToEye, vsDetailNormal, specular_power, diffuse, specular);
  spotlight(vsSpotlightPos1.xyz - vsPos, -vsSpotlightDir1.xyz, spotlightAtten0,
            spotlightParams0.xyz, spotlightColor0.rgb, spotlightTexCoord1,
            vsVecToEye, vsDetailNormal, specular_power, diffuse, specular);
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
