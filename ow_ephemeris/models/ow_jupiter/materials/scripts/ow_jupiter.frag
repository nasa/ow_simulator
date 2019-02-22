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
  // albedo
  // Jupiter's Bond albedo = 0.503 according to https://www.nature.com/articles/s41467-018-06107-2
  // Our texture already has an average value of 0.51372549 (by converting to
  // grayscale and scaling to 1 pixel in Gimp).
  // We'll modulate our texture by 0.503 / 0.51372549 = 0.9791, so the planet
  // will shine with the correct brightness relative to incoming sunlight.

  vec3 color = texture2D(diffuseMap, uv).rgb * 0.9791;

  float light = max(dot(normalize(vsNormal), vsVecToSun), 0.0);

  vec3 exposedClr = color * light * exposureMultiplier;
  vec3 gammaClr = pow(exposedClr, vec3(gammaCorrection));
  outputCol = vec4(gammaClr, 1.0);  
}
