#version 130

uniform vec3 sunIntensity;  // lux (visual spectrum) or watts per square meter (some other spectrum)
uniform sampler2D diffuseMap;

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

  color *= sunIntensity * max(dot(normalize(vsNormal), vsVecToSun), 0.0);

  outputCol = vec4(color, 1.0);
}
