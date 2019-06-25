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
  // Saturn's Bond albedo = 0.342 according to https://nssdc.gsfc.nasa.gov/planetary/factsheet/saturnfact.html
  // Our texture already has an average value of 0.7647 (by converting to
  // grayscale and scaling to 2x2 pixels in Gimp).
  // We'll modulate our texture by 0.342 / 0.7647 = 0.4472, so the planet
  // will shine with the correct brightness relative to incoming sunlight.

  vec3 color = texture2D(diffuseMap, uv).rgb * 0.4472;

  color *= sunIntensity * max(dot(normalize(vsNormal), vsVecToSun), 0.0);

  outputCol = vec4(color, 1.0);
}
