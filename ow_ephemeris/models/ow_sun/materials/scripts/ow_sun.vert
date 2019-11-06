#version 130

// params bound by ogre
in vec4 position;

uniform mat4 worldViewProjMatrix;

//*********************************************************
void main()
{
  gl_Position = worldViewProjMatrix * position;
}
