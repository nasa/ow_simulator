// Geometry Shader
#version 130
#extension GL_ARB_geometry_shader4 : enable

// input
in vec3 wsPos;
in vec3 wsNormal;
in vec3 wsVecToEye;
in vec2 wsHeightmapUV;
in vec3 vsPos;
//in vec3 vsNormal;
in mat3 normalMatrix;
//in vec3 vsVecToSun;
in vec4 lsPos[3];
in vec4 spotlightTexCoord[2];

// // output
// out vec3 wsPos;
// out vec3 wsNormal;
// out vec3 wsVecToEye;
// out vec2 wsHeightmapUV;
// out vec3 vsPos;
// //out vec3 vsNormal;
// out mat3 normalMatrix;
// //out vec3 vsVecToSun;
// out vec4 spotlightTexCoord[2];
// out vec4 lsPos[3];

uniform float offset;
// --------------------
void main()
{
  // increment variable
  int i;
  vec4 vertex;
  // --------------------
  // This example has two parts
  //  step a) draw the primitive pushed down the pipeline
  //     there are gl_VerticesIn # of vertices
  //     put the vertex value into gl_Position
  //     use EmitVertex => 'create' a new vertex
  //    use EndPrimitive to signal that you are done creating a primitive!
  //  step b) create a new piece of geometry
  //    I just do the same loop, but I negate the vertex.z
  //  result => the primitive is now mirrored.
  // Pass-thru!
  for(i = 0; i < gl_VerticesIn; i++)
  {
    gl_Position = gl_PositionIn[i];
    EmitVertex();
  }
  EndPrimitive();

  // New piece of geometry!
  // for(i = 0; i < gl_VerticesIn; i++)
  // {
  //   vertex = gl_PositionIn[i];
  //   vertex.x += offset;
  //   gl_Position = vertex;

  //   EmitVertex();
  // }
  // EndPrimitive();
}

