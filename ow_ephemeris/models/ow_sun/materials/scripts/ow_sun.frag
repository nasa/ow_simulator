#version 130

out vec4 outputCol;

void main()
{
  // Sun will look white at any reasonable simulated camera exposure, even as close as Mercury.
  outputCol = vec4(1000000000.0, 1000000000.0, 1000000000.0, 1.0);
}
