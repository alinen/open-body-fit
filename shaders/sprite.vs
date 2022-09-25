#version 400

layout (location = 0) in vec3 vPosition;

uniform vec3 CameraPos;
uniform float Size;
uniform float Rot;
uniform vec3 Offset;
uniform vec4 Color;
uniform mat4 MVP;

out vec4 color;
out vec2 uv;

void main()
{
  color = Color;
  uv = vPosition.xy;
  gl_Position = MVP * vec4(vPosition, 1.0); 
}
