#version 400

layout (location = 0) in vec3 vPos;
layout (location = 1) in vec3 vNor;
layout (location = 2) in vec2 vUV;

uniform mat4 ModelViewMatrix;
uniform mat4 ModelMatrix;
uniform mat3 NormalMatrix;
uniform mat4 MVP;

out vec4 position;
out vec3 normal;
out vec2 uv;

void main()
{
   position = ModelMatrix * vec4( vPos, 1.0);
   normal = normalize( NormalMatrix * vNor);
   uv = vUV;

   gl_Position = MVP * vec4(vPos, 1.0);
}


