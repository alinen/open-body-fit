#version 400

uniform float uLargeBlockSize; 
uniform float uSmallBlockSize; 

in vec4 position;
in vec3 normal;
in vec2 uv;

out vec4 FragColor;

vec3 gridColor(in vec3 pos)
{
   float largeBlock = uLargeBlockSize; // 200
   float smallBlock = uSmallBlockSize; // 50

   float lineWidth = smallBlock * 0.1; //5.0;
   float largePercent = lineWidth / largeBlock;
   float smallPercent = (lineWidth * 0.25) / smallBlock;

   vec3 lineColor = vec3(0.0);

   float adx = abs(pos.x);
   float adz = abs(pos.z);
   if (adx < lineWidth && pos.z > 0.0) lineColor = vec3(0.0, 0.0, 1.0);
   if (adz < lineWidth && pos.x > 0.0) lineColor = vec3(1.0, 0.0, 0.0);

   vec3 modPos = fract(pos.xyz/largeBlock);
   float dx = min(modPos.x, 1.0 - modPos.x);
   float dz = min(modPos.z, 1.0 - modPos.z);
   float d = min(dx, dz);

   vec3 color1 = mix(lineColor, vec3(0.8), smoothstep(0.0, largePercent, d));

   modPos = fract(pos.xyz/smallBlock);
   dx = min(modPos.x, 1.0 - modPos.x);
   dz = min(modPos.z, 1.0 - modPos.z);
   d = min(dx, dz);

   vec3 color2 = mix(lineColor, vec3(0.8), smoothstep(0.0, smallPercent, d));

   vec3 color = min(color1, color2);
   return color;
}

void main()
{
   // hack filtering based on blending (slow and looks so so)
   // TODO: estimate percentage of foreground color across pixel
   vec3 color = vec3(0.0);
   float size = 4.0f;
   for (float x = 0.0; x < 1.1; x += 1.0/size)
   {
      vec3 offsetx = x * dFdx(position.xyz);
      for (float y = 0.0; y < 1.1; y += 1.0/size)
      {
         vec3 offsety = y * dFdy(position.xyz);
         color += gridColor(position.xyz + offsetx + offsety);
      }
   }
   color /= (size+1)*(size+1);
   //vec3 color = gridColor(position.xyz);

   // feather the edges of the grid to create a smooth horizon
   vec3 backgroundColor = vec3(0.8);
   float gridSize = 1000.0; // should match scale of floor plane
   float edge = 0.05 * gridSize;
   float d = length(position.xz);
   float factor = clamp((d - edge)/(gridSize - edge), 0.0, 1.0);
   color = mix(color, backgroundColor, factor);

   FragColor = vec4(color, 1.0);
}

