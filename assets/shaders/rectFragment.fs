#version 330 core

uniform vec3 color;
uniform float outline;

in vec2 UV;
out vec4 FragColor;

void main()
{
    float outlineHalf = outline * 0.5;
    if(!(UV.x > 0.5 && UV.y < 0.5 + outlineHalf && UV.y > 0.5 - outlineHalf)){
        if(UV.x > outline && UV.x < 1.0 - outline && UV.y > outline && UV.y < 1.0 - outline) discard;
    }

    FragColor = vec4(color, 1.0); 
}