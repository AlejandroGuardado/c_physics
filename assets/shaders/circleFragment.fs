#version 330 core

uniform vec3 color;
uniform float outline;

out vec4 FragColor;
in vec2 UV;

void main()
{
    float dist = distance(vec2(0.5, 0.5), UV);
    if(dist > 0.5) discard;
    float outlineWidth = mix(0, 0.5, outline);
    float outlineWidthHalf = outlineWidth * 0.5;

    if(!(UV.x > 0.5 && UV.y < 0.5 + outlineWidthHalf && UV.y > 0.5 - outlineWidthHalf)){
        if(dist < 0.5 - outlineWidth) discard;
    }

    FragColor = vec4(color, 1.0); 
}