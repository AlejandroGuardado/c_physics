#version 330 core

in vec2 UV;
out vec4 FragColor;

uniform sampler2D ourTexture;

void main()
{
    FragColor = texture(ourTexture, UV); 
}