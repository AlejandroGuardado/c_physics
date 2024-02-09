#include "sprite.h"
#include "shader.h"
#include "glad.h"

static float vertices[] = {
    //Position          //UV
     0.5f,  0.5f, 0.0f, 1.0f, 1.0f, // top right
     0.5f, -0.5f, 0.0f, 1.0f, 0.0f, // bottom right
    -0.5f, -0.5f, 0.0f, 0.0f, 0.0f, // bottom left
    -0.5f,  0.5f, 0.0f, 0.0f, 1.0f  // top left 
};

static unsigned int indices[] = {
    0, 1, 3,   // first triangle
    1, 2, 3    // second triangle
};

static unsigned int VBO;
static unsigned int VAO;
static unsigned int EBO;
static shader_t shaderProgram;

bool cwsprite_init(void){
    glGenVertexArrays(1, &VAO); 
    glGenBuffers(1, &VBO);  
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3* sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    if(!shader_create(&shaderProgram, "assets/shaders/spriteVertex.vs", "assets/shaders/spriteFragment.fs")){
        fprintf(stderr, "Error creating shader program\n");
        return false;
    }

    return true;
}

void cwsprite_draw(mat4 model, texture_t textureID, mat4 view, mat4 projection){
    shader_use(shaderProgram);
    
    int modelLoc = glGetUniformLocation(shaderProgram, "model");
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, (float*)model);

    int viewLoc = glGetUniformLocation(shaderProgram, "view");
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, (float*)view);

    int projectionLoc = glGetUniformLocation(shaderProgram, "projection");
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, (float*)projection);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    shader_use(0);
}

void cwsprite_free(void){
    shader_destroy(shaderProgram);
}