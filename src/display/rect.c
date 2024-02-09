#include "rect.h"
#include "shader.h"

#include "glad.h"

static GLint VAO;
static GLint VBO;
static unsigned int EBO;
static shader_t shaderProgram;

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

bool cwrect_init(void){
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

   if(!shader_create(&shaderProgram, "assets/shaders/rectVertex.vs", "assets/shaders/rectFragment.fs")){
      fprintf(stderr, "Error creating shader program\n");
      return false;
   }
   
   return true;
}

void cwrect_draw(vec3 position, vec2 scale, vec3 color, float outline, float rotation, mat4 view, mat4 projection){
   shader_use(shaderProgram);

   vec3 scale_3 = { scale[0], scale[1], 0.0f };
   mat4 model;
   glm_mat4_identity(model);
   glm_translate(model, position);
   glm_rotate_z(model, rotation, model);
   glm_scale(model, scale_3);

   mat4 mvp;
   glm_mat4_identity(mvp);
   glm_mat4_mul(view, model, mvp);
   glm_mat4_mul(projection, mvp, mvp);

   int mvpLoc = glGetUniformLocation(shaderProgram, "mvp");
   glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, (float*)mvp);

   int colorLoc = glGetUniformLocation(shaderProgram, "color");
   glUniform3fv(colorLoc, 1, (float*)color);

   int outlineLoc = glGetUniformLocation(shaderProgram, "outline");
   outline = glm_clamp(outline, 0.0f, 1.0f);
   glUniform1f(outlineLoc, outline);

   glBindVertexArray(VAO);
   glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
   
   glBindTexture(GL_TEXTURE_2D, 0);
   glBindVertexArray(0);
   shader_use(0);
}


void cwrect_free(void){
   shader_destroy(shaderProgram);
}