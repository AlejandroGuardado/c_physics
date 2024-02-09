#include "line.h"
#include "shader.h"

#include "glad.h"

static GLint rectVAO;
static GLint rectVBO;
static shader_t rectShader;

void cwline_init(void){
   glGenVertexArrays(1, &rectVAO); 
   glGenBuffers(1, &rectVBO);
   shader_create(&rectShader, "assets/shaders/lineVertex.vs", "assets/shaders/lineFragment.fs");
}

void cwline_draw(vec3 v1, vec3 v2, vec3 color, float width, mat4 view, mat4 projection){
   float buffer[] = {
      v1[0], v1[1], v1[2],
      v2[0], v2[1], v2[2]
   };

   glBindVertexArray(rectVAO);
   glBindBuffer(GL_ARRAY_BUFFER, rectVBO);
   glBufferData(GL_ARRAY_BUFFER, sizeof(buffer), buffer, GL_DYNAMIC_DRAW);
   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)0);
   glEnableVertexAttribArray(0);

   shader_use(rectShader);
   int colorLoc = glGetUniformLocation(rectShader, "color");
   glUniform3fv(colorLoc, 1, (float*)color);

   int viewLoc = glGetUniformLocation(rectShader, "view");
   glUniformMatrix4fv(viewLoc, 1, GL_FALSE, (float*)view);

   int projectionLoc = glGetUniformLocation(rectShader, "projection");
   glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, (float*)projection);

   glLineWidth(width);
   glDrawArrays(GL_LINES, 0, 2);

   shader_use(0);
   glBindBuffer(GL_ARRAY_BUFFER, 0);
   glBindVertexArray(0);
}

void cwline_free(void){
   shader_destroy(rectShader);
}