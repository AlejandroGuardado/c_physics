#include "texture.h"
#include <stdio.h>
#include "glad.h"

bool cwtexture_load(texture_t* texture, const char* filename){
    upng_t *texture_data = upng_new_from_file(filename);
    if(texture_data == NULL) return false;
    upng_decode(texture_data);
    if(upng_get_error(texture_data) != UPNG_EOK){
        fprintf(stderr, "Texture not loaded\n");
        return false;
    }
    
    unsigned int ID;
    glGenTextures(1, &ID);
    glBindTexture(GL_TEXTURE_2D, ID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    int texture_width = upng_get_width(texture_data);
    int texture_height = upng_get_height(texture_data);
    const unsigned char *texture_buffer = upng_get_buffer(texture_data);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_width, texture_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_buffer);
    upng_free(texture_data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    *texture = ID;
    return true;
}