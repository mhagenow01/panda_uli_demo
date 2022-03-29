#include <stdio.h>

//  How I compiled the library: g++ -shared -o testctypestruct.so -fopenmp -fPIC testctypestruct.cpp

struct Link{
    char* name;
    float R[3][3];
};

struct Model{
    char *name;
    Link *links;
    // Need Joints
};

extern "C" {
    struct Model* passArguments(struct Model* model)
    {
        printf("I got this from python: %s\n", model->name);
        printf("I also got this from python: %s\n", model->links[0].name);
        printf("I also got this from python: %f\n", model->links[1].R[0][2]);

        // Change something about the model to see if we can receive the change in python
        char myword[] = {'n','e','w','m','o','d','e','l','\0'}; 
        model->name = myword;

        return model;
    }
}
