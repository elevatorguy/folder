#include "utilities.h"

void allocateArray(double *&data, int num_elements) {
        #ifdef __HOSTED__
        data = calloc(sizeof(double), num_elements);
        #else
        EFI_STATUS status = bs->allocatePool(EfiLoaderData, num_elements*sizeof(double), &data);
        #endif
}
void allocateArray(int *&data, int num_elements) {
        #ifdef __HOSTED__
        data = calloc(sizeof(int), num_elements);
        #else
        EFI_STATUS status = bs->allocatePool(EfiLoaderData, num_elements*sizeof(int), &data);
        #endif
}

void freeArray(double *&data) {
    #ifdef __cplusplus
    delete[] data;
    data = NULL;
    #else
        #ifdef __HOSTED__
                free(data);
        #else
                bs->freePool(data);
        #endif
    #endif
}

void freeArray(int *&data) {
    #ifdef __cplusplus
    delete[] data;
    data = NULL;
    #else
        #ifdef __HOSTED__
                free(data);
        #else
                bs->freePool(data);
        #endif
    #endif
}
