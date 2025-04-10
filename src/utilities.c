#include "utilities.h"

void freeArray(double *&data) {
    delete[] data;
    data = nullptr;
}

void freeArray(int *&data) {
    delete[] data;
    data = nullptr;
}
