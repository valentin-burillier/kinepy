#include "stdio.h"
#include "math.h"


int main() {
    volatile float a = sqrtf(-1.0f);
    printf("%f", a);
}