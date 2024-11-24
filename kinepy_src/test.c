#include "stdio.h"
#include "math.h"

#include "xmmintrin.h"

#define c(a, b) a ## b ## _t
#define e(a, b) c(a, b)

int main() {
    volatile float a = sqrtf(-1.0f);
    printf("%f", a);
}