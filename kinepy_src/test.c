#include "stdio.h"
#include "immintrin.h"

int main() {
    float const _op1[] = {1.0f, 1.0f, 1.0f, 1.0f, 5.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 5.0f, 1.0f, 1.0f, 1.0f};
    float * v = malloc(sizeof(float) * 9);
    __m256 op1;
    op1 = _mm256_load_ps(v+1);
    __m256 op2;
    float const _op2[] = {1.0f, 8.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 8.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f};
    op2 = _mm256_load_ps(_op2+1);
    printf("%p ::: %p\n", v+1, _op2);
    float output[8];
    __m256 result = _mm256_div_ps(op1, op2);
    _mm256_store_ps(output, result);

    for (int index = 0; index < 8; ++index) {
        printf("%f\n", output[index]);
    }
    free(v);
}