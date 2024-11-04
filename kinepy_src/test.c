#include "stdio.h"
#include "immintrin.h"

int main() {
    void * ptr = NULL;
    void * new_ptr = realloc(ptr, 64);
    printf("%p", new_ptr);
    free(ptr);
    free(new_ptr);
}