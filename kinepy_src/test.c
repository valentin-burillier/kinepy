#include "stdio.h"

int main() {
    volatile int a = 0;
    printf("%d\n", a++ + a);
    a = 0;
    printf("%d\n", a + a++);
}