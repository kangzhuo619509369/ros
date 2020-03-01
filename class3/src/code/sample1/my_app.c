#include <stdio.h>
#include "greeting.h"
#define N 10

int main(void)
{
    char name[N];
    printf("Your name,Please:");
    scanf("%s",name);
    greeting(name);
    return 0;
}

