#include <stdio.h>
#include <stdlib.h>

int main()
{
	char * s = (char*)malloc(20);
	free(s);
	free(s);
	return 0;
}
