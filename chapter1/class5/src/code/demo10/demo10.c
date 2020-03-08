#include <error.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int main()
{
	char string[] = "an example string";
	char* offset;

	offset = strchr(string,' ' );
	if(offset)
		strcpy(string, offset+1);
	printf("changed string is %s\n", string);
	return 0;
}
