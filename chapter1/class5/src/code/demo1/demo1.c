#include <stdio.h>
#include <pthread.h>

void * show(void * u)
{
	printf("New Thread\n");
}

int main()
{
	pthread_t tid;
	pthread_create(&tid, NULL, &show, NULL);
	printf("main thread\n");
	pthread_join(tid, NULL);
	return 0;
}
