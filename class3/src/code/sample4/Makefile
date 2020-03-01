CFLAGS=-c -g -Wall -I headfile
hello:main.o annai.o
	gcc main.o annai.o -o hello
main.o:main.c
	gcc ${CFLAGS} main.c -o main.o
annai.o:annai.c
	gcc ${CFLAGS} annai.c -o annai.o
.PHONY: clean
clean:
	rm *.o hello 

