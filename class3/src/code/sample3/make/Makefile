objects= greeting.o my_app.o
CC=gcc
my_app: $(objects)
	$(CC) -o my_app $(objects)
greeting.o:greeting.c greeting.h
	$(CC) -c greeting.c
my_app.o:my_app.c greeting.h
	$(CC) -c my_app.c

