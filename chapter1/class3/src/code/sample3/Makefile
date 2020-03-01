objects= greeting.o my_app.o
CC=gcc
my_app: $(objects)
	$(CC) -o my_app $(objects)
greeting.o:greeting.c greeting.h
my_app.o:my_app.c greeting.h
clean:
	rm my_app $(objects)

