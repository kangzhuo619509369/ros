objects=main.o hello.o
hello: ${objects}
	gcc ${objects} -o hello
hello.o: function/hello.c hello.h
	gcc function/hello.c -o hello.o -I.
main.o: main.c hello.h
	gcc main.c -o main.o
clean:
	rm hello ${objects}
