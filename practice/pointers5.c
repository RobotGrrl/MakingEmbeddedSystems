#include <stdio.h>

/*
notes

char *meep = "hi";
^ this is a pointer to a string constant
	it gets stored in the global data section of the compiled program

char meep[10]
^ this allocates an array of 10 chars on the local stack
	it will disappear when it goes out of scope

*/

int main(void) {

	char **s; // pointer to a pointer of char
	char foo[] = "Hello world"; // type char []

	// where are these things
	printf("foo at 0x%x\r\n", &foo);
	printf("foo[0] at 0x%x\r\n", &foo[0]); // should be the same, it is
	printf("*foo is %c\r\n", *foo);

	char *tmp = &foo;

	printf("tmp 0x%x\r\n", tmp); // tmp = address of foo
	printf("tmp %c\r\n", *tmp); // value at tmp = value at foo
	printf("tmp %c\r\n", *(tmp+1)); // should be e, it is

	return 0;
}

