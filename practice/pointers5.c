#include <stdio.h>

/*
notes

char *meep = "hi";
^ this is a pointer to a string literal (constant)
	it gets stored in the global data section of the compiled program

char meep[10]
^ this allocates an array of 10 chars on the local stack
	it will disappear when it goes out of scope

*/

int main(void) {

	char foo[] = "Hello world"; // type char []
	//char *foo = "Hello world"; // type char * is not the same location as char []
	char **s = &foo; // array of strings - like (*s)[0] - pointer to a pointer of char
	
	// where are these things
	printf("foo at 0x%x\r\n", &foo);
	printf("foo[0] at 0x%x\r\n", &foo[0]); // should be the same, it is
	printf("*foo is %c\r\n", *foo);

	char *tmp = &foo;

	printf("tmp 0x%x\r\n", tmp); // tmp = address of foo
	printf("tmp %c\r\n", *tmp); // value at tmp = value at foo
	printf("tmp %c\r\n", *(tmp+1)); // value at address of foo+1: should be e, it is
	printf("tmp %c\r\n", *tmp++); // should be e, it is not ... why?

	//**s = &foo; // why did this not work

	printf("s 0x%x\r\n", s); // value of s (it is an address)
	printf("s %c\r\n", *s); // value at address s
	//printf("s 0x%x\r\n", &s); // address of s itself (doesn't matter though)
	
	char *tmp2 = &(*s); // address of the value at address s

	printf("tmp2 0x%x\r\n", tmp2); // value of tmp2 = address of pointer s
	printf("tmp2 %c\r\n", *tmp2); // value at addres tmp2
	printf("tmp2 %c\r\n", *(tmp2+1)); // value at addres tmp2+1

	char c = ' ';
	int i=0;
	while(c != '\0') {
		c = *(tmp2+i);
		printf("0x%x %c\r\n", tmp2+i, c);
		i++;
	}

	return 0;
}

