#include <stdio.h>

void swap_nums(int *x, int *y) {

	int tmp;
	tmp = *x; // "tmp is the value at x"
	*x = *y; // "pointer x is the value at y"
	*y = tmp; // "pointer y is tmp"

}

// char *s1, *s2; // char pointers
// swap_pointers(&s1, &s2); // sending address of char pointers
//void swap_pointers(char *x, char *y) {
// this is a pointer to a pointer to a char
void swap_pointers(char **x, char **y) {

	// to swap it we want to get
	// the address of the pointer
	// and assign that to a char pointer

	printf("hi\r\n");

	printf("*x: 0x%x\r\n", *x); // value at x, which is a pointer
	printf("*y: 0x%x\r\n", *y); // value at y, which is a pointer

	char *tmp1 = *x; // char pointer tmp1 is assigned 
						  		 // the value at x (which is a pointer)

	char *tmp2 = *y; // char pointer tmp2 is assigned
									 // the value at y (which is a pointer)

	printf("tmp1: 0x%x\r\n", tmp1);	// printing tmp1 (a char pointer),
																	// which is the value at x (a pointer)
	printf("tmp2: 0x%x\r\n", tmp2);

	*x = tmp2; // pointer x is assigned to tmp2
	*y = tmp1; // pointer y is assigned to tmp1

	printf("x: 0x%x\r\n", *x); // printing the value at x, which is a pointer
	printf("y: 0x%x\r\n", *y);

}

int main(void) {

	int a,b;
	
	a=3; b=4;
	printf("a is %d\r\n", a);
	printf("b is %d\r\n", b);

	swap_nums(&a, &b);

	printf("a is %d\r\n", a);
	printf("b is %d\r\n", b);

	char *s1, *s2;

	s1 = "second";
	s2 = "first";

	printf("s1 is %s\r\n", s1);
	printf("s2 is %s\r\n", s2);

	printf("&s1 is 0x%x\r\n", &s1);
	printf("&s2 is 0x%x\r\n", &s2);

	swap_pointers(&s1, &s2);

	printf("\r\n");

	printf("s1 is %s\r\n", s1);
	printf("s2 is %s\r\n", s2);

	printf("&s1 is 0x%x\r\n", &s1);
	printf("&s2 is 0x%x\r\n", &s2);

	return 0;
}





