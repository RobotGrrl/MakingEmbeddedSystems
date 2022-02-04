#include <stdio.h>

int main(void) {

	double d = 1.2;
	int i = 6;
	char c = 'e';

	double *pd = &d;
	int *pi = &i;
	char *pc = &c;

	int **ppi = &pi;

	printf("Address of char c is 0x%x\r\n", &c);
	printf("Address of int i is 0x%x\r\n", &i);
	printf("Address of double d is 0x%x\r\n", &d);
	
	printf("\r\n");
	
	printf("Address of char *pc is 0x%x\r\n", &pc);
	printf("Address of int *pi is 0x%x\r\n", &pi);
	printf("Address of double *pd is 0x%x\r\n", &pd);
	
	printf("\r\n");

	printf("Value of char c is %c\r\n", c);
	printf("Value of int i is %d\r\n", i);
	printf("Value of double d is %f\r\n", d);

	printf("\r\n");

	printf("Value of char *pc is %c\r\n", *pc);
	printf("Value of int *pi is %d\r\n", *pi);
	printf("Value of double *pd is %f\r\n", *pd);

	printf("\r\n");

	printf("Size of char c is %d\r\n", sizeof(c));
	printf("Size of int i is %d\r\n", sizeof(i));
	printf("Size of double d is %d\r\n", sizeof(d));

	printf("\r\n");

	printf("Size of char *pc is %d\r\n", sizeof(pc));
	printf("Size of int *pi is %d\r\n", sizeof(pi));
	printf("Size of double *pd is %d\r\n", sizeof(pd));

	printf("\r\n");

	printf("Size of value at char *pc is %d\r\n", sizeof(*pc));
	printf("Size of value at int *pi is %d\r\n", sizeof(*pi));
	printf("Size of value at double *pd is %d\r\n", sizeof(*pd));

	printf("\r\n");

	printf("What's ppi 0x%x\r\n", ppi);
	printf("What's *ppi 0x%x\r\n", *ppi);
	printf("What's **ppi %d\r\n", **ppi);

	printf("\r\n");

	return 0;
}