#include <stdio.h>

void foo1(int xval) {
	//static int x;
	int x;
	x = xval;
	printf("Address of x: 0x%x, value: %d\r\n", &x, x);
}

void foo2(int silly) {
	int y;
	printf("Address of y: 0x%x, value: %d\r\n", &y, y);
}

int main(void) {
	foo1(7);
	foo2(11);
	return 0;
}

/*

	Address of x: 0xe15ad678, value: 7
	Address of y: 0xe15ad678, value: 7

Cool, x is allocated when foo1 is called.
Then when foo2 is called, it is allocated at the
same address. The value is the same because no value
was initialized. The address is the same because after
foo1 is complete, x is no longer needed.

If x is given the static keyword, then the address for
y is different, and a different value.

*/
