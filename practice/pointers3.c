#include <stdio.h>

int main(void) {

	//char c = 'Z';
	char c[] = "ZA";
	char *cp = &c;

	printf("cp is 0x%x\r\n", cp);
	printf("char at cp is %c\r\n", *cp);

	//cp = cp+1;
	cp++; // increment the ADDRESS (because it was declared &c)
	printf("cp is 0x%x\r\n", cp);
	printf("char at cp is %c\r\n", *cp);

	//cp = cp+1;
	cp++; // increment the ADDRESS (because it was declared &c)
	printf("cp is 0x%x\r\n", cp);
	printf("char at cp is %c\r\n", *cp);

	cp--; // go back

	(*cp)++; // increment the VALUE (because *cp aka value at cp)
	printf("cp is 0x%x\r\n", cp);
	printf("char at cp is %c\r\n", *cp);

	return 0;
}