#include <stdio.h>
#include <stdint.h>

/*

limitations:
- max size of sum data type
- amount of time in the loop
	- this could be halved, by adding numbers iterating both directions

n = 2^16
sum = n*(n+1)/2
		= (n^2+n)/2
		= (2^(16*2)+2^16)/2 // so sub in n
		= (2^32+2^16)/2

n = 2^3
sum = n*(n+1)/2
		= (n^2+n)/2
		= (2^(3*2)+2^3)/2 // so sub in n
		= (2^6+2^3)/2
		= 36

sum is 4.5x greater?

- could store in a different datatype
- could factor the sum
- could approximate the sum
- could write the number to a text file (sd card logging)
- could chunk the sum into multiple smaller variables
- if it gets even larger for the chunks, could have pages of chunks
- even larger, could have sectors of pages

*/

int main() {
	
	uint8_t n = 6;
	uint16_t sum = 0; // sum will always be bigger than n
	for(int i=1; i<=n; i++) {
		sum += i;
	}
	printf("sum: %d\r\n", sum);

	sum = 0;
	uint8_t count = 0;
	for(count=1; count<=n/2; count++) {
		uint8_t i = count;
		uint8_t j = n-count;
		sum += i+j;
	}
	// add count because it is the half way point of the loop, it wasn't added in
	// subtract 1 from count because it added one more at the end of the for loop
	sum += count-1;
	printf("sum: %d\r\n", sum);

	return 0;
}










