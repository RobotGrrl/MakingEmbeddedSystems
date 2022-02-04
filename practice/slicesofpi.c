#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// max size of unsigned long long
// 18,446,744,073,709,551,615
// https://en.wikibooks.org/wiki/C_Programming/limits.h

unsigned long long pi_digits = 14159265358979323846;

int main() {

	uint8_t num_digits = 0;
	bool counting_digits = true;
	unsigned long long temp_pi = pi_digits;
	while(counting_digits) {
		temp_pi/=10;
		num_digits++;
		if(temp_pi==0) {
			counting_digits = false;
		}
	}
	//printf("num digits: %d\r\n", num_digits);

	char slices[num_digits];
	sprintf(slices,"%llu",pi_digits);

	uint8_t slice_count = 0;
	for(int i=0; i<num_digits; i++) {
		if((i+1)%3==0) {
			slice_count++;
			uint16_t slice_digits = ( (slices[i-2]-'0')*100) + ( (slices[i-1]-'0')*10) + (slices[i]-'0');
			printf("slice %d: %d\r\n", slice_count, slice_digits);
		}
	}

	return 0;
}





