#include <stdio.h>
#include <stdint.h>

void square(uint8_t *num_square, uint8_t *num_add, uint32_t *result);

int main(void) {

	uint8_t num_square = 2;
	uint8_t num_add = 1;
	uint32_t result = 0;
	square(&num_square, &num_add, &result);
	printf("result: %d, num_add = %d\r\n", result, num_add);

}

void square(uint8_t *num_square, uint8_t *num_add, uint32_t *result) {
	*result = ( *num_square * *num_square ) + *num_add;
	(*num_add)++;
}

