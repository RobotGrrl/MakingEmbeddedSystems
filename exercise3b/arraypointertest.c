#include <stdint.h>
#include <stdio.h>

#define FLASHSIZE 10

void coolReadFunction(uint8_t *pBuffer);
void coolWriteFunction(uint8_t *pBuffer, uint8_t addr);

int main() {

	uint8_t flashmem[FLASHSIZE];
	flashmem[0] = 3;
	flashmem[1] = 13;
	flashmem[FLASHSIZE-1] = 33;
	
	printf("Hello world\n");

	printf("coolReadFunction\r\n");
	coolReadFunction(&flashmem[0]);

	// this also works as flashmem is already a pointer to &flashmem[0]
	//coolFunction(flashmem);

	printf("coolWriteFunction\r\n");
	coolWriteFunction(flashmem, 2);

	return 0;
}

void coolReadFunction(uint8_t *pBuffer) {

	for(uint8_t i=0; i<FLASHSIZE; i++) {
		printf("[%d]: ", i);
		printf("%d", *(pBuffer+i));
		//printf("%d", *(pBuffer+0b00001001)); // 9 in binary
		printf("\r\n");
	}

}

void coolWriteFunction(uint8_t *pBuffer, uint8_t addr) {

	*(pBuffer+addr) = 7;

	for(uint8_t i=0; i<FLASHSIZE; i++) {
		printf("[%d]: ", i);
		printf("%d", *(pBuffer+i));
		printf("\r\n");
	}

}



