#include <stdio.h>
#include <stdint.h>
#include <limits.h>

int main(void) {

	printf("int max = %lu\r\n", INT_MAX);
	printf("uint16_t max = %lu\r\n", UINT16_MAX);

	uint8_t val1 = 99;
	uint8_t val2 = 0;
	uint16_t PM2_5 = val1 + (256*val2); // 65535

	printf("PM2_5 = %d\r\n", PM2_5);

	int pm25 = PM2_5;

	printf("pm25 = %d\r\n", pm25);
	
	// > 99 = Abort trap: 6

	char buf[3];
	sprintf(buf, "%d", pm25);

	printf("buf[0]: %c\r\n", buf[0]);
	printf("buf[1]: %c\r\n", buf[1]);
	printf("buf[2]: %c\r\n", buf[2]);

	printf("buf: %s\r\n", buf);

	return 0;
}

