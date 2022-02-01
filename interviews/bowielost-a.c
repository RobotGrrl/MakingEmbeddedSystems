#include <stdint.h>
#include <stdio.h>

#define CLEAR 0
#define HOME 1
#define PUDDLE 2
#define START 3

typedef struct {
	uint8_t north;
	uint8_t east;
	uint8_t south;
	uint8_t west;
	uint8_t obstacle;
} Compass;

Compass themap[3][5] = { { {0,1,1,0,START}, {0,1,1,1,CLEAR}, {0,1,0,1,CLEAR}, {0,1,0,1,CLEAR}, {0,0,1,1,CLEAR} }, 
					   				     { {0,1,1,0,CLEAR}, {1,0,1,1,CLEAR}, {0,0,0,0,PUDDLE}, {0,0,0,0,PUDDLE}, {1,0,1,0,CLEAR} },
											   { {1,1,0,0,CLEAR}, {1,1,0,1,CLEAR}, {0,1,0,1,CLEAR}, {0,1,0,1,HOME}, {1,0,0,1,CLEAR} }
											 };

uint8_t xpos = 0;
uint8_t ypos = 0;
uint8_t odometer = 0;

int main() {

	while(1) {

		printf("(%d,%d)\r\n", xpos, ypos);

		Compass here = themap[ypos][xpos];

		if(here.obstacle == HOME) {
			printf("bowie is home! (%d, %d)\r\n", xpos, ypos);
			return 0;
		}

		// check for no obstacles
		if(here.obstacle == CLEAR || here.obstacle == START) {

			// establish its order of operations
			// there's no tracking if it gets in a loop

			if(here.south == 1) {
				ypos++;
			} else if(here.east == 1) {
				xpos++;
			} else if(here.west == 1) {
				xpos--;
			} else if(here.north == 1) {
				ypos--;
			} else { // bowie is very much stuck now
				printf("call for help! (%d, %d)\r\n", xpos, ypos);
				return 0;
			}

		} else if(here.obstacle == PUDDLE) {
			// bowie is very much stuck now
			printf("call for help! (%d, %d)\r\n", xpos, ypos);
			return 0;
		}

	}

	return 0;	
}



