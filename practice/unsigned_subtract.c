#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

uint32_t hal_get_tick;
uint32_t last_time;

int main(void) {
    
    // A
    hal_get_tick = 3000;
    last_time = 1500;
    
    if(hal_get_tick-last_time >= 1000) {
        printf("A triggered\r\n");
    } else {
        printf("A NOT triggered\r\n");
    }
    
    // B
    hal_get_tick = 300;
    last_time = 50000;
    
    if(hal_get_tick-last_time >= 1000) {
        printf("B triggered\r\n");
    } else {
        printf("B NOT triggered\r\n");
    }
    
    // C
    hal_get_tick = 300;
    last_time = 50000;
    
    if( abs(hal_get_tick-last_time) >= 1000) {
        printf("C triggered\r\n");
    } else {
        printf("C NOT triggered\r\n");
    }
    
}
