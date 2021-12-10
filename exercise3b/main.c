/*
 * Making Embedded Systems - Red Jellys
 * Exercise 3b:
 *   Make a flash simulator for W25Q80DV/DL
 *
 * Erin K - Dec 9, 2021
 *
 * Functionality that's missing:
 * - Check Read Status Register for BUSY bit before erase
 * - Does Write Status Register need to be set before writing?
 */

#include <stdint.h>
#include <stdio.h>

#include "spi_flash.h"

#define FLASHSIZE 10
#define SECTOR_SIZE 3
#define PAGESIZE 2

uint8_t flashmem[FLASHSIZE];

uint32_t cycle_counter = 0;

void initFlash(void);
void printFlashMem(void);
void eraseSector(uint32_t SectorAddr);
void eraseBulk(void);
void readBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void writePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void writeBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);


int main() {

	printf("Welcome %d\r\n", sFLASH_M25P128_ID);

	initFlash();

	// testing halfword to make sure it's good
	uint16_t bigword = 0b1000001101001111;
	printf("testing halfword with 0x%X\r\n", bigword);
	sFLASH_SendHalfWord(bigword); // b10000011 0x83  ,  b01001111 0x4f

	printFlashMem();
	eraseSector(2);
	printFlashMem();

	printFlashMem();
	eraseBulk();
	printFlashMem();

	readBuffer(flashmem, 2, 6);
	printFlashMem();

	uint8_t newpage[PAGESIZE];
	for(uint8_t i=0; i<PAGESIZE; i++) {
		newpage[i] = 9;
	}
	writePage(newpage, 3, PAGESIZE);
	printFlashMem();

	uint8_t newflash[FLASHSIZE];
	for(uint8_t i=0; i<FLASHSIZE; i++) {
		if(i%2 == 0) {
			newflash[i] = 4;
		} else {
			newflash[i] = 2;
		}
	}
	writeBuffer(newflash, 0, FLASHSIZE);
	printFlashMem();


	printf("de-init\r\n");
	sFLASH_DeInit();
	printf("de-init done\r\n");

	printf("OK\r\n");

	return 0;
}


void initFlash(void) {
	printf("init enter\r\n");
	
	sFLASH_Init();
	
	for(uint32_t i=0; i<FLASHSIZE; i++) {
		flashmem[i] = 1;
		// since this is simply initing the array for our internal usage,
		// the cycle_counter is not incremented here
	}
	printFlashMem();

	printf("init exit\r\n");
}


void printFlashMem(void) {
	printf("flashmem (%d): ", FLASHSIZE);
	for(uint32_t i=0; i<FLASHSIZE; i++) {
		printf("%d", flashmem[i]);
		if(i!=FLASHSIZE-1) printf(", ");
	}
	printf("\r\n");
}


void eraseSector(uint32_t SectorAddr) {

	printf("eraseSector enter\r\n");

	uint32_t start_addr = SectorAddr;
	uint32_t end_addr = SectorAddr+SECTOR_SIZE;

	// depending on the level of fault tolerance, this could be
	// changed to return out of this if the end address does not fit
	if(end_addr > FLASHSIZE) {
		printf("!! warning: eraseSector: end_addr > FLASHSIZE\r\n");
		end_addr = FLASHSIZE;
	}

	sFLASH_EraseSector(SectorAddr);

	// update the array
	for(uint32_t i=start_addr; i<end_addr; i++) {
		flashmem[i] = 0;
		cycle_counter++;
	}

	printf("eraseSector exit\r\n");

}


void eraseBulk(void) {

	printf("eraseBulk enter\r\n");

	uint32_t start_addr = 0;
	uint32_t end_addr = FLASHSIZE;

	sFLASH_EraseBulk();
	
	// update the array
	for(uint32_t i=start_addr; i<end_addr; i++) {
		flashmem[i] = 0;
		cycle_counter++;
	}

	printf("eraseBulk exit\r\n");

}

void readBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead) {

	printf("readBuffer enter\r\n");

	uint32_t start_addr = ReadAddr;
	uint32_t end_addr = ReadAddr+NumByteToRead;

	// depending on the level of fault tolerance, this could be
	// changed to return out of this if the end address does not fit
	if(end_addr > FLASHSIZE) {
		printf("!! warning: readBuffer: end_addr > FLASHSIZE\r\n");
		end_addr = FLASHSIZE;
	}

	sFLASH_ReadBuffer(flashmem, ReadAddr, NumByteToRead); // TODO: revist this for passing flashmem

	printf("read it\r\n");
	for(uint32_t i=start_addr; i<end_addr; i++) {
		//*(pBuffer+i) = flashmem[i];
		printf("[%d] = %d\r\n", i, *(pBuffer+i));
	}
	
	printf("readBuffer exit\r\n");

}


void writePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
	
	printf("writePage enter\r\n");

	uint32_t start_addr = WriteAddr;
	uint32_t end_addr = WriteAddr+NumByteToWrite;

	// depending on the level of fault tolerance, this could be
	// changed to return out of this if the end address does not fit
	// TODO: unsure in reality if this is how to handle writing pages
	if(end_addr > (WriteAddr+PAGESIZE)) {
		printf("!! warning: writePage: end_addr > PAGESIZE\r\n");
		return;
	}

	uint8_t inc = 0;
	for(uint32_t i=start_addr; i<end_addr; i++) {
		flashmem[i] = *(pBuffer+inc); // ehhh just for simulation
		inc++;
	}

	printf("writePage exit\r\n");

}


void writeBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
	
	printf("writeBuffer enter\r\n");

	uint32_t start_addr = WriteAddr;
	uint32_t end_addr = WriteAddr+NumByteToWrite;

	// depending on the level of fault tolerance, this could be
	// changed to return out of this if the end address does not fit
	if(end_addr > FLASHSIZE) {
		printf("!! warning: writePage: end_addr > FLASHSIZE\r\n");
		return;
	}

	for(uint32_t i=start_addr; i<end_addr; i++) {
		flashmem[i] = *(pBuffer+i); // ehhh just for simulation
	}

	printf("writeBuffer exit\r\n");

}













