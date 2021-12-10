#include "spi_flash.h"

void sFLASH_DeInit(void) {
	printf("FLASH: going to de-init\r\n");
	sFLASH_WaitForWriteEnd();
	printf("FLASH: finished de-init\r\n");
}

void sFLASH_Init(void) {
	printf("FLASH: going to init\r\n");
	printf("FLASH: the id is: %X\r\n", sFLASH_ReadID());
	// FUTURE: check the ID received is the same as sFLASH_M25P128_ID
	sFLASH_SendByte(sFLASH_CMD_RDSR); // read status register 
	// FUTURE: do something with status register if it reports an error
}

void sFLASH_EraseSector(uint32_t SectorAddr) {
	printf("FLASH: going to erase sector at %X\r\n", SectorAddr);
	sFLASH_WriteEnable(); // as per the documentation, write enable is needed before erase
	// TODO: drive /CS low
	sFLASH_SendByte(sFLASH_CMD_SE);
	// TODO: drive /CS high
}

void sFLASH_EraseBulk(void) {
	printf("FLASH: going to erase bulk\r\n");
	sFLASH_WriteEnable(); // as per the documentation, write enable is needed before erase
	// TODO: drive /CS low
	sFLASH_SendByte(sFLASH_CMD_BE);
	// TODO: drive /CS high
}

void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
	printf("FLASH: write page buffer start: 0x%X num: %d\r\n", WriteAddr, NumByteToWrite);

	// TODO: does write page have different command than write buffer?

	uint32_t start_addr = WriteAddr;
	uint32_t end_addr = WriteAddr+NumByteToWrite;

	// guessing this is how it's done
	uint8_t inc = 0;
	for(uint32_t i=start_addr; i<end_addr; i++) {
		sFLASH_WriteEnable();
		sFLASH_SendByte(sFLASH_CMD_WRITE);
		sFLASH_SendByte(*(pBuffer+inc));
		inc++;
	}

}

void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
	printf("FLASH: write buffer start: 0x%X num: %d\r\n", WriteAddr, NumByteToWrite);

	uint32_t start_addr = WriteAddr;
	uint32_t end_addr = WriteAddr+NumByteToWrite;

	// guessing this is how it's done
	for(uint32_t i=start_addr; i<end_addr; i++) {
		sFLASH_WriteEnable();
		sFLASH_SendByte(sFLASH_CMD_WRITE);
		sFLASH_SendByte(*(pBuffer+i));
	}

}

void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead) {
	printf("FLASH: read buffer start: 0x%X num: %d\r\n", ReadAddr, NumByteToRead);
	sFLASH_SendByte(sFLASH_CMD_READ); // read from memory instruction
	// datasheet says a 24 bit address
	// QUESTION: how does the address get formatted when we're given 32 bit?
	// going with sending the entire 32 bit address...
	sFLASH_SendHalfWord(ReadAddr >> 16);
	sFLASH_SendHalfWord(ReadAddr |= (ReadAddr >> (16+1)));

	uint32_t start_addr = ReadAddr;
	uint32_t end_addr = ReadAddr+NumByteToRead;

	for(uint32_t i=start_addr; i<end_addr; i++) {
		*(pBuffer+i) = *(pBuffer+i);// = 7; // idk lol
	}
}

uint32_t sFLASH_ReadID(void) {
	printf("FLASH: read id\r\n");
	sFLASH_SendByte(sFLASH_CMD_RDID); // read identification
	return sFLASH_M25P128_ID;
}

void sFLASH_StartReadSequence(uint32_t ReadAddr) {
	// TODO
	// QUESTION: unsure what this one is for
	// maybe it's to start the read, then when it receives back
	// the address, just call this again until the end?
}

/* Low layer functions */
uint8_t sFLASH_ReadByte(void) {
	// TODO
	// QUESTION: unsure where this is used
	return 0;
}

uint8_t sFLASH_SendByte(uint8_t byte) {
	printf("FLASH: sending byte 0x%X\r\n", byte);
	return 0;
}

uint16_t sFLASH_SendHalfWord(uint16_t HalfWord) {
	printf("FLASH: sending half word %X\r\n", HalfWord);
	// QUESTION: unsure where halfword is used
	sFLASH_SendByte(HalfWord >> 8); // first half
	sFLASH_SendByte(HalfWord |= (HalfWord >> (8+1) )); // last half
	// ^ mask out the first half of halfword with |=
	// unsure of off by one there, might revisit in the future
	return 0;
}

void sFLASH_WriteEnable(void) {
	printf("FLASH: write enable\r\n");
	sFLASH_SendByte(sFLASH_CMD_WREN);
}

void sFLASH_WaitForWriteEnd(void) {
	printf("FLASH: waiting for write to end\r\n");
	// QUESTION: any commands supposed to be sent here?
	// FUTURE: there could be a timeout here, in case something
	// happens and it's waiting forever for the write to end
}


