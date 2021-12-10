Exercise 3b

gcc -I(path to folder goes here) main.c spi_flash.c -o gogo
./gogo

Output:

Welcome 2105368
init enter
FLASH: going to init
FLASH: read id
FLASH: sending byte 0x9F
FLASH: the id is: 202018
FLASH: sending byte 0x5
flashmem (10): 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
init exit
testing halfword with 0x834F
FLASH: sending half word 834F
FLASH: sending byte 0x83
FLASH: sending byte 0x4F
flashmem (10): 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
eraseSector enter
FLASH: going to erase sector at 2
FLASH: write enable
FLASH: sending byte 0x6
FLASH: sending byte 0xD8
eraseSector exit
flashmem (10): 1, 1, 0, 0, 0, 1, 1, 1, 1, 1
flashmem (10): 1, 1, 0, 0, 0, 1, 1, 1, 1, 1
eraseBulk enter
FLASH: going to erase bulk
FLASH: write enable
FLASH: sending byte 0x6
FLASH: sending byte 0xC7
eraseBulk exit
flashmem (10): 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
readBuffer enter
FLASH: read buffer start: 0x2 num: 6
FLASH: sending byte 0x3
FLASH: sending half word 0
FLASH: sending byte 0x0
FLASH: sending byte 0x0
FLASH: sending half word 2
FLASH: sending byte 0x0
FLASH: sending byte 0x2
read it
[2] = 0
[3] = 0
[4] = 0
[5] = 0
[6] = 0
[7] = 0
readBuffer exit
flashmem (10): 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
writePage enter
writePage exit
flashmem (10): 0, 0, 0, 9, 9, 0, 0, 0, 0, 0
writeBuffer enter
writeBuffer exit
flashmem (10): 4, 2, 4, 2, 4, 2, 4, 2, 4, 2
de-init
FLASH: going to de-init
FLASH: waiting for write to end
FLASH: finished de-init
de-init done
OK

