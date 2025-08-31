#ifndef SD_SPI_H
#define SD_SPI_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "integer.h"
#include "fatfs.h"

/* COMNMAND */

#define CMD0    0   // GO_IDLE_STATE
#define CMD1    1   // SEND_OP_COND (MMC)
#define CMD8    8   // SEND_IF_COND
#define CMD9    9   // SEND_CSD
#define CMD10   10  // SEND_CID
#define CMD12   12  // STOP_TRANSMISSION
#define CMD17   17  // READ_SINGLE_BLOCK
#define CMD18   18  // READ_MULTIPLE_BLOCK
#define CMD24   24  // WRITE_BLOCK
#define CMD25   25  // WRITE_MULTIPLE_BLOCK
#define CMD55   55  // APP_CMD
#define CMD58   58  // READ_OCR

// Application-specific commands (must be preceded by CMD55)
#define ACMD41  41  // SD_SEND_OP_COND
#define ACMD22  22  // SEND_NUM_WR_BLOCKS
#define ACMD23  23  // SET_WR_BLK_ERASE_COUNT


/* SPI  definition*/

extern SPI_HandleTypeDef hspi1;

/* FUNCTIONS PROTOTYPES */

DSTATUS SD_SPI_Init(void);
DRESULT SD_SPI_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
DRESULT SD_SPI_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
DRESULT SD_SPI_IsInserted(void);
DRESULT SD_SPI_ioctl(uint8_t pdrv, uint8_t cmd, void *buff);

#endif
