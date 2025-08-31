/* Includes ------------------------------------------------------------------*/

#include "sd_spi.h"

#define FCLK_SLOW() { MODIFY_REG(hspi1.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_128); }	/* Set SCLK = slow, approx 280 KBits/s*/
#define FCLK_FAST() { MODIFY_REG(hspi1.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_8); }	/* Set SCLK = fast, approx 4.5 MBits/s */

/** Functions ----------------------------------------------------------------*/

static void CS_LOW(void) {
    HAL_GPIO_WritePin(SD_SPI_CS_GPIO_Port, SD_SPI_CS_Pin, GPIO_PIN_RESET);
}

static void CS_HIGH(void) {
    HAL_GPIO_WritePin(SD_SPI_CS_GPIO_Port, SD_SPI_CS_Pin, GPIO_PIN_SET);
}

static bool SD_SPI_Sync(void) {
    BYTE dummy = 0xFF, rxData;
    uint32_t timeout = HAL_GetTick();


    CS_LOW();

    do {
        HAL_SPI_TransmitReceive(&hspi1, &dummy, &rxData, 1, 50);
        if (rxData == 0xFF) return true;
    } while (HAL_GetTick() - timeout < 500);

    return false;
}

static BYTE SD_SendCommand(BYTE cmd, uint32_t arg, BYTE crc) {
    BYTE buf[6];
    BYTE response = 0xFF;
    BYTE dummy = 0xFF;

    // Construct the command packet
    buf[0] = 0x40 | cmd;               // Command token
    buf[1] = (arg >> 24) & 0xFF;
    buf[2] = (arg >> 16) & 0xFF;
    buf[3] = (arg >> 8) & 0xFF;
    buf[4] = arg & 0xFF;
    buf[5] = crc;

    // Ensure at least one dummy clock and CS high before command
    CS_HIGH();
    HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY);

    // Pull CS low to select the card
    CS_LOW();

    // Send command
    for (int i = 0; i < 6; i++) {
        HAL_SPI_Transmit(&hspi1, &buf[i], 1, HAL_MAX_DELAY);
    }

    // Wait for response (up to 8 bytes), as per SD spec
    for (int i = 0; i < 8; i++) {
        HAL_SPI_TransmitReceive(&hspi1, &dummy, &response, 1, HAL_MAX_DELAY);
        if ((response & 0x80) == 0) {
            break;  // Valid response received
        }
    }


    return response;
}

DSTATUS SD_SPI_Init(void) {
    BYTE response;
    BYTE dummy = 0xFF;
    BYTE ocr[4];

    FCLK_SLOW();
    CS_HIGH();
    for (int i = 0; i < 10; i++) {
        HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY); // Send 80 dummy clocks
    }

    // CMD0: Go to idle state
    uint32_t timeout = HAL_GetTick();
	do
	{
		response = SD_SendCommand(CMD0, 0, 0x95);

	} while( (response != 0x01) && (HAL_GetTick() - timeout < 1000));

    if (response != 0x01) return STA_NOINIT;

    // CMD8: Send Interface Condition
    response = SD_SendCommand(CMD8, 0x1AA, 0x87);
    if (response != 0x01) return STA_NOINIT;

    // Read R7 response (4 bytes)
    for (int i = 0; i < 4; i++) {
        HAL_SPI_TransmitReceive(&hspi1, &dummy, &ocr[i], 1, HAL_MAX_DELAY);
    }

    if (ocr[2] != 0x01 || ocr[3] != 0xAA) return STA_NOINIT;

    // ACMD41: Initialize card
    for (uint32_t start = HAL_GetTick(); HAL_GetTick() - start < 1000;) {
        SD_SendCommand(CMD55, 0, 0x01);
        response = SD_SendCommand(ACMD41, 0x40000000, 0x01);
        if (response == 0x00) break;
    }
    if (response != 0x00) return STA_NOINIT;

    // CMD58: Read OCR
    response = SD_SendCommand(CMD58, 0, 0x01);
    if (response != 0x00) return STA_NOINIT;
    for (int i = 0; i < 4; i++) {
        HAL_SPI_TransmitReceive(&hspi1, &dummy, &ocr[i], 1, 50);
    }

    // Card is now initialized
    FCLK_FAST();
    CS_HIGH();

    return RES_OK;
}

DRESULT SD_SPI_ReadBlocks(BYTE *buff, uint32_t sector, uint32_t count)
{
    if (count == 0) return RES_ERROR;

    for (uint32_t i = 0; i < count; i++) {
        if (SD_SendCommand(CMD17, sector + i, 0xFF) != 0x00) return RES_ERROR;

        // Wait for 0xFE
        BYTE token;
        for (uint32_t t = 0; t < 100000; t++) {
            HAL_SPI_Receive(&hspi1, &token, 1, HAL_MAX_DELAY);
            if (token == 0xFE) break;
        }

        // Read 512 bytes
        HAL_SPI_Receive(&hspi1, &buff[i * 512], 512, HAL_MAX_DELAY);

        // Read 2-byte CRC
        BYTE dummy[2];
        HAL_SPI_Receive(&hspi1, dummy, 2, HAL_MAX_DELAY);
    }

    BYTE dummy = 0xFF;
    CS_HIGH();
    HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY);

    return RES_OK;
}

DRESULT SD_SPI_WriteBlocks(const BYTE *buff, uint32_t sector, uint32_t count)
{
	BYTE token;
	if (count == 0) return RES_PARERR;
	if (count == 1)
	{
		token = 0xFE;
		if (SD_SendCommand(CMD24, sector, 0xFF) != 0x00) return RES_ERROR;

		HAL_SPI_Transmit(&hspi1, &token, 1, HAL_MAX_DELAY);

		// Send 512-byte data block
		HAL_SPI_Transmit(&hspi1, buff, 512, HAL_MAX_DELAY);

		// Dummy CRC
		BYTE crc[2] = {0xFF, 0xFF};
		HAL_SPI_Transmit(&hspi1, crc, 2, HAL_MAX_DELAY);

		// Check data response
		BYTE resp;
		HAL_SPI_Receive(&hspi1, &resp, 1, HAL_MAX_DELAY);
		if ((resp & 0x1F) != 0x05) return RES_ERROR;
	}
	else
	{
		token = 0xFC;
		for (uint32_t i = 0; i < count; i++) {

			SD_SendCommand(CMD55, 0, 0x01);
			SD_SendCommand(ACMD23, count, 0xFF);

			if (SD_SendCommand(CMD25, sector + i, 0xFF) != 0x00) return RES_ERROR;

			HAL_SPI_Transmit(&hspi1, &token, 1, HAL_MAX_DELAY);

			// Send 512-byte data block
			HAL_SPI_Transmit(&hspi1, (BYTE*)&buff[i * 512], 512, HAL_MAX_DELAY);

			// Dummy CRC
			BYTE crc[2] = {0xFF, 0xFF};
			HAL_SPI_Transmit(&hspi1, crc, 2, HAL_MAX_DELAY);

			// Check data response
			BYTE resp;
			HAL_SPI_Receive(&hspi1, &resp, 1, HAL_MAX_DELAY);
			if ((resp & 0x1F) != 0x05) return RES_ERROR;
		}
		token = 0xFD;
		HAL_SPI_Transmit(&hspi1, &token, 1, HAL_MAX_DELAY);
		if (SD_SPI_Sync() == false) return RES_ERROR;
	}


    BYTE dummy = 0xFF;
    SD_SPI_Sync();
    CS_HIGH();
    HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY);

	return RES_OK;
}

DRESULT SD_SPI_IsInserted(void)
{
	return RES_OK;
}

DRESULT SD_SPI_ioctl(uint8_t pdrv, uint8_t cmd, void *buff)
{
	DRESULT res;
	switch(cmd)
	{
	case CTRL_SYNC : /* Wait for internal operations to be done */
		res = SD_SPI_Sync() ? RES_OK:RES_ERROR;
		break;
	default:
		res = RES_ERROR;
		break;
	}

    BYTE dummy = 0xFF;
    CS_HIGH();
    HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY);

	return res;
}




