#ifndef SDCARD_H
#define SDCARD_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include "sd_spi.h"
#include "diskio.h"


FRESULT SD_Initialize(void);
bool SD_IsInserted(void);
FRESULT SD_GetInfo(uint32_t *totalMB, uint32_t *freeMB);
FRESULT SD_Deinitialize(void);

FRESULT SD_Format(void);
FRESULT SD_ListDirectory(const char *path, void (*print_fn)(const char *fmt, ...));

FRESULT SD_CreateFile(const char *path);
FRESULT SD_WriteToFile(const char *path, const void *data, uint32_t length, uint16_t *written);
FRESULT SD_ReadFromFile(const char *path, void *buffer, uint32_t bufsize, uint32_t *bytesRead);
FRESULT SD_DeleteFile(const char *path);

typedef struct {
    uint8_t *buffer;
    uint32_t length;
    char filename[64];
    bool write; // true for write, false for read
    SemaphoreHandle_t done; // signal completion
} SD_Job_t;

#endif
