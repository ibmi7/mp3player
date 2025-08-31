#ifndef SDCARD_H
#define SDCARD_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "sd_spi.h"
#include "diskio.h"

typedef enum {
    SD_JOB_CREATE,
    SD_JOB_READ,
    SD_JOB_WRITE,
    SD_JOB_GETINFO,
    SD_JOB_FORMAT,
    SD_JOB_LISTDIRECTORY,
    SD_JOB_STREAM,
    SD_JOB_DELETE
} SD_JobType_t;

typedef struct {
    uint32_t totalMB;
    uint32_t freeMB;
} SD_Info_t;

typedef enum {
    SD_JOB_PENDING,
    SD_JOB_SUCCESS,
    SD_JOB_FAILED
} SD_JobStatus_t;

typedef struct {
    SD_JobType_t type;
    char filename[_MAX_LFN]; // Max file length
    uint8_t *buffer;
    uint32_t length;
    uint32_t bytesTransferred;
    SD_JobStatus_t status;      // job completion status
    SD_Info_t* info;
    FRESULT fresult; 
} SD_Job_t;

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

bool SD_SubmitJob(SD_Job_t *job, osMessageQId queueHandle);



#endif
