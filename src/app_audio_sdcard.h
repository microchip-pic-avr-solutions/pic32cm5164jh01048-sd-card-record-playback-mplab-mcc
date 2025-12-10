/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_audio_sdcard.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef APP_AUDIO_SDCARD_H
#define APP_AUDIO_SDCARD_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "definitions.h"
#include "ssd1306.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#define BUFFER_TX_BTCTRL_FOR_PLAYBACK       (DMAC_BTCTRL_STEPSIZE_X2 | DMAC_BTCTRL_SRCINC_Msk |     \
                                        DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_VALID_Msk)     /* DMAC settings */
    
#define BUFFER_TX_BTCTRL_FOR_RECORD         (DMAC_BTCTRL_STEPSIZE_X2 | DMAC_BTCTRL_STEPSEL_Msk | DMAC_BTCTRL_DSTINC_Msk |     \
                                        DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_VALID_Msk)     /* DMAC settings */   
    
#define MAX_AUDIO_DATA_BUFFER_SIZE                  4096U                                /* max number of audio data samples in buffer */
#define MAX_SD_CARD_BUFFER_SIZE                     8192U                                /* max number of SD card data bytes in buffer */
#define MAX_AUDIO_DATA_BUFFER_SIZE_FOR_RECORD       MAX_AUDIO_DATA_BUFFER_SIZE
#define MAX_SD_CARD_BUFFER_SIZE_FOR_RECORD          MAX_SD_CARD_BUFFER_SIZE
#define MAX_AUDIO_DATA_BUFFER_SIZE_FOR_PLAYBACK     512U
#define MAX_SD_CARD_BUFFER_SIZE_FOR_PLAYBACK        1024U
#define NUM_AUDIO_BUFFERS                           3U
#define NUM_AUDIO_BUFFERS_FOR_RECORD                NUM_AUDIO_BUFFERS
#define NUM_AUDIO_BUFFERS_FOR_PLAYBACK              2U
#define MAX_MOUNT_ATTEMPTS                          100U                                 /* max number of mount attempts before issuing error */ 
#define WAV_HEADER_SIZE                             44U                                  /* number of bytes in the WAV header */
#define WAV_HEADER_FILE_SIZE_INDEX                  4U                                   /* index for WAV file size chunk */
#define WAV_HEADER_NUM_CHANNELS_INDEX               22U                                  /* index for WAV number of channels, needs to be equal to 1 (mono) in the WAV header */
#define WAV_HEADER_FREQUENCY_INDEX                  24U                                  /* index for WAV sample rate, needs to be equal to 48000 Hz in the WAV header */
#define WAV_HEADER_BYTES_PER_SAMPLE_INDEX           32U                                  /* index for WAV bytes per block, needs to be equal to 2 in the WAV header */
#define WAV_HEADER_BITS_PER_SAMPLE_INDEX            34U                                  /* index for WAV bits per sample, needs to be equal to 16 in the WAV header */
#define WAV_HEADER_DATA_SIZE_INDEX                  40U                                  /* index for WAV file data size chunk */
#define SDCARD_FILE_ALLOCATION_SIZE                 65536U                               /* cluster size in number of bytes set at the SD card format process */
#define SDCARD_SECTOR_SIZE                          512U                                 /* SD card sector size */
#define NUM_BUTTONS                                 4U                                   /* number of buttons used on the board */
#define DEBOUNCE_DELAY_MS                           20U                                  /* debounce time in ms */
#define FILE_NAME_RECORD_AND_PLAYBACK               "RecordAndPlayback.wav"              /* name of the file to record in; make sure this file is present on the SD card and has the correct format */
#define MAX_FILES                                   20U                                  /* max number of files that will be read from the SD card */
#define AUDIO_DATA_OFFSET                           (SDCARD_FILE_ALLOCATION_SIZE + 2U)   /* reading from the 2nd audio sample (new cluster boundary + 2 bytes) offers significantly reduced SD card read time */
#define ADC_MAX_VALUE                               4095U                                /* max ADC result value */
#define SEEK_THRESHOLD                              62U                                  /* threshold (in ADC counts) at which a file seek sequence is started (to account for aprox. 80mV peak-to-peak POT voltage fluctuation */
#define OLED_VISIBLE_CHARS                          21U                                  /* 16 for 8x16 font */
#define MAX_DMA_I2C_DATA_BUFFER_LEN                 255U                                 /* max number of data bytes the DMA can transmit via I2C in one transaction */
#define SEEK_CHANNEL_INDEX                          1U                                   /* AIN7 POT output */
#define VU_CHANNEL_INDEX                            0U                                   /* AIN0 DAC output */
#define MCP23008_I2C_ADDRESS                        0x25U
#define IODIR_REG_ADDRESS                           0x00U
#define GPIO_REG_ADDRESS                            0x09U
#define NUM_WS2812_LEDS                             8U
#define WS2812_BYTES_PER_LED                        3U
#define WS2812_BRIGHTNESS_12P5PCT                   32U
#define WS2812_BRIGHTNESS_25PCT                     64U
#define WS2812_BRIGHTNESS_50PCT                     128U
#define WS2812_BRIGHTNESS_100PCT                    255U
#define HSV_HUE_MAX                                 255U
#define HSV_SATURATION_MAX                          255U
#define HSV_REGION_COUNT                            6U
#define HSV_REGION_WIDTH                            43U
#define HSV_HUE_INTERVAL_250MS                      1U
#define HSV_HUE_INTERVAL_500MS                      2U
#define HSV_HUE_INTERVAL_1S                         4U
#define HSV_HUE_INTERVAL_2S                         8U
#define TOUCH_PROCESS_CALL_INTERVAL_MS              10U

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    APP_AUDIO_STATE_INIT,
    APP_AUDIO_STATE_IDLE,
    APP_AUDIO_STATE_WAIT,
    APP_AUDIO_STATE_MOUNT,
    APP_AUDIO_STATE_UNMOUNT,
    APP_AUDIO_STATE_CURRENT_DRIVE_SET,
    APP_AUDIO_STATE_FILE_OPEN,
    APP_AUDIO_STATE_FILE_SIZE_READ,
    APP_AUDIO_STATE_WAV_HEADER_READ,
    APP_AUDIO_STATE_INITIAL_DUMMY_READ,
    APP_AUDIO_STATE_WAV_HEADER_WRITE,        
    APP_AUDIO_STATE_DATA_READ,
    APP_AUDIO_STATE_AUDIO_PLAY,
    APP_AUDIO_STATE_NEXT_FILE_PLAY,
    APP_AUDIO_STATE_MIC_RECORD,
    APP_AUDIO_STATE_DATA_WRITE,
    APP_AUDIO_STATE_DATA_PAD_WRITE,
    APP_AUDIO_STATE_FILE_TRUNCATE,
    APP_AUDIO_STATE_FILES_NAME_GET,
    APP_AUDIO_STATE_FILE_NAME_SELECT,
    APP_AUDIO_STATE_ERROR                    
} APP_AUDIO_STATES;


// *****************************************************************************

typedef enum
{
    BUFFER_EMPTY,
    BUFFER_FILLING,
    BUFFER_FULL,
    BUFFER_WRITING
} APP_AUDIO_BUFFER_STATE;

typedef struct
{
    uint16_t *data;
    uint16_t sampleNumber;
    APP_AUDIO_BUFFER_STATE state;
} APP_AUDIO_BUFFER;

typedef struct
{
    APP_AUDIO_BUFFER buffer1;
    APP_AUDIO_BUFFER buffer2;
    APP_AUDIO_BUFFER buffer3;
} APP_AUDIO_DATA;

typedef struct
{
    uint8_t dataBufferPlayback[MAX_SD_CARD_BUFFER_SIZE_FOR_PLAYBACK];
    uint8_t dataBufferRecord[MAX_SD_CARD_BUFFER_SIZE_FOR_RECORD];
    uint16_t byteNumber;       
} APP_WAV_DATA;

typedef enum
{
    PCM_FORMAT_16BIT_SIGNED,
    PCM_FORMAT_10BIT_UNSIGNED
} APP_PCM_FORMAT;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* Current application state */
    APP_AUDIO_STATES state;
    /* Audio data to speaker */
    APP_AUDIO_DATA audioData;
    /* Audio data from SDCARD */
    APP_WAV_DATA wavData;
    /* File Handle for .wav file */
    SYS_FS_HANDLE fileHandle;
    /* File Size */
    int32_t fileSize;
    /* File Name */
    char fileName[SYS_FS_FILE_NAME_LEN + 1];
    /* Current file position or number of bytes read from file */
    int32_t currentFilePosition;
    /* PCM format of the current file */
    APP_PCM_FORMAT pcmFormat;
} APP_AUDIO_SESSION;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_AUDIO_SDCARD_Initialize(void);

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_AUDIO_SDCARD_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_AUDIO_Initialize(void); 

/*******************************************************************************
  Function:
    void APP_AUDIO_SDCARD_Tasks (void)

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_SDCARD_AUDIO_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_AUDIO_Tasks( void );

static bool APP_AUDIO_ReadSDCardData( 
    const DRV_HANDLE handle,
    uint8_t* const pBuffer,
    const uint16_t requestedBytes,
    uint16_t* const pNumBytesRead);

static bool APP_AUDIO_WriteSDCardData( 
    const DRV_HANDLE handle,
    uint8_t* const pBuffer,
    const uint16_t requestedBytes,
    uint16_t* const pNumBytesWritten);

static uint16_t APP_AUDIO_DecodeSDCardData(
    const uint8_t* const pInBuffer, 
    uint16_t* const pOutBuffer,
    const uint16_t nInputBytes, 
    uint16_t* const pnBytesDecoded,
    APP_PCM_FORMAT pcmFormat);

static uint16_t APP_AUDIO_EncodeSDCardData(
    const uint16_t* const pInBuffer, 
    uint8_t* const pOutBuffer,
    const uint16_t nInputBytes, 
    uint16_t* const pnBytesEncoded);

static void APP_AUDIO_InitializeTxLinkedListDescriptorPlayback(void);
static void APP_AUDIO_InitializeTxLinkedListDescriptorRecord(void);
static void APP_AUDIO_SeekAudioFile(uint16_t adcResult);
static void APP_AUDIO_SetRGBLED(bool red, bool green, bool blue);
static void APP_AUDIO_I2CStartDMATransfer(uint16_t address, size_t dataBufferSize);
static void APP_AUDIO_OLEDBuildFileNameBuffer(uint8_t *oledBuffer, size_t *oledBufferLen);
static void APP_AUDIO_OLEDBuildPlaybackTimestampBuffer(uint8_t *oledBuffer, size_t *oledBufferLen, uint32_t elapsed, uint32_t total);
static void APP_AUDIO_OLEDBuildRecordTimestampBuffer(uint8_t *oledBuffer, size_t *oledBufferLen, uint32_t elapsed);
static void APP_AUDIO_IOExpanderWriteRegister(uint8_t reg, uint8_t value);
static void APP_AUDIO_UpdateVUMeter(uint16_t adcResult);
static void APP_AUDIO_UpdateWS2812(uint16_t adcResult);
static void APP_AUDIO_HSVtoRGB(uint8_t hue, uint8_t saturation, uint8_t value, uint8_t* outRed, uint8_t* outGreen, uint8_t* outBlue);
static void DAC_Enable(void);
static void DAC_Disable(void);
static void DMAC_TransmitCompleteCallback_CH0(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle);
static void DMAC_TransmitCompleteCallback_CH1(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle);
static void TC4_OverflowCallback(TC_TIMER_STATUS status, uintptr_t context);
static void TC6_OverflowCallback(TC_TIMER_STATUS status, uintptr_t context);
static void DMAC_TransmitCompleteCallback_CH5(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle);
static void TC7_OverflowCallback(TC_TIMER_STATUS status, uintptr_t context);
static void DMAC_TransmitCompleteCallback_CH4(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle);
static void ADC1_ResultReadyCallback(ADC_STATUS status, uintptr_t context);
static void DMAC_TransmitCompleteCallback_CH6(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle);
static void TCC1_OverflowCallback(uint32_t status, uintptr_t context);


#endif /* APP_AUDIO_SDCARD_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

