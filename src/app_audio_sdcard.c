/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_tone_textfile_sdcard.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015-2016 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#include "app_audio_sdcard.h"
#include <inttypes.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

/* The record and playback feature works with 1 channel (mono), 10-bit unsigned data, sampled at 48000 Hz.
   The system can also play 16-bit signed PCM audio samples using addition and right shifting. */

static uint16_t __attribute__((aligned(16))) audioDataBuffer1[MAX_AUDIO_DATA_BUFFER_SIZE];
static uint16_t __attribute__((aligned(16))) audioDataBuffer2[MAX_AUDIO_DATA_BUFFER_SIZE];
static uint16_t __attribute__((aligned(16))) audioDataBuffer3[MAX_AUDIO_DATA_BUFFER_SIZE];
static dmac_descriptor_registers_t __attribute__((aligned(16))) pTxLinkedListDesc[NUM_AUDIO_BUFFERS];
static APP_AUDIO_SESSION appSession;
static volatile uint8_t dmaBufferFillIndex = 0U;
static volatile uint8_t bufferProcessIndex = 0U;
static volatile uint8_t bufferFullCount = 0U;
static volatile bool bufferOverrun = false;
static volatile bool fileEnded = false;
static volatile bool firstBufferPartiallyFilledOnFirstRead = false;
static volatile bool lastBufferProcessed = false;
static volatile bool audioStarted = false;
static volatile bool record = false;
static volatile bool playback = false;
static volatile bool button1Pressed = false;
static volatile bool button2Pressed = false;
static volatile bool button3Pressed = false;
static volatile bool button4Pressed = false;
static volatile bool touchButtonPressed = false;
static volatile bool seekingEnabled = false;
static volatile uint16_t lastSeekAdcResult = 0U;
static volatile bool seekPending = false;
static volatile size_t scrollPos = 0U;
static volatile bool dmaTransferInProgress = false;
static volatile bool playbackPaused = false;
static volatile bool recordPaused = false;
static char currentFileName[SYS_FS_FILE_NAME_LEN+1] = {0};
static volatile uint32_t totalSeconds = 0U;
static volatile uint32_t elapsedSeconds = 0U;
static volatile uint8_t overflowCount = 0U;
static volatile bool timestampUpdatePending = false;
static volatile bool stopDisplayTimer = false;
static volatile uint16_t adcResults[2];
static uint8_t WS2812DataBuffer[NUM_WS2812_LEDS * WS2812_BYTES_PER_LED];
static volatile bool dmaToSPITransferInProgress = false;
static volatile bool WS2812OutputDisablePending = false;
static const uint8_t hueIntervalOptions[] = {HSV_HUE_INTERVAL_2S, HSV_HUE_INTERVAL_1S, HSV_HUE_INTERVAL_500MS, HSV_HUE_INTERVAL_250MS};
static const uint8_t maxBrightnessOptions[] = {WS2812_BRIGHTNESS_12P5PCT, WS2812_BRIGHTNESS_25PCT, WS2812_BRIGHTNESS_50PCT, WS2812_BRIGHTNESS_100PCT};
static uint8_t hueIntervalIndex = 0U;
static uint8_t maxBrightnessIndex = 0U;

static APP_AUDIO_BUFFER* buffers[NUM_AUDIO_BUFFERS] = 
{
    &appSession.audioData.buffer1,
    &appSession.audioData.buffer2,
    &appSession.audioData.buffer3
};

/* Initializes the DMA linked list descriptors for playback. */
static void APP_AUDIO_InitializeTxLinkedListDescriptorPlayback(void)
{
    /* Configure first descriptor for buffer1. */
    pTxLinkedListDesc[0].DMAC_BTCTRL     = (uint16_t)BUFFER_TX_BTCTRL_FOR_PLAYBACK;
    pTxLinkedListDesc[0].DMAC_BTCNT      = appSession.audioData.buffer1.sampleNumber;
    pTxLinkedListDesc[0].DMAC_DESCADDR   = (uint32_t)&pTxLinkedListDesc[1];
    pTxLinkedListDesc[0].DMAC_DSTADDR    = (uint32_t)&DAC_REGS->DAC_DATA;
    pTxLinkedListDesc[0].DMAC_SRCADDR    = (uint32_t)((uintptr_t)appSession.audioData.buffer1.data + (uint32_t)(appSession.audioData.buffer1.sampleNumber * (uint32_t)sizeof(audioDataBuffer1[0])));
    /* Configure second descriptor for buffer2. */
    pTxLinkedListDesc[1].DMAC_BTCTRL     = (uint16_t)BUFFER_TX_BTCTRL_FOR_PLAYBACK;
    pTxLinkedListDesc[1].DMAC_BTCNT      = appSession.audioData.buffer2.sampleNumber;
    pTxLinkedListDesc[1].DMAC_DESCADDR   = (uint32_t)&pTxLinkedListDesc[0];
    pTxLinkedListDesc[1].DMAC_DSTADDR    = (uint32_t)&DAC_REGS->DAC_DATA;
    pTxLinkedListDesc[1].DMAC_SRCADDR    = (uint32_t)((uintptr_t)appSession.audioData.buffer2.data + (uint32_t)(appSession.audioData.buffer2.sampleNumber * (uint32_t)sizeof(audioDataBuffer2[0])));
}

/* Initializes the DMA linked list descriptors for recording. */
static void APP_AUDIO_InitializeTxLinkedListDescriptorRecord(void)
{
    /* Configure first descriptor for buffer1. */
    pTxLinkedListDesc[0].DMAC_BTCTRL     = (uint16_t)BUFFER_TX_BTCTRL_FOR_RECORD;
    pTxLinkedListDesc[0].DMAC_BTCNT      = (uint16_t)MAX_AUDIO_DATA_BUFFER_SIZE_FOR_RECORD;
    pTxLinkedListDesc[0].DMAC_DESCADDR   = (uint32_t)&pTxLinkedListDesc[1];
    pTxLinkedListDesc[0].DMAC_DSTADDR    = (uint32_t)((uintptr_t)appSession.audioData.buffer1.data + (uint32_t)(MAX_AUDIO_DATA_BUFFER_SIZE_FOR_RECORD * (uint32_t)sizeof(audioDataBuffer1[0])));
    pTxLinkedListDesc[0].DMAC_SRCADDR    = (uint32_t)&ADC1_REGS->ADC_RESULT;
    /* Configure second descriptor for buffer2. */
    pTxLinkedListDesc[1].DMAC_BTCTRL     = (uint16_t)BUFFER_TX_BTCTRL_FOR_RECORD;
    pTxLinkedListDesc[1].DMAC_BTCNT      = (uint16_t)MAX_AUDIO_DATA_BUFFER_SIZE_FOR_RECORD;
    pTxLinkedListDesc[1].DMAC_DESCADDR   = (uint32_t)&pTxLinkedListDesc[2];
    pTxLinkedListDesc[1].DMAC_DSTADDR    = (uint32_t)((uintptr_t)appSession.audioData.buffer2.data + (uint32_t)(MAX_AUDIO_DATA_BUFFER_SIZE_FOR_RECORD * (uint32_t)sizeof(audioDataBuffer2[0])));
    pTxLinkedListDesc[1].DMAC_SRCADDR    = (uint32_t)&ADC1_REGS->ADC_RESULT;
    /* Configure third descriptor for buffer3. */
    pTxLinkedListDesc[2].DMAC_BTCTRL     = (uint16_t)BUFFER_TX_BTCTRL_FOR_RECORD;
    pTxLinkedListDesc[2].DMAC_BTCNT      = (uint16_t)MAX_AUDIO_DATA_BUFFER_SIZE_FOR_RECORD;
    pTxLinkedListDesc[2].DMAC_DESCADDR   = (uint32_t)&pTxLinkedListDesc[0];
    pTxLinkedListDesc[2].DMAC_DSTADDR    = (uint32_t)((uintptr_t)appSession.audioData.buffer3.data + (uint32_t)(MAX_AUDIO_DATA_BUFFER_SIZE_FOR_RECORD * (uint32_t)sizeof(audioDataBuffer3[0])));
    pTxLinkedListDesc[2].DMAC_SRCADDR    = (uint32_t)&ADC1_REGS->ADC_RESULT;
}

/* Sets the RGB LED state. */
static void APP_AUDIO_SetRGBLED(bool red, bool green, bool blue)
{
    /* Set or clear each color LED based on input flags. */
    if (red)
    {
        LED_RED_Set();
    }
    else
    {
        LED_RED_Clear();
    }

    if (green)
    {
        LED_GREEN_Set();
    }
    else
    {
        LED_GREEN_Clear();
    }

    if (blue)
    {
        LED_BLUE_Set();
    }
    else
    {
        LED_BLUE_Clear();
    }
}

/* Seeks the audio file to a new position based on ADC input. */
static void APP_AUDIO_SeekAudioFile(uint16_t adcResult)
{
    /* Calculate the size of the audio data section. */
    uint64_t audioDataSize = 0U;
    if (appSession.fileSize >= ((int32_t)SDCARD_FILE_ALLOCATION_SIZE + 2))
    {
        audioDataSize = (uint64_t)appSession.fileSize - ((uint64_t)SDCARD_FILE_ALLOCATION_SIZE + 2U);
    }
    else
    {
        audioDataSize = 0U;
    }
    /* Map ADC value to a new offset within the audio data. */
    uint64_t newDataOffset = ((uint64_t)adcResult * audioDataSize) / (uint64_t)(ADC_MAX_VALUE);
    /* Calculate the new file position (add offset to start of audio data). */
    uint32_t newFilePosition = SDCARD_FILE_ALLOCATION_SIZE + 2U + (uint32_t)newDataOffset;
    /* Ensure 2-byte alignment. */
    if ((newFilePosition % 2U) != 0U)
    {
        newFilePosition++;
    }
    /* Ensure the new file position is not a multiple of the SD card sector size. */
    if ((newFilePosition % (uint32_t)SDCARD_SECTOR_SIZE) == 0U)
    {
        newFilePosition += 2U;
    }
    /* Clamp to valid range. */
    if (newFilePosition >= (uint32_t)appSession.fileSize)
    {
        if (appSession.fileSize >= 2)
        {
            newFilePosition = (uint32_t)appSession.fileSize - 2U;
        }
        else
        {
            newFilePosition = 0U;
        }
    }
    /* Only seek if the file position has changed. */
    if ((int32_t)newFilePosition != appSession.currentFilePosition)
    {
        if (newFilePosition <= (uint32_t)INT32_MAX)
        {
            if (SYS_FS_FileSeek(appSession.fileHandle, (int32_t)newFilePosition, SYS_FS_SEEK_SET) != -1)
            {
                appSession.currentFilePosition = (int32_t)newFilePosition;
            }
            else
            {
                /* Error on file seek. */
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
        }
    }
}

/* Enables the DAC peripheral. */
static void DAC_Enable(void)
{
    DAC_REGS->DAC_CTRLA = (uint8_t)(DAC_CTRLA_ENABLE_Msk);
}

/* Disables the DAC peripheral. */
static void DAC_Disable(void)
{
    DAC_REGS->DAC_CTRLA &= (uint8_t)(~DAC_CTRLA_ENABLE_Msk);        
}

/* Starts the DMA transaction on the I2C bus. */
static void APP_AUDIO_I2CStartDMATransfer(uint16_t address, size_t dataBufferSize)
{
    SERCOM0_REGS->I2CM.SERCOM_ADDR = SERCOM_I2CM_ADDR_ADDR(((uint32_t)address << 1U)) | SERCOM_I2CM_ADDR_LEN((uint32_t)dataBufferSize) | SERCOM_I2CM_ADDR_LENEN(1U);
}

/* Builds the OLED filename buffer for display. */
static void APP_AUDIO_OLEDBuildFileNameBuffer(uint8_t *oledBuffer, size_t *oledBufferLen)
{
    char displayBuf[OLED_VISIBLE_CHARS + 1U] = {0};
    /* Copy visible portion of filename to display buffer. */
    (void)strncpy(displayBuf, currentFileName + scrollPos, OLED_VISIBLE_CHARS);
    displayBuf[OLED_VISIBLE_CHARS] = '\0';
    *oledBufferLen = SSD1306_BuildI2CBuffer(0U, displayBuf, oledBuffer);
}

/* Builds the OLED playback timestamp buffer for display. */
static void APP_AUDIO_OLEDBuildPlaybackTimestampBuffer(uint8_t *oledBuffer, size_t *oledBufferLen, uint32_t elapsed, uint32_t total)
{
    char timeStr[32];
    /* Format elapsed and total time as string. */
    (void)snprintf(timeStr, sizeof(timeStr), " %02" PRIu32 ":%02" PRIu32 ":%02" PRIu32 " / %02" PRIu32 ":%02" PRIu32 ":%02" PRIu32 " ",
        (uint32_t)(elapsed / 3600U),
        (uint32_t)((elapsed / 60U) % 60U),
        (uint32_t)(elapsed % 60U),
        (uint32_t)(total / 3600U),
        (uint32_t)((total / 60U) % 60U),
        (uint32_t)(total % 60U));
    *oledBufferLen = SSD1306_BuildI2CBuffer(3U, timeStr, oledBuffer);
}

/* Builds the OLED record timestamp buffer for display. */
static void APP_AUDIO_OLEDBuildRecordTimestampBuffer(uint8_t *oledBuffer, size_t *oledBufferLen, uint32_t elapsed)
{
    char timeStr[32];
    /* Format elapsed time as string for recording. */
    (void)snprintf(timeStr, sizeof(timeStr), "       %02" PRIu32 ":%02" PRIu32 ":%02" PRIu32 "      ",
        (uint32_t)(elapsed / 3600U),
        (uint32_t)((elapsed / 60U) % 60U),
        (uint32_t)(elapsed % 60U));
    *oledBufferLen = SSD1306_BuildI2CBuffer(3U, timeStr, oledBuffer);
}

/* Writes a value to a register on the I/O expander (MCP23008). */
static void APP_AUDIO_IOExpanderWriteRegister(uint8_t reg, uint8_t value)
{
    static uint8_t expanderData[2];
    expanderData[0] = reg;
    expanderData[1] = value;
    while (SERCOM0_I2C_IsBusy() != false)	
    {
        ;
    }
    if (SERCOM0_I2C_Write(MCP23008_I2C_ADDRESS, expanderData, (uint32_t)sizeof(expanderData)) == true)
    {
        /* No action required. */
    }
    else
    {
        /* No action required. */
    }
    while (SERCOM0_I2C_IsBusy() != false)	
    {
        ;
    }
}

/* Updates the VU meter LEDs based on the ADC result. */
static void APP_AUDIO_UpdateVUMeter(uint16_t adcResult)
{
    /* Center the ADC value. */
    int16_t centeredValue = (int16_t)adcResult - (((int16_t)ADC_MAX_VALUE + 1) / 2);
    /* Calculate amplitude (absolute value). */
    uint16_t amplitude = 0U;
    if (centeredValue == (int16_t)INT16_MIN)
    {
        amplitude = (uint16_t)INT16_MAX;
    }
    else
    {
        amplitude = (uint16_t)((centeredValue < 0) ? (uint16_t)(-centeredValue) : (uint16_t)centeredValue);
    }
    /* Turn all LEDs off. */
    LED7_Set();
    LED6_Set();
    LED5_Set();
    LED4_Set();
    LED3_Set();
    LED2_Set();
    LED1_Set();
    LED0_Set();
    /* Turn on LEDs based on amplitude thresholds. */
    LED7_Clear();
    if (amplitude >= (uint16_t)(((uint16_t)(ADC_MAX_VALUE + 1U) / 16U) * 1U))
    {
        LED6_Clear();
    }
    if (amplitude >= (uint16_t)(((uint16_t)(ADC_MAX_VALUE + 1U) / 16U) * 2U))
    {
        LED5_Clear();
    }
    if (amplitude >= (uint16_t)(((uint16_t)(ADC_MAX_VALUE + 1U) / 16U) * 3U))
    {
        LED4_Clear();
    }
    if (amplitude >= (uint16_t)(((uint16_t)(ADC_MAX_VALUE + 1U) / 16U) * 4U))
    {
        LED3_Clear();
    }
    if (amplitude >= (uint16_t)(((uint16_t)(ADC_MAX_VALUE + 1U) / 16U) * 5U))
    {
        LED2_Clear();
    }
    if (amplitude >= (uint16_t)(((uint16_t)(ADC_MAX_VALUE + 1U) / 16U) * 6U))
    {
        LED1_Clear();
    }
    if (amplitude >= (uint16_t)(((uint16_t)(ADC_MAX_VALUE + 1U) / 16U) * 7U))
    {
        LED0_Clear();
    }  
}

static void APP_AUDIO_HSVtoRGB(uint8_t hue, uint8_t saturation, uint8_t value, uint8_t* outRed, uint8_t* outGreen, uint8_t* outBlue)
{
    uint8_t colorRegion;
    uint8_t regionPosition; 
    uint8_t fadeToGray; 
    uint8_t fadeBetweenColors1;
    uint8_t fadeBetweenColors2;
    /* If saturation is zero, output is a shade of gray (no color). */
    if (saturation == 0U)
    {
        *outRed = value;
        *outGreen = value;
        *outBlue = value;
    }
    else
    {
        /* Divide hue into 6 regions (0-5). Each region is about approx 43 units wide. */
        colorRegion = (uint8_t)(hue / HSV_REGION_WIDTH);
        /* Position within the region (0-255). */
        regionPosition = (uint8_t)(((uint16_t)hue - ((uint16_t)colorRegion * (uint16_t)HSV_REGION_WIDTH)) * HSV_REGION_COUNT);
        /* Calculate fades for RGB conversion. */
        fadeToGray = (uint8_t)(((uint16_t)value * ((uint16_t)HSV_HUE_MAX - (uint16_t)saturation)) >> 8U);
        fadeBetweenColors1 = (uint8_t)(((uint16_t)value * ((uint16_t)HSV_HUE_MAX - (((uint16_t)saturation * (uint16_t)regionPosition) >> 8U))) >> 8U);
        fadeBetweenColors2 = (uint8_t)(((uint16_t)value * ((uint16_t)HSV_HUE_MAX - (((uint16_t)saturation * ((uint16_t)HSV_HUE_MAX - (uint16_t)regionPosition)) >> 8U))) >> 8U);
        /* Assign RGB based on the region. */
        switch (colorRegion) 
        {
            /* Red to Yellow. */
            case 0U: 
                *outRed   = value;
                *outGreen = fadeBetweenColors2;
                *outBlue  = fadeToGray;
                break;
            /* Yellow to Green. */
            case 1U:
                *outRed   = fadeBetweenColors1;
                *outGreen = value;
                *outBlue  = fadeToGray;
                break;
            /* Green to Cyan. */    
            case 2U:
                *outRed   = fadeToGray;
                *outGreen = value;
                *outBlue  = fadeBetweenColors2;
                break;
            /* Cyan to Blue. */    
            case 3U:
                *outRed   = fadeToGray;
                *outGreen = fadeBetweenColors1;
                *outBlue  = value;
                break;
            /* Blue to Magenta. */
            case 4U:
                *outRed   = fadeBetweenColors2;
                *outGreen = fadeToGray;
                *outBlue  = value;
                break;
            /* Magenta to Red. */
            case 5U:
                *outRed   = value;
                *outGreen = fadeToGray;
                *outBlue  = fadeBetweenColors1;
                break;
            default:
                *outRed   = value;
                *outGreen = value;
                *outBlue  = value;
                break;
        }   
    }
}

static void APP_AUDIO_UpdateWS2812(uint16_t adcResult)
{
    /* Calculate amplitude and brightness. */
    int16_t centeredValue = (int16_t)adcResult - (((int16_t)ADC_MAX_VALUE + 1) / 2);
    uint16_t amplitude = 0U;
    if (centeredValue == INT16_MIN)
    {
        amplitude = (uint16_t)INT16_MAX;
    }
    else
    {
        amplitude = (uint16_t)((centeredValue < 0) ? (uint16_t)(-centeredValue) : (uint16_t)centeredValue);
    }
    uint8_t minBrightness = 1U;
    uint8_t maxBrightness = maxBrightnessOptions[maxBrightnessIndex];
    uint8_t brightness = (uint8_t)((((uint32_t)amplitude * ((uint32_t)maxBrightness - (uint32_t)minBrightness)) / (((uint32_t)ADC_MAX_VALUE + 1U) / 2U)) + (uint32_t)minBrightness);
    /* Static hue for smooth color cycling. */
    static uint8_t hue = 0U;
    static uint8_t hueUpdateCounter = 0U;
    uint8_t hueInterval = hueIntervalOptions[hueIntervalIndex];
    hueUpdateCounter++;
    if (hueUpdateCounter >= hueInterval)
    {
        hue++;
        hueUpdateCounter = 0U;
    }
    /* Convert HSV to RGB */
    uint8_t r = 0U;
    uint8_t g = 0U;
    uint8_t b = 0U;
    APP_AUDIO_HSVtoRGB(hue, HSV_SATURATION_MAX, brightness, &r, &g, &b);
    /* Set all LEDs to the same color. */
    for (uint8_t i = 0U; i < NUM_WS2812_LEDS; i++) {
        WS2812DataBuffer[(i * 3U) + 0U] = g; // Green
        WS2812DataBuffer[(i * 3U) + 1U] = r; // Red
        WS2812DataBuffer[(i * 3U) + 2U] = b; // Blue
    }
    /* Start DMA transfer to WS2812 via SPI. */
    TCC1_PWMStart();
    if (DMAC_ChannelTransfer(DMAC_CHANNEL_6, WS2812DataBuffer, (const void *)&SERCOM1_REGS->SPIM.SERCOM_DATA, sizeof(WS2812DataBuffer)) == true)
    {
        /* No action required. */
    }
    else
    {
        /* No action required. */
    }
    dmaToSPITransferInProgress = true;
}

/* Reads a specified number of data bytes from the SD card. */
static bool APP_AUDIO_ReadSDCardData( 
    const DRV_HANDLE handle,
    uint8_t* const pBuffer,
    const uint16_t requestedBytes,
    uint16_t* const pNumBytesRead)
{    
    size_t nBytesRead = 0U;
    bool isSuccess = true;        
    /* Read data from SD card. */
    nBytesRead = SYS_FS_FileRead(handle, (void *)pBuffer, (size_t)requestedBytes);
    if (nBytesRead == 0xFFFFFFFFU)
    {                        
        if (SYS_FS_FileEOF(handle) == false)
        {
            /* Error on file read. */
            isSuccess = false;         
        }
        *pNumBytesRead = 0U;
    }                    
    else
    {
        *pNumBytesRead = (uint16_t)nBytesRead;
    }
    return isSuccess;
}

/* Writes a specified number of data bytes to the SD card. */
static bool APP_AUDIO_WriteSDCardData(
    const DRV_HANDLE handle,
    uint8_t* const pBuffer,
    const uint16_t requestedBytes,
    uint16_t* const pNumBytesWritten)
{
    size_t nBytesWritten = 0U;
    bool isSuccess = true;       
    /* Writes data to SD card. */
    nBytesWritten = SYS_FS_FileWrite(handle, (void *)pBuffer, (size_t)requestedBytes);
    if (nBytesWritten == 0xFFFFFFFFU)
    {
        /* Error on file write. */
        isSuccess = false;
        *pNumBytesWritten = 0U;
    }                   
    else
    {
        *pNumBytesWritten = (uint16_t)nBytesWritten;
    }
    return isSuccess;    
}

/* Decodes the data bytes read from the SD card. */
static uint16_t APP_AUDIO_DecodeSDCardData(
    const uint8_t* const pInBuffer,
    uint16_t* const pOutBuffer,
    const uint16_t nInputBytes,
    uint16_t* const pnBytesDecoded,
    APP_PCM_FORMAT pcmFormat)
{
    /* Copy input buffer to output buffer. */
    (void)memcpy((void *)pOutBuffer, (const void *)pInBuffer, (size_t)nInputBytes);
    /* If 16-bit signed PCM, convert to unsigned and scale. */
    if (pcmFormat == PCM_FORMAT_16BIT_SIGNED)
    {
        uint16_t nSamples = (uint16_t)(nInputBytes / 2U);
        for (uint16_t i = 0U; i < nSamples; i++)
        {
            pOutBuffer[i] = (uint16_t)(((uint16_t)pOutBuffer[i] + 32768U) >> 6U);
        }
    }
    *pnBytesDecoded = nInputBytes;
    return (uint16_t)(nInputBytes / 2U);
}

/* Encodes the data bytes written to the SD card. */
static uint16_t APP_AUDIO_EncodeSDCardData(
    const uint16_t* const pInBuffer, 
    uint8_t* const pOutBuffer,
    const uint16_t nInputBytes, 
    uint16_t* const pnBytesEncoded)
{
    /* Copy input buffer to output buffer. */
    (void)memcpy((void *)pOutBuffer, (const void *)pInBuffer, (size_t)nInputBytes);
    *pnBytesEncoded = nInputBytes;
    return (uint16_t)((uint32_t)nInputBytes * 2U);
}

/* DMA callback for playback buffer transfer completion. */
static void DMAC_TransmitCompleteCallback_CH0(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{    
    DBG1_Set();
    DBG1_Clear();
    
    /* Mark the buffer just played as EMPTY. */
    buffers[dmaBufferFillIndex]->state = BUFFER_EMPTY;
    bufferFullCount--;
    /* If first buffer was partially filled on first read, playback is done. */
    if (firstBufferPartiallyFilledOnFirstRead) 
    {
        TC0_TimerStop();
        TC5_TimerStop();
    }
    /* If file not ended, refill the buffer just played. */
    else 
    {
        if (!fileEnded) 
        {
            /* Switch DMA to the other buffer (ping-pong). */
            dmaBufferFillIndex = (uint8_t)(((uint32_t)dmaBufferFillIndex + 1U) % (uint32_t)NUM_AUDIO_BUFFERS_FOR_PLAYBACK);
            bufferProcessIndex = (uint8_t)(((uint32_t)dmaBufferFillIndex + 1U) % (uint32_t)NUM_AUDIO_BUFFERS_FOR_PLAYBACK);
            appSession.state = APP_AUDIO_STATE_DATA_READ;
        }
        /* If file ended, handle last buffer. */
        else 
        {
            if (!lastBufferProcessed) 
            {
                lastBufferProcessed = true;
            } 
            else
            {
                TC0_TimerStop();
                TC5_TimerStop();
            }
        }
    }
}

/* DMA callback for record buffer transfer completion. */
static void DMAC_TransmitCompleteCallback_CH1(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{    
    DBG1_Set();
    DBG1_Clear();
    
    /* Mark the buffer just filled as FULL. */
    buffers[dmaBufferFillIndex]->state = BUFFER_FULL;
    bufferFullCount++;
    /* Advance to next buffer for DMA fill. */
    uint8_t nextFill = (uint8_t)(((uint32_t)dmaBufferFillIndex + 1U) % (uint32_t)NUM_AUDIO_BUFFERS_FOR_RECORD);
    if (buffers[nextFill]->state == BUFFER_EMPTY)
    {
        /* DMA will start filling this buffer next. */
        buffers[nextFill]->state = BUFFER_FILLING;
        dmaBufferFillIndex = nextFill;
    } 
    else 
    {
        /* No empty buffer available: overrun. */
        bufferOverrun = true;
    }
    appSession.state = APP_AUDIO_STATE_DATA_WRITE;    
}

/* Timer callback for button debouncing. */
static void TC4_OverflowCallback(TC_TIMER_STATUS status, uintptr_t context)
{
    /* Debounce logic for each button. */
    static uint32_t debounceCounters[NUM_BUTTONS];
    static bool lastButtonReading[NUM_BUTTONS];
    static bool debouncedState[NUM_BUTTONS];
    static bool initialized = false;
    if (initialized == false)
    {
        for (uint8_t i = 0U; i < NUM_BUTTONS; i++)
        {
            debounceCounters[i] = 0U;
            lastButtonReading[i] = false;
            debouncedState[i] = false;
        }
        initialized = true;
    }
    for (uint8_t i = 0U; i < NUM_BUTTONS; i++)
    {
        bool currentReading = false;
        switch (i)
        {
            case 0: currentReading = (SW1_Get() == 0U); break;
            case 1: currentReading = (SW2_Get() == 0U); break;
            case 2: currentReading = (SW3_Get() == 0U); break;
            case 3: currentReading = (SW4_Get() == 0U); break;
            default: /* No action required. */ break;
        }
        if (currentReading != lastButtonReading[i])
        {
            debounceCounters[i] = 0U;
        }
        else
        {
            if (debounceCounters[i] < DEBOUNCE_DELAY_MS)
            {
                debounceCounters[i]++;
            }
            else if (debouncedState[i] != currentReading)
            {
                debouncedState[i] = currentReading;
                switch (i)
                {
                    case 0: button1Pressed = debouncedState[i]; break;
                    case 1: button2Pressed = debouncedState[i]; break;
                    case 2: button3Pressed = debouncedState[i]; break;
                    case 3: button4Pressed = debouncedState[i]; break;
                    default: /* No action required. */ break;
                }
            }
            else
            {
                /* No action required */
            }
        }
        lastButtonReading[i] = currentReading;
    }
    
    /* Touch button handling. */
    static uint8_t touch_counter = 0U;
    static bool lastTouchState = false;
    touch_counter++;
    /* Call touch process every CALL_INTERVAL ms. */
    if (touch_counter >= TOUCH_PROCESS_CALL_INTERVAL_MS)
    {
        touch_counter = 0U;
        touch_process();
        if (measurement_done_touch != 0U)
        {
            measurement_done_touch = 0u;
            uint8_t key_state = get_sensor_state(0U);
            bool currentTouchState = ((key_state & KEY_TOUCHED_MASK) != 0U);
            if (currentTouchState != lastTouchState)
            {
                touchButtonPressed = currentTouchState;
                lastTouchState = currentTouchState;
            }
        }
    }
}

/* Timer callback for RGB LED timeout. */
static void TC6_OverflowCallback(TC_TIMER_STATUS status, uintptr_t context)
{
    TC6_TimerStop();
    APP_AUDIO_SetRGBLED(false, false, true);
}

/* DMA callback for ADC0 result buffer transfer completion (playback VU / WS2812 and file seek). */
static void DMAC_TransmitCompleteCallback_CH5(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{
    /* If seeking not enabled, initialize lastSeekAdcValue*/
    if (!seekingEnabled)
    {
        lastSeekAdcResult = adcResults[SEEK_CHANNEL_INDEX];
        seekingEnabled = true;
    }
    /* Only seek if the value has changed significantly. */
    else
    {
        int32_t diff = (int32_t)adcResults[SEEK_CHANNEL_INDEX] - (int32_t)lastSeekAdcResult;
        uint32_t abs_diff;
        if (diff == (int32_t)INT32_MIN)
        {
            abs_diff = (uint32_t)INT32_MAX;
        }
        else
        {
            abs_diff = (uint32_t)((diff < 0) ? (uint32_t)(-diff) : (uint32_t)diff);
        }
        
        if (abs_diff > (uint32_t)SEEK_THRESHOLD)
        {
            seekPending = true;
            lastSeekAdcResult = adcResults[SEEK_CHANNEL_INDEX];
        }
    }
    /* Update VU meter with latest ADC result. */
    APP_AUDIO_UpdateVUMeter(adcResults[VU_CHANNEL_INDEX]);
    APP_AUDIO_UpdateWS2812(adcResults[VU_CHANNEL_INDEX]);
    /* Restart DMA transfer for next ADC results. */
    if (DMAC_ChannelTransfer(DMAC_CHANNEL_5, (const void*)&ADC0_REGS->ADC_RESULT, (void*)&adcResults[0], sizeof(adcResults)) == true)
    {
        /* No action required. */
    }
    else
    {
        /* No action required. */
    }
}

/* Timer callback for OLED filename scrolling and update. */
static void TC7_OverflowCallback(TC_TIMER_STATUS status, uintptr_t context)
{
    static uint8_t oledFilenameBuffer[MAX_DMA_I2C_DATA_BUFFER_LEN];
    static size_t oledFilenameBufferLen = 0U;
    if (!dmaTransferInProgress)
    {
        oledFilenameBufferLen = 0U;
        /* Build OLED buffer for filename display. */
        APP_AUDIO_OLEDBuildFileNameBuffer(oledFilenameBuffer, &oledFilenameBufferLen);
        if (oledFilenameBufferLen < (size_t)MAX_DMA_I2C_DATA_BUFFER_LEN) 
        {
            /* Start DMA transfer to OLED. */
            if (DMAC_ChannelTransfer(DMAC_CHANNEL_4, oledFilenameBuffer, (const void *)&SERCOM0_REGS->I2CM.SERCOM_DATA, oledFilenameBufferLen) == true)
            {
                /* No action required. */
            }
            else
            {
                /* No action required. */
            }
            APP_AUDIO_I2CStartDMATransfer(SSD1306_I2C_ADDRESS, oledFilenameBufferLen);
            dmaTransferInProgress = true;
            if ((playback && audioStarted && !playbackPaused && !timestampUpdatePending) ||
                (record && !recordPaused && !timestampUpdatePending))
            {
                overflowCount++;
                if (overflowCount >= 2U)
                {
                    timestampUpdatePending = true;
                }
            }
        }
        /* Scroll file name. */
        size_t len = strlen(currentFileName);
        size_t scrollLen = (len > (size_t)OLED_VISIBLE_CHARS) ? (len - (size_t)OLED_VISIBLE_CHARS + 1U) : 1U;
        scrollPos++;
        if (scrollPos >= scrollLen)
        {
            scrollPos = 0U;
        }
    }
}

/* DMA callback for OLED I2C transfer completion. */
static void DMAC_TransmitCompleteCallback_CH4(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{
    static uint8_t oledTimestampBuffer[MAX_DMA_I2C_DATA_BUFFER_LEN];
    static size_t oledTimestampBufferLen = 0U;
    dmaTransferInProgress = false;
    if (stopDisplayTimer)
    {
        appSession.state = APP_AUDIO_STATE_NEXT_FILE_PLAY;        
    }
    /* If timestamp update is pending, build and send the timestamp buffer. */
    if (timestampUpdatePending && !stopDisplayTimer) 
    {
        oledTimestampBufferLen = 0U;
        if (playback)
        {
            APP_AUDIO_OLEDBuildPlaybackTimestampBuffer(oledTimestampBuffer, &oledTimestampBufferLen, elapsedSeconds, totalSeconds);
        }
        else if (record)
        {
            APP_AUDIO_OLEDBuildRecordTimestampBuffer(oledTimestampBuffer, &oledTimestampBufferLen, elapsedSeconds);
        }
        else
        {
            /* No action required */
        }
        if (oledTimestampBufferLen < (size_t)MAX_DMA_I2C_DATA_BUFFER_LEN)
        {
            if (DMAC_ChannelTransfer(DMAC_CHANNEL_4, oledTimestampBuffer, (const void *)&SERCOM0_REGS->I2CM.SERCOM_DATA, oledTimestampBufferLen) == true)
            {
                /* No action required. */
            }
            else
            {
                /* No action required. */
            }
            APP_AUDIO_I2CStartDMATransfer(SSD1306_I2C_ADDRESS, oledTimestampBufferLen);
            dmaTransferInProgress = true;
        }
        overflowCount = 0U;
        timestampUpdatePending = false;
        elapsedSeconds++;
    }
    /* If playback finished, stop the display timer on next overflow interrupt. */
    if (playback && audioStarted && (elapsedSeconds >= (totalSeconds + 1U)))
    {
        stopDisplayTimer = true;
        TC7_TimerStop();
    }
}

/* ADC1 result ready callback for VU meter during recording. */
static void ADC1_ResultReadyCallback(ADC_STATUS status, uintptr_t context)
{
    /* Get latest microphone sample. */
    uint16_t adcResult = ADC1_ConversionResultGet();
    /* Result is a 10 bit value, scale it to 12 to match the value expected by the VU function*/
    adcResult = (uint16_t)(adcResult << 2U);
    APP_AUDIO_UpdateVUMeter(adcResult);
}

static void DMAC_TransmitCompleteCallback_CH6(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{
    dmaToSPITransferInProgress = false;
    WS2812OutputDisablePending = true;
}

static void TCC1_OverflowCallback(uint32_t status, uintptr_t context)
{
    if (WS2812OutputDisablePending)
    {
        TCC1_PWMStop();
        WS2812OutputDisablePending = false;
    }
}

/* Initializes the audio application state machine. */
void APP_AUDIO_Initialize(void)
{
    /* Set the initial state of the application. */
    appSession.state = APP_AUDIO_STATE_INIT;
}

/* Main application state machine task function. */
void APP_AUDIO_Tasks(void)
{
    static uint16_t numChannels = 0U;
    static uint32_t sampleRate = 0U;
    static uint16_t bytesPerSample = 0U;
    static uint16_t bitsPerSample = 0U;
    static uint32_t dataSize = 0U;
    static bool recordAndPlaybackFileFound = false;
    static uint8_t fileCount = 0U;
    static char fileNames[MAX_FILES][SYS_FS_FILE_NAME_LEN+1];
    static uint8_t selectedFileIndex = 0U;
    static uint32_t totalByteNumber = 0U;
    static uint8_t nMountAttempts = 0U;
    static uint8_t wavHeader[WAV_HEADER_SIZE];
    switch(appSession.state)
    {
        case APP_AUDIO_STATE_INIT:
        {
            /* Reset all buffer indices, flags and counters. */
            dmaBufferFillIndex = 0U;
            bufferProcessIndex = 0U;
            bufferFullCount = 0U;
            bufferOverrun = false;
            fileEnded = false;
            firstBufferPartiallyFilledOnFirstRead = false;
            lastBufferProcessed = false;
            audioStarted = false;
            nMountAttempts = 0U;
            appSession.fileSize = 0;
            appSession.currentFilePosition = 0;
            appSession.audioData.buffer1.data = audioDataBuffer1;
            appSession.audioData.buffer2.data = audioDataBuffer2;
            appSession.audioData.buffer3.data = audioDataBuffer3;
            appSession.audioData.buffer1.sampleNumber = 0U;
            appSession.audioData.buffer2.sampleNumber = 0U;
            appSession.audioData.buffer3.sampleNumber = 0U;
            appSession.audioData.buffer1.state = BUFFER_FILLING;
            appSession.audioData.buffer2.state = BUFFER_EMPTY;
            appSession.audioData.buffer3.state = BUFFER_EMPTY;
            appSession.wavData.byteNumber = 0U;
            TC0_Timer32bitCounterSet(0U);
            TC2_Timer32bitCounterSet(0U);
            DMAC_ChannelDisable(DMAC_CHANNEL_0);
            DMAC_ChannelDisable(DMAC_CHANNEL_1);
            DAC_Disable();
            record = false;
            playback = false;
            totalByteNumber = 0U;
            fileCount = 0U;
            selectedFileIndex = 0U;
            seekingEnabled = false;
            lastSeekAdcResult = 0U;
            seekPending = false;
            scrollPos = 0U;
            playbackPaused = false;
            recordPaused = false;
            recordAndPlaybackFileFound = false;
            sampleRate = 0U;
            numChannels = 0U;
            bitsPerSample = 0U;
            dataSize = 0U;
            totalSeconds = 0U;
            bytesPerSample = 0U;
            elapsedSeconds = 0U;
            overflowCount = 0U;
            timestampUpdatePending = false;
            stopDisplayTimer = false;
            dmaTransferInProgress = false;
            /* Initialize user interface features (RGB LED and OLED display). */
            APP_AUDIO_SetRGBLED(false, false, false);
            SSD1306_Init();
            SSD1306_Clear();
            SSD1306_DrawBitmap(MCHP);
            SSD1306_SetAddressingMode(0x02U);
            SSD1306_SetPage(4U);
            SSD1306_SetColumn(0U);
            SSD1306_WriteString5x8("Audio Player&Recorder");
            SSD1306_SetPage(5U);
            SSD1306_SetColumn(0U);
            SSD1306_WriteString5x8(" PIC32CM5164JH01048  ");
            SSD1306_DrawBitmapCustom(26U, 56U, 8U, 8U, RecordSymbol);
            SSD1306_DrawBitmapCustom(60U, 56U, 8U, 8U, PlaySymbol);
            APP_AUDIO_IOExpanderWriteRegister(IODIR_REG_ADDRESS, 0xFFU);
            APP_AUDIO_IOExpanderWriteRegister(GPIO_REG_ADDRESS, 0x00U);
            TC4_TimerCallbackRegister(&TC4_OverflowCallback, 0U);
            TC6_TimerCallbackRegister(&TC6_OverflowCallback, 0U);
            TC7_TimerCallbackRegister(&TC7_OverflowCallback, 0U);
            TC4_TimerStart();
            LED7_Clear();
            LED6_Clear();
            LED5_Clear();
            LED4_Clear();
            LED3_Clear();
            LED2_Clear();
            LED1_Clear();
            LED0_Clear();
            for (uint8_t i = 0U; i < (uint8_t)(NUM_WS2812_LEDS * WS2812_BYTES_PER_LED); i++)
            {
                WS2812DataBuffer[i] = 0x01U;
            }
            DMAC_ChannelCallbackRegister(DMAC_CHANNEL_6, &DMAC_TransmitCompleteCallback_CH6, 0U);
            TCC1_PWMCallbackRegister(&TCC1_OverflowCallback, 0U);
            TCC1_PWMStart();
            while (dmaToSPITransferInProgress)
            {
                ;
            }
            if (DMAC_ChannelTransfer(DMAC_CHANNEL_6, WS2812DataBuffer, (const void *)&SERCOM1_REGS->SPIM.SERCOM_DATA, sizeof(WS2812DataBuffer)) == true)
            {
                /* No action required. */
            }
            else
            {
                /* No action required. */
            }
            dmaToSPITransferInProgress = true;
            appSession.state = APP_AUDIO_STATE_IDLE;
            break;
        }
        
        case APP_AUDIO_STATE_IDLE:
        {
            /* Set RGB LED to white to indicate idle state. */
            APP_AUDIO_SetRGBLED(true, true, true);
            /* Application is running in IDLE state until a button press is detected. */
            if (button2Pressed)
            {
                /* Button 2 press starts the audio playback sequence. */
                playback = true;
                button2Pressed = false;
                appSession.state = APP_AUDIO_STATE_MOUNT;
            }
            else if (button1Pressed)
            {
                /* Button 1 press starts the audio recording sequence. */
                record = true;
                button1Pressed = false;
                appSession.state = APP_AUDIO_STATE_MOUNT;
            }
            else
            {
                /* No action required */
            }
            break;
        }
        
        case APP_AUDIO_STATE_MOUNT:
        {
            /* Mount SD card. */
            if (SYS_FS_Mount("/dev/mmcblka1", "/mnt/MYDRIVE", FAT, 0U, NULL) != SYS_FS_RES_SUCCESS)
            {
                /* The disk could not be mounted. Try mounting again until success. */
                nMountAttempts++;
                if (nMountAttempts <= (uint8_t)MAX_MOUNT_ATTEMPTS)
                {
                    appSession.state = APP_AUDIO_STATE_MOUNT;
                }
                else
                {
                    /* Error while mounting. */
                    appSession.state = APP_AUDIO_STATE_ERROR;
                }
            }
            else
            {
                /* Mount was successful. */
                appSession.state = APP_AUDIO_STATE_CURRENT_DRIVE_SET;
            }
            break;
        } 
        
        case APP_AUDIO_STATE_CURRENT_DRIVE_SET:
        {
            /* Set current drive. */
            if (SYS_FS_CurrentDriveSet("/mnt/MYDRIVE") == SYS_FS_RES_FAILURE)
            {
                /* Error while setting current drive. */
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
            else
            {
                /* Drive set was successful. */
                appSession.state = APP_AUDIO_STATE_FILES_NAME_GET;
            }
            break;
        }
        
        case APP_AUDIO_STATE_FILES_NAME_GET:
        {
            /* Open directory and read file names. */
            SYS_FS_HANDLE dirHandle;
            SYS_FS_FSTAT stat = {0U};
            stat.lfname = NULL;
            dirHandle = SYS_FS_DirOpen("/mnt/MYDRIVE/");
            if (dirHandle != SYS_FS_HANDLE_INVALID)
            {
                /* Read all files (not directories) into fileNames buffer. */
                while ((SYS_FS_DirRead(dirHandle, &stat) == SYS_FS_RES_SUCCESS) && (stat.fname[0] != '\0') && (fileCount < (uint8_t)(MAX_FILES - 1U)))
                {
                    /* Check if entry is a file. */
                    if ((stat.fattrib & (uint8_t)SYS_FS_ATTR_DIR) == 0U)
                    {
                        (void)strcpy(fileNames[fileCount], stat.fname);
                        fileCount++;
                    }
                }
                if (SYS_FS_DirClose(dirHandle) == SYS_FS_RES_SUCCESS)
                {
                    /* Set the file index to the record/playback file. */
                    for (uint8_t i = 0U; i < fileCount; i++)
                    {
                        if (strcmp(fileNames[i], FILE_NAME_RECORD_AND_PLAYBACK) == 0)
                        {
                            recordAndPlaybackFileFound = true;
                            selectedFileIndex = i;
                            break;
                        }
                    }
                    /* If record mode was selected, display specific menu options. */
                    if (record)
                    {
                        if (!recordAndPlaybackFileFound)
                        {
                            /* Create and open file for write. */
                            appSession.fileHandle = SYS_FS_FileOpen(FILE_NAME_RECORD_AND_PLAYBACK, SYS_FS_FILE_OPEN_WRITE_PLUS);
                            if (appSession.fileHandle != SYS_FS_HANDLE_INVALID)
                            {
                                uint16_t bytesWritten = 0U;
                                uint8_t defaultWavHeader[WAV_HEADER_SIZE] = {
                                    0x52U, 0x49U, 0x46U, 0x46U,       /* RIFF chunk identifier */
                                    0x24U, 0x00U, 0x00U, 0x00U,       /* Size of the chunk (36 bytes) */
                                    0x57U, 0x41U, 0x56U, 0x45U,       /* WAVE format identifier */
                                    0x66U, 0x6DU, 0x74U, 0x20U,       /*"fmt " subchunk identifier */
                                    0x10U, 0x00U, 0x00U, 0x00U,       /* Size of "fmt " subchunk (16 bytes) */
                                    0x01U, 0x00U,                     /* PCM audio format */
                                    0x01U, 0x00U,                     /* Mono channel */
                                    0x80U, 0xBBU, 0x00U, 0x00U,       /* Sample rate: 48000 Hz */
                                    0x00U, 0x77U, 0x01U, 0x00U,       /* Byte rate: 96000 bytes/sec */
                                    0x02U, 0x00U,                     /* Block align: 2 bytes */
                                    0x10U, 0x00U,                     /* Bits per sample: 16 */
                                    0x64U, 0x61U, 0x74U, 0x61U,       /* "data" subchunk identifier */
                                    0x00U, 0x00U, 0x00U, 0x00U        /* Size of "data" subchunk: 0 bytes */
                                };
                                if (true == APP_AUDIO_WriteSDCardData(
                                        appSession.fileHandle,
                                        defaultWavHeader,
                                        WAV_HEADER_SIZE,
                                        &bytesWritten))
                                {
                                    recordAndPlaybackFileFound = true;
                                    if (SYS_FS_FileClose(appSession.fileHandle) == SYS_FS_RES_SUCCESS)
                                    {
                                        (void)strcpy(fileNames[fileCount], FILE_NAME_RECORD_AND_PLAYBACK);
                                        selectedFileIndex = fileCount;
                                        fileCount++;
                                    }
                                    else
                                    {
                                        /* Error on file close. */
                                        appSession.state = APP_AUDIO_STATE_ERROR;
                                    }
                                }
                                else
                                {
                                    /* Error on file write. */
                                    appSession.state = APP_AUDIO_STATE_ERROR;
                                }
                            }
                            else
                            {
                                /* Error on file open. */
                                appSession.state = APP_AUDIO_STATE_ERROR;
                                break;
                            }                        
                        }
                        (void)strcpy(appSession.fileName, fileNames[selectedFileIndex]);
                        SSD1306_Init();
                        SSD1306_Clear();
                        SSD1306_DrawBitmapCustom(26U, 56U, 8U, 8U, RecordSymbol);
                        SSD1306_DrawBitmapCustom(60U, 56U, 8U, 8U, PauseSymbol);
                        SSD1306_DrawBitmapCustom(94U, 56U, 8U, 8U, StopSymbol);
                        appSession.state = APP_AUDIO_STATE_FILE_OPEN;             
                    }
                    /* If playback mode was selected, display specific menu options. */
                    if (playback)
                    {
                        /* Set RGB LED to blue to indicate file selection menu state. */
                        APP_AUDIO_SetRGBLED(false, false, true);
                        /* Update OLED to display menu selection options. */
                        SSD1306_Init();
                        SSD1306_Clear();
                        SSD1306_DrawBitmapCustom(26U, 56U, 8U, 8U, PreviousSymbol);
                        SSD1306_DrawBitmapCustom(60U, 56U, 8U, 8U, PlaySymbol);
                        SSD1306_DrawBitmapCustom(94U, 56U, 8U, 8U, NextSymbol);  
                        appSession.state = APP_AUDIO_STATE_FILE_NAME_SELECT;
                    }
                    /* Setup DMA for I2C transfers. */
                    DMAC_ChannelCallbackRegister(DMAC_CHANNEL_4, &DMAC_TransmitCompleteCallback_CH4, 0U);
                    (void)strncpy(currentFileName, fileNames[selectedFileIndex], SYS_FS_FILE_NAME_LEN);
                    currentFileName[SYS_FS_FILE_NAME_LEN] = '\0';
                    /* Start file name scrolling. */
                    TC7_TimerStart();
                }
                else
                {
                    /* Error on directory close. */
                    appSession.state = APP_AUDIO_STATE_ERROR;
                }
            }
            else
            {
                /* Error on directory open. */
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
            break;
        }
        
        case APP_AUDIO_STATE_FILE_NAME_SELECT:
        {
            /* If button 2 was pressed, select, open and play file. */
            if (button2Pressed)
            {
                button2Pressed = false;
                (void)strcpy(appSession.fileName, fileNames[selectedFileIndex]);
                if (strcmp(appSession.fileName, FILE_NAME_RECORD_AND_PLAYBACK) != 0)
                {
                    appSession.pcmFormat = PCM_FORMAT_16BIT_SIGNED;
                }
                else
                {
                    appSession.pcmFormat = PCM_FORMAT_10BIT_UNSIGNED;
                }
                TC7_TimerStop();
                while (dmaTransferInProgress)
                {
                    ;
                }
                SSD1306_DrawBitmapCustom(26U, 56U, 8U, 8U, ListSymbol);
                SSD1306_DrawBitmapCustom(60U, 56U, 8U, 8U, PlayPauseSymbol);
                SSD1306_DrawBitmapCustom(94U, 56U, 8U, 8U, StopSymbol);
                TC7_TimerStart();
                appSession.state = APP_AUDIO_STATE_FILE_OPEN;
            }
            /* If button 3 was pressed, increment file index (next file). */
            if (button3Pressed)
            {
                button3Pressed = false;
                selectedFileIndex++;
                if (selectedFileIndex >= fileCount)
                {
                    /* Wrap around to first file. */
                    selectedFileIndex = 0U;
                }
                /* Blink RGB LED to magenta to indicate a registered button press. */
                APP_AUDIO_SetRGBLED(true, false, true);
                TC6_TimerStart();
                (void)strncpy(currentFileName, fileNames[selectedFileIndex], SYS_FS_FILE_NAME_LEN);
                currentFileName[SYS_FS_FILE_NAME_LEN] = '\0';
                scrollPos = 0U;
                TC7_TimerStart();

            }
            /* If button 1 was pressed, decrement file index (previous file). */
            if (button1Pressed)
            {
                button1Pressed = false;
                if (selectedFileIndex == 0U)
                {
                    /* Wrap around to last file. */
                    selectedFileIndex = (uint8_t)(fileCount - 1U);
                }
                else
                {
                    selectedFileIndex--;
                }
                /* Blink RGB LED to cyan to indicate a registered button press. */
                APP_AUDIO_SetRGBLED(false, true, true);
                TC6_TimerStart();
                (void)strncpy(currentFileName, fileNames[selectedFileIndex], SYS_FS_FILE_NAME_LEN);
                currentFileName[SYS_FS_FILE_NAME_LEN] = '\0';
                scrollPos = 0U;
                TC7_TimerStart();
            }
            break;
        }
         
        case APP_AUDIO_STATE_FILE_OPEN:
        {
            /* Open file for read/write. */
            appSession.fileHandle = SYS_FS_FileOpen(appSession.fileName, (SYS_FS_FILE_OPEN_READ_PLUS));        
            if(appSession.fileHandle == SYS_FS_HANDLE_INVALID)
            {
                /* Error on file opening. */
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
            else
            {
                /* Read the file size. */
                appSession.state = APP_AUDIO_STATE_FILE_SIZE_READ;
            }
            break;
        }
            
        case APP_AUDIO_STATE_FILE_SIZE_READ:
        {
            /* Read the file size (number of bytes). */
            appSession.fileSize = SYS_FS_FileSize(appSession.fileHandle);
            if (appSession.fileSize == -1)
            {
                /* Error on file size read. */
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
            else
            {
                /* Read the WAV file header section. */
                appSession.state = APP_AUDIO_STATE_WAV_HEADER_READ;
            }
            break;
        }    
        
        case APP_AUDIO_STATE_WAV_HEADER_READ:
        {
            uint16_t nWavHeaderBytesRead = 0U;
            /* Read the WAV header file section. */
            if (true == APP_AUDIO_ReadSDCardData(appSession.fileHandle,
                    wavHeader, (uint16_t)WAV_HEADER_SIZE, &nWavHeaderBytesRead))
            {
                /* Parse WAV header file sections. */
                (void)memcpy((void*)&numChannels, (const void*)&wavHeader[WAV_HEADER_NUM_CHANNELS_INDEX], sizeof(numChannels));
                (void)memcpy((void*)&sampleRate, (const void*)&wavHeader[WAV_HEADER_FREQUENCY_INDEX], sizeof(sampleRate));
                (void)memcpy((void*)&bytesPerSample, (const void*)&wavHeader[WAV_HEADER_BYTES_PER_SAMPLE_INDEX], sizeof(bytesPerSample));
                (void)memcpy((void*)&bitsPerSample, (const void*)&wavHeader[WAV_HEADER_BITS_PER_SAMPLE_INDEX], sizeof(bitsPerSample));
                (void)memcpy((void*)&dataSize, (const void*)&wavHeader[WAV_HEADER_DATA_SIZE_INDEX], sizeof(dataSize));
                if ((sampleRate != 0U) && (bytesPerSample != 0U))
                {
                    totalSeconds = dataSize / ((uint32_t)sampleRate * (uint32_t)bytesPerSample);
                }
                else
                {
                    appSession.state = APP_AUDIO_STATE_ERROR;
                    break;
                }
                /* Update current file position to point to the data section. */
                if (nWavHeaderBytesRead == (uint16_t)WAV_HEADER_SIZE)
                {
                    appSession.currentFilePosition += (int32_t)WAV_HEADER_SIZE;
                    if (playback == true)
                    {
                        if (appSession.fileSize <= (int32_t)SDCARD_FILE_ALLOCATION_SIZE)
                        {
                            /* If file size is smaller than the file allocation size, issue error. */
                            appSession.state = APP_AUDIO_STATE_ERROR;
                        }
                        else
                        {
                            /* An initial dummy read of more than the SDCARD_FILE_ALLOCATION_SIZE
                               number of bytes is required to maintain playback synchronization.
                               This is necessary as the first read sequence that crosses the 
                               SDCARD_FILE_ALLOCATION_SIZE border takes longer than usual. */
                            appSession.state = APP_AUDIO_STATE_INITIAL_DUMMY_READ;                           
                        }
                    }
                    else if (record == true)
                    {
                        /* Pad with 0s until the end of the cluster. */
                        appSession.state = APP_AUDIO_STATE_DATA_PAD_WRITE;
                    }
                    else
                    {
                        /* No action required */
                    }
                }
                else
                {
                    /* Error on file read. */
                    appSession.state = APP_AUDIO_STATE_ERROR;
                }
            }
            else
            {
                /* Error on file read. */
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
            break;
        }
        
        case APP_AUDIO_STATE_INITIAL_DUMMY_READ:
        {
            /* Perform dummy reads to align to SD card cluster size. */
            uint16_t nDummyBytesRead = 0U;
            uint16_t nTotalDummyBytes = (uint16_t)(SDCARD_FILE_ALLOCATION_SIZE - WAV_HEADER_SIZE);
            uint16_t nFullDummyBufferReads = (uint16_t)(nTotalDummyBytes / MAX_SD_CARD_BUFFER_SIZE_FOR_PLAYBACK);
            uint16_t nRemainingDummyBytes = (uint16_t)(nTotalDummyBytes % MAX_SD_CARD_BUFFER_SIZE_FOR_PLAYBACK);
            for (uint16_t i = 0U; i < nFullDummyBufferReads; i++)
            {
                if (true == APP_AUDIO_ReadSDCardData(
                        appSession.fileHandle,
                        appSession.wavData.dataBufferPlayback,
                        MAX_SD_CARD_BUFFER_SIZE_FOR_PLAYBACK,
                        &nDummyBytesRead))
                {
                    appSession.currentFilePosition += (int32_t)nDummyBytesRead;
                }
                else
                {
                    appSession.state = APP_AUDIO_STATE_ERROR;
                }
            }
            /* Read two extra data bytes to advance the file pointer beyond the cluster file boundary.
               Reading from the 2nd audio sample (new cluster boundary + 2 bytes) offers significantly reduced SD card read time. */
            if (true == APP_AUDIO_ReadSDCardData(
                    appSession.fileHandle,
                    appSession.wavData.dataBufferPlayback,
                    (uint16_t)(nRemainingDummyBytes + 2U),
                    &nDummyBytesRead))
            {
                appSession.currentFilePosition += (int32_t)nDummyBytesRead;
                appSession.state = APP_AUDIO_STATE_DATA_READ;
            }
            else
            {
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
            break;
        }
        
        case APP_AUDIO_STATE_DATA_PAD_WRITE:
        {
            /* Pad the file with zeros to align to SD card cluster size. */
            uint16_t nPadBytesWritten = 0U;
            uint16_t nTotalPadBytes = (uint16_t)(SDCARD_FILE_ALLOCATION_SIZE - WAV_HEADER_SIZE);
            uint16_t nFullBufferWrites = (uint16_t)(nTotalPadBytes / MAX_SD_CARD_BUFFER_SIZE_FOR_RECORD);
            uint16_t nRemainingPadBytes = (uint16_t)(nTotalPadBytes % MAX_SD_CARD_BUFFER_SIZE_FOR_RECORD);
            for (uint16_t j = 0U; j < (uint16_t)MAX_SD_CARD_BUFFER_SIZE_FOR_RECORD; j++)
            {
                appSession.wavData.dataBufferRecord[j] = 0U;
            }       
            for (uint16_t i = 0U; i < nFullBufferWrites; i++)
            {
                if (true == APP_AUDIO_WriteSDCardData(
                        appSession.fileHandle,
                        appSession.wavData.dataBufferRecord,
                        (uint16_t)MAX_SD_CARD_BUFFER_SIZE_FOR_RECORD,
                        &nPadBytesWritten))
                {
                    totalByteNumber += nPadBytesWritten;
                    appSession.currentFilePosition += (int32_t)nPadBytesWritten;
                }
                else
                {
                    /* Error on file write. */
                    appSession.state = APP_AUDIO_STATE_ERROR;
                }
            }
            if (true == APP_AUDIO_WriteSDCardData(
                    appSession.fileHandle,
                    appSession.wavData.dataBufferRecord,
                    nRemainingPadBytes,
                    &nPadBytesWritten))
            {
                totalByteNumber += nPadBytesWritten;
                appSession.currentFilePosition += (int32_t)nPadBytesWritten;
                /* Start recording microphone data. */
                appSession.state = APP_AUDIO_STATE_MIC_RECORD;
            }
            else
            {
                /* Error on file write. */
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
            break;
        }
        
        case APP_AUDIO_STATE_WAV_HEADER_WRITE:
        {
            /* Update the WAV file and data sizes. */
            uint32_t updatedFileSize = totalByteNumber + ((uint32_t)WAV_HEADER_SIZE - 8U);
            uint32_t updatedDataSize = totalByteNumber;
            uint16_t nWAVHeaderBytesWritten = 0U;
            (void)memcpy((void*)&wavHeader[WAV_HEADER_FILE_SIZE_INDEX], (const void*)&updatedFileSize, sizeof(uint32_t));
            (void)memcpy((void*)&wavHeader[WAV_HEADER_DATA_SIZE_INDEX], (const void*)&updatedDataSize, sizeof(uint32_t));
            /* Set pointer at the start of the file. */
            if (SYS_FS_FileSeek(appSession.fileHandle, 0, SYS_FS_SEEK_SET) != -1)
            {
                /* Write the new WAV header to the file. */
                if (true == APP_AUDIO_WriteSDCardData(
                        appSession.fileHandle,
                        wavHeader,
                        (uint16_t)WAV_HEADER_SIZE,
                        &nWAVHeaderBytesWritten))
                {
                    appSession.state = APP_AUDIO_STATE_UNMOUNT;
                }
                else
                {
                    /* Error on file write. */
                    appSession.state = APP_AUDIO_STATE_ERROR;
                }
            }
            else
            {
                /* Error on file seek. */
                appSession.state = APP_AUDIO_STATE_ERROR;                
            }
            break;
        }
              
        case APP_AUDIO_STATE_DATA_READ:
        {
            DBG2_Set();
            uint16_t nBytesRead = 0U;
            uint16_t nBytesDecoded = 0U;
            /* Read data from the SD card into buffers. */
            APP_AUDIO_BUFFER* bufToFill = buffers[bufferProcessIndex];
            if (appSession.currentFilePosition < appSession.fileSize)
            {
                if (APP_AUDIO_ReadSDCardData(
                        appSession.fileHandle,
                        appSession.wavData.dataBufferPlayback,
                        MAX_SD_CARD_BUFFER_SIZE_FOR_PLAYBACK,
                        &nBytesRead))
                {
                    appSession.wavData.byteNumber += nBytesRead;
                    appSession.currentFilePosition += (int32_t)nBytesRead;
                    bufToFill->sampleNumber =
                    APP_AUDIO_DecodeSDCardData(
                        appSession.wavData.dataBufferPlayback,
                        bufToFill->data,
                        nBytesRead,
                        &nBytesDecoded,
                        appSession.pcmFormat);
                    bufToFill->state = BUFFER_FULL;
                    bufferFullCount++;
                    /* Update linked list DMA descriptors for playback. */
                    pTxLinkedListDesc[bufferProcessIndex].DMAC_BTCNT = bufToFill->sampleNumber;
                    pTxLinkedListDesc[bufferProcessIndex].DMAC_SRCADDR = (uint32_t)bufToFill->data + ((uint32_t)bufToFill->sampleNumber * (uint32_t)sizeof(uint16_t));
                    /* If less than a full buffer was read, we've reached the end of the file. */
                    if (nBytesRead != (uint16_t)MAX_SD_CARD_BUFFER_SIZE_FOR_PLAYBACK) 
                    {
                        fileEnded = true;
                        /* If this is the first buffer and playback hasn't started, mark partial. */
                        if (!audioStarted && (bufferProcessIndex == 0U)) 
                        {
                            firstBufferPartiallyFilledOnFirstRead = true;
                        }
                    }
                    bufferProcessIndex = (uint8_t)(((uint32_t)bufferProcessIndex + 1U) % (uint32_t)NUM_AUDIO_BUFFERS_FOR_PLAYBACK);
                    /* If both buffers are full or first buffer is partial, start playback. */
                    if (!audioStarted && ((bufferFullCount == (uint8_t)NUM_AUDIO_BUFFERS_FOR_PLAYBACK) || firstBufferPartiallyFilledOnFirstRead))
                    {
                        appSession.state = APP_AUDIO_STATE_AUDIO_PLAY;
                    }
                    else if (audioStarted)
                    {
                        /* Wait for DMA callback to trigger next read. */
                        appSession.state = APP_AUDIO_STATE_WAIT;
                    }
                    else
                    {
                        /* No action required */    
                    }
                }
                else
                {
                    appSession.state = APP_AUDIO_STATE_ERROR;
                }
            }
            else
            {
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
            DBG2_Clear();
            break;
        }     
        
        case APP_AUDIO_STATE_AUDIO_PLAY:
        {
            /* Initialize the descriptor linked list. */
            APP_AUDIO_InitializeTxLinkedListDescriptorPlayback();
            /* Setup DMA transfer for audio data. */
            if (DMAC_ChannelLinkedListTransfer(DMAC_CHANNEL_0, &pTxLinkedListDesc[0]) == true)
            {
                /* No action required. */
            }
            else
            {
                /* No action required */
            }
            DMAC_ChannelCallbackRegister(DMAC_CHANNEL_0, &DMAC_TransmitCompleteCallback_CH0, 0U);
            /* Enable DAC. */
            DAC_Enable();	
            /* Start the DMA transfer for audio data. */
            TC0_TimerStart();
            /* Setup DMA transfer for VU and file seek update. */
            DMAC_ChannelCallbackRegister(DMAC_CHANNEL_5, &DMAC_TransmitCompleteCallback_CH5, 0U);
            if (DMAC_ChannelTransfer(DMAC_CHANNEL_5, (const void*)&ADC0_REGS->ADC_RESULT, (void*)&adcResults[0], sizeof(adcResults)) == true)
            {
                /* No action required. */
            }
            else
            {
                /* No action required */
            }
            /* Enable ADC0. */
            ADC0_Enable();
            /* Start ADC conversions on VU and SEEK channels. */
            TC5_TimerStart();
            /* Set RGB LED to green indicate playback state. */
            APP_AUDIO_SetRGBLED(false, true, false);
            audioStarted = true;
            if ((sampleRate != 0U) && (bytesPerSample != 0U))
            {
                int32_t dataPos = appSession.currentFilePosition - ((int32_t)SDCARD_FILE_ALLOCATION_SIZE + 2);
                uint32_t denom = (uint32_t)sampleRate * (uint32_t)bytesPerSample;
                if (dataPos > 0)
                {
                    elapsedSeconds = (uint32_t)dataPos / denom;
                }
                else
                {
                    elapsedSeconds = 0U;
                }
            }
            else
            {
                elapsedSeconds = 0U;
            }
            timestampUpdatePending = true;
            overflowCount = 0U;
            /* Wait for DMA CH0 Callback to update application state. */
            appSession.state = APP_AUDIO_STATE_WAIT;
            break;
        }
                  
        case APP_AUDIO_STATE_WAIT:
        {
            static const uint8_t hueIntervalOptionsCount = sizeof(hueIntervalOptions) / sizeof(hueIntervalOptions[0]);
            static const uint8_t maxBrightnessOptionsCount = sizeof(maxBrightnessOptions) / sizeof(maxBrightnessOptions[0]);
            /* Button 1 press resumes the recording  or stops the song and returns to file name menu in playback mode. */
            if (button1Pressed)
            {
                button1Pressed = false;
                if (playback) 
                {
                    TC7_TimerStop();
                    if (SYS_FS_FileClose(appSession.fileHandle) == SYS_FS_RES_SUCCESS)
                    {
                        TC0_TimerStop();
                        DMAC_ChannelDisable(DMAC_CHANNEL_0);
                        TC0_Timer32bitCounterSet(0U);
                        TC5_TimerStop();
                        appSession.currentFilePosition = 0;
                        dmaBufferFillIndex = 0U;
                        bufferProcessIndex = 0U;
                        bufferFullCount = 0U;
                        bufferOverrun = false;
                        fileEnded = false;
                        firstBufferPartiallyFilledOnFirstRead = false;
                        lastBufferProcessed = false;
                        audioStarted = false;
                        appSession.audioData.buffer1.state = BUFFER_FILLING;
                        appSession.audioData.buffer2.state = BUFFER_EMPTY;
                        appSession.audioData.buffer3.state = BUFFER_EMPTY;
                        seekingEnabled = false;
                        lastSeekAdcResult = 0U;
                        seekPending = false;
                        stopDisplayTimer = false;
                        elapsedSeconds = 0U;
                        overflowCount = 0U;
                        timestampUpdatePending = false;
                        scrollPos = 0U;
                        playbackPaused = false;
                        LED7_Clear();
                        LED6_Clear();
                        LED5_Clear();
                        LED4_Clear();
                        LED3_Clear();
                        LED2_Clear();
                        LED1_Clear();
                        LED0_Clear();
                        /* Set RGB LED to blue to indicate file selection menu state. */
                        APP_AUDIO_SetRGBLED(false, false, true);
                        /* Update OLED to display menu selection options. */
                        while (dmaTransferInProgress)
                        {
                            ;
                        }
                        SSD1306_Init();
                        SSD1306_Clear();
                        SSD1306_DrawBitmapCustom(26U, 56U, 8U, 8U, PreviousSymbol);
                        SSD1306_DrawBitmapCustom(60U, 56U, 8U, 8U, PlaySymbol);
                        SSD1306_DrawBitmapCustom(94U, 56U, 8U, 8U, NextSymbol);
                        TC7_TimerStart();
                        appSession.state = APP_AUDIO_STATE_FILE_NAME_SELECT;
                    }
                    else
                    {
                        /* Error on file close. */
                        appSession.state = APP_AUDIO_STATE_ERROR;
                    }
                }
                if (recordPaused)
                {
                    /* Resume recording. */
                    recordPaused = false;
                    TC2_TimerStart();
                    TC7_TimerStart();
                    /* Set RGB LED to red to indicate recording state. */
                    APP_AUDIO_SetRGBLED(true, false, false);    
                }
            }
            /* Button 2 press toggles between pause and resume in playback mode or pauses recording in record mode. */
            if (button2Pressed)
            {
                button2Pressed = false;
                if (playback)
                {
                    if (!playbackPaused)
                    {
                        /* Pause playback. */
                        playbackPaused = true;
                        TC0_TimerStop();
                        TC5_TimerStop();
                        /* Set RGB LED to yellow to indicate pause state. */
                        APP_AUDIO_SetRGBLED(true, true, false);
                    }
                    else
                    {
                        /* Resume playback. */
                        playbackPaused = false;
                        seekPending = false;
                        seekingEnabled = false;
                        TC0_TimerStart();
                        TC5_TimerStart();
                        /* Set RGB LED to green to indicate playback state. */
                        APP_AUDIO_SetRGBLED(false, true, false);
                    }              
                }
                else if (record)
                {
                    /* Pause recording. */
                    if (!recordPaused)
                    {
                        recordPaused = true;
                        TC2_TimerStop();
                        /* Set RGB LED to yellow to indicate pause state. */
                        APP_AUDIO_SetRGBLED(true, true, false);
                    }
                }
                else
                {
                    
                }
            }
             /* Button 3 press terminates the ongoing action (playback or recording). */
            if (button3Pressed)
            {
                button3Pressed = false;
                if (playback)
                {
                    TC0_TimerStop();
                    TC5_TimerStop();
                    TC7_TimerStop();
                    appSession.state = APP_AUDIO_STATE_UNMOUNT;
                }
                else if (record)
                {
                    TC2_TimerStop();
                    TC7_TimerStop();
                    appSession.state = APP_AUDIO_STATE_FILE_TRUNCATE;
                }
                else
                {
                    /* No action required */
                }
            }
            if (button4Pressed)
            {
                button4Pressed = false;
                if (playback) 
                {
                    if (!playbackPaused)
                    {
                        maxBrightnessIndex++;
                        if (maxBrightnessIndex >= maxBrightnessOptionsCount)
                        {
                            maxBrightnessIndex = 0U;      
                        }
                    }
                }
            }
            if (touchButtonPressed)
            {
                touchButtonPressed = false;
                if (playback) 
                {
                    if (!playbackPaused)
                    {
                        hueIntervalIndex++;
                        if (hueIntervalIndex >= hueIntervalOptionsCount)
                        {
                            hueIntervalIndex = 0U;
                        }
                    }
                }
            }
 
            if (seekPending)
            {
                /* If file seeking is required, stop playback and reset hardware. */
                TC0_TimerStop();
                DMAC_ChannelDisable(DMAC_CHANNEL_0);
                TC0_Timer32bitCounterSet(0U);
                TC5_TimerStop();
                APP_AUDIO_SeekAudioFile(lastSeekAdcResult);
                /* Reset buffer flags to force refill. */
                dmaBufferFillIndex = 0U;
                bufferProcessIndex = 0U;
                bufferFullCount = 0U;
                bufferOverrun = false;
                fileEnded = false;
                firstBufferPartiallyFilledOnFirstRead = false;
                lastBufferProcessed = false;
                audioStarted = false;
                appSession.audioData.buffer1.state = BUFFER_FILLING;
                appSession.audioData.buffer2.state = BUFFER_EMPTY;
                appSession.audioData.buffer3.state = BUFFER_EMPTY;
                seekingEnabled = false;
                lastSeekAdcResult = 0U;
                seekPending = false;
                appSession.state = APP_AUDIO_STATE_DATA_READ;
            }
            break;
        }
        
        case APP_AUDIO_STATE_NEXT_FILE_PLAY:
        {
            if (SYS_FS_FileClose(appSession.fileHandle) == SYS_FS_RES_SUCCESS)
            {
                TC0_TimerStop();
                DMAC_ChannelDisable(DMAC_CHANNEL_0);
                TC0_Timer32bitCounterSet(0U);
                TC5_TimerStop();
                appSession.currentFilePosition = 0;
                dmaBufferFillIndex = 0U;
                bufferProcessIndex = 0U;
                bufferFullCount = 0U;
                bufferOverrun = false;
                fileEnded = false;
                firstBufferPartiallyFilledOnFirstRead = false;
                lastBufferProcessed = false;
                audioStarted = false;
                scrollPos = 0U;
                appSession.audioData.buffer1.state = BUFFER_FILLING;
                appSession.audioData.buffer2.state = BUFFER_EMPTY;
                appSession.audioData.buffer3.state = BUFFER_EMPTY;
                seekingEnabled = false;
                lastSeekAdcResult = 0U;
                seekPending = false;
                stopDisplayTimer = false;
                elapsedSeconds = 0U;
                overflowCount = 0U;
                timestampUpdatePending = false;
                selectedFileIndex++;
                if (selectedFileIndex >= fileCount)
                {
                    selectedFileIndex = 0U;
                }
                (void)strncpy(currentFileName, fileNames[selectedFileIndex], SYS_FS_FILE_NAME_LEN);
                currentFileName[SYS_FS_FILE_NAME_LEN] = '\0';
                (void)strcpy(appSession.fileName, fileNames[selectedFileIndex]);
                if (strcmp(appSession.fileName, FILE_NAME_RECORD_AND_PLAYBACK) != 0)
                {
                    appSession.pcmFormat = PCM_FORMAT_16BIT_SIGNED;
                }
                else
                {
                    appSession.pcmFormat = PCM_FORMAT_10BIT_UNSIGNED;
                }
                TC7_TimerStart();
                appSession.state = APP_AUDIO_STATE_FILE_OPEN;  
            }
            else
            {
                /* Error on file close. */
                appSession.state = APP_AUDIO_STATE_ERROR;
            }
            break;
        }
        
        case APP_AUDIO_STATE_MIC_RECORD:
        {           
            /* Initialize the descriptor linked list for recording. */
            APP_AUDIO_InitializeTxLinkedListDescriptorRecord();
            /* Setup DMA transfer for sampled data. */
            if (DMAC_ChannelLinkedListTransfer(DMAC_CHANNEL_1, &pTxLinkedListDesc[0]) == true)
            {
                /* No action required. */
            }
            else
            {
                /* No action required. */
            }
            DMAC_ChannelCallbackRegister(DMAC_CHANNEL_1, &DMAC_TransmitCompleteCallback_CH1, 0U);
            /* Setup ADC1 callback for VU update. */
            ADC1_CallbackRegister(&ADC1_ResultReadyCallback, 0U);
            /* Enable ADC1. */
            ADC1_Enable();
            /* Start the DMA transfer of sample data. */
            TC2_TimerStart();
            /* Signal an immediate OLED timestamp update. */
            timestampUpdatePending = true;
            /* Set RGB LED to red to indicate record state. */
            APP_AUDIO_SetRGBLED(true, false, false);
            /* Wait for DMA CH1 Callback to update application state. */
            appSession.state = APP_AUDIO_STATE_WAIT;
            break;
        }
        
        case APP_AUDIO_STATE_DATA_WRITE:
        {
            DBG2_Set();
            uint16_t nBytesWritten = 0U;
            uint16_t nBytesEncoded = 0U;
            appSession.state = APP_AUDIO_STATE_WAIT;
            /* Write all FULL buffers in order to SD card. */
            for (uint8_t i = 0U; i < (uint8_t)NUM_AUDIO_BUFFERS_FOR_RECORD; i++)
            {
                if (buffers[bufferProcessIndex]->state == BUFFER_FULL) 
                {
                    /* Encode buffer. */
                    appSession.wavData.byteNumber = APP_AUDIO_EncodeSDCardData(
                        buffers[bufferProcessIndex]->data,
                        (uint8_t* const)(&(appSession.wavData.dataBufferRecord)[0]),
                        (uint16_t)MAX_SD_CARD_BUFFER_SIZE_FOR_RECORD,
                        &nBytesEncoded);

                    /* Write buffer to SD card. */
                    if (APP_AUDIO_WriteSDCardData(
                            appSession.fileHandle,
                            appSession.wavData.dataBufferRecord,
                            (uint16_t)MAX_SD_CARD_BUFFER_SIZE_FOR_RECORD,
                            &nBytesWritten))
                    {
                        totalByteNumber += nBytesWritten;
                        buffers[bufferProcessIndex]->state = BUFFER_EMPTY;
                        bufferFullCount--;
                        /* Advance to next buffer. */
                        bufferProcessIndex = (uint8_t)(((uint32_t)bufferProcessIndex + 1U) % (uint32_t)NUM_AUDIO_BUFFERS_FOR_RECORD);
                    }
                    else
                    {
                        /* Error on file write. */
                        appSession.state = APP_AUDIO_STATE_ERROR;
                    }
                }
            }
            DBG2_Clear();
            break;
        }
        
        case APP_AUDIO_STATE_FILE_TRUNCATE:
        {
            /* Truncate file to current position. */
            if (SYS_FS_FileTruncate(appSession.fileHandle) == SYS_FS_RES_SUCCESS)
            {
                appSession.state = APP_AUDIO_STATE_WAV_HEADER_WRITE;
            }
            else
            {
                /* Error on file truncation. */
                appSession.state = APP_AUDIO_STATE_ERROR;                        
            }
            break;
        }
        
        case APP_AUDIO_STATE_UNMOUNT:
        {
            /* Close current file. */
            if (SYS_FS_FileClose(appSession.fileHandle) == SYS_FS_RES_SUCCESS)
            {
                /* Unmount current drive */
                if (SYS_FS_Unmount("/mnt/MYDRIVE") == SYS_FS_RES_SUCCESS)
                {
                    /* Set application state to INIT. */
                    appSession.state = APP_AUDIO_STATE_INIT;                      
                }
                else
                {
                    /* Error on drive unmount. */
                    appSession.state = APP_AUDIO_STATE_ERROR;
                }
            }
            else
            {
                /* Error on file close. */
                appSession.state = APP_AUDIO_STATE_ERROR; 
            }
            break;
        }
             
        case APP_AUDIO_STATE_ERROR:
        {
            /* On error, signal on GPIO and unmount. */
            appSession.state = APP_AUDIO_STATE_UNMOUNT;
            break;
        }
            
        default: /* No action required. */ break;
    }
}
