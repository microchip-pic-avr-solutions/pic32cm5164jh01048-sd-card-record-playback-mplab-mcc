#include "ssd1306.h"

/* I2C Function Implementations */

/* Function to initialize the SSD1306 display */
void SSD1306_Init(void)
{
    /* Turn display off during setup. */
    SSD1306_SendCommand(SSD1306_CMD_DISPLAY_OFF);
    /* Set display clock divide ratio/oscillator frequency. */
    SSD1306_SendCommand(SSD1306_CMD_SET_DISPLAY_CLK_DIV);               // 0xD5
    SSD1306_SendCommand(0x80U);                                         // Recommended oscillator frequency
    /* Set multiplex ratio (number of display rows). */
    SSD1306_SendCommand(SSD1306_CMD_SET_MUX_RATIO);                     // 0xA8
    SSD1306_SendCommand(SSD1306_HEIGHT - 1U);                           // 0x3F for 64 rows
    /* Set display offset (shift mapping of display rows). */
    SSD1306_SendCommand(SSD1306_CMD_SET_DISPLAY_OFFSET);                // 0xD3
    SSD1306_SendCommand(0x00U);                                         // No offset
    /* Set display start line (first row to display). */
    SSD1306_SendCommand(SSD1306_CMD_SET_DISPLAY_START_LINE | 0x00U);    // 0x40
    /* Enable charge pump regulator (for internal voltage). */
    SSD1306_SendCommand(OLED_CMD_SET_CHARGE_PUMP);                      // 0x8D
    SSD1306_SendCommand(0x14U);                                         // Enable charge pump
    /* Set memory addressing mode. */
    SSD1306_SendCommand(SSD1306_CMD_SET_MEMORY_ADDR_MODE);              // 0x20
    SSD1306_SendCommand(0x00U);                                         // Horizontal addressing mode
    /* Set segment remap (column address 0 is mapped to SEG127). */
    SSD1306_SendCommand(SSD1306_CMD_SET_SEGMENT_REMAP);                 // 0xA1
    /* Set COM output scan direction (remap rows). */
    SSD1306_SendCommand(SSD1306_CMD_SET_COM_SCAN_MODE_REMAP);           // 0xC8
    /* Set COM pins hardware configuration. */
    SSD1306_SendCommand(SSD1306_CMD_SET_COM_PIN_MAP);                   // 0xDA
    SSD1306_SendCommand(0x12U);                                         // Recommended for 128x64
    /* Set contrast control. */
    SSD1306_SendCommand(SSD1306_CMD_SET_CONTRAST);                      // 0x81
    SSD1306_SendCommand(0x8FU);                                         // Medium contrast
    /* Set pre-charge period. */
    SSD1306_SendCommand(SSD1306_CMD_SET_PRECHARGE);                     // 0xD9
    SSD1306_SendCommand(0xF1U);                                         // Recommended value
    /* Set VCOMH deselect level. */
    SSD1306_SendCommand(SSD1306_CMD_SET_VCOMH_DESELCT);                 // 0xDB
    SSD1306_SendCommand(0x40U);                                         // Recommended value
    /* Resume to RAM content display (normal mode). */
    SSD1306_SendCommand(SSD1306_CMD_DISPLAY_RAM);                       // 0xA4
    /* Set normal display (not inverted). */
    SSD1306_SendCommand(SSD1306_CMD_DISPLAY_NORMAL);                    // 0xA6
    /* Deactivate any scrolling. */
    SSD1306_SendCommand(SSD1306_CMD_DEACTIVATE_SCROLL);                 // 0x2E
    /* Turn display on. */
    SSD1306_SendCommand(SSD1306_CMD_DISPLAY_ON);                        // 0xAF
}

/* Send I2C Command (single command byte). */
void SSD1306_SendCommand(uint8_t command)
{
    static uint8_t cmd[2];
    cmd[0] = SSD1306_CONTROL_BYTE_CMD_SINGLE;
    cmd[1] = command;
    while (SERCOM0_I2C_IsBusy() == true) 
    {
        ;
    }
    if (SERCOM0_I2C_Write(SSD1306_I2C_ADDRESS, cmd, sizeof(cmd)) == true)
    {
        /* No action required. */
    }
    else
    {
        /* No action required. */
    }
    while (SERCOM0_I2C_IsBusy() == true) 
    {
        ;
    }
}

/* Send I2C Data (single data byte). */
void SSD1306_SendData(uint8_t data)
{
    static uint8_t d[2];
    d[0] = SSD1306_CONTROL_BYTE_DATA_STREAM;
    d[1] = data;
    while (SERCOM0_I2C_IsBusy() == true) 
    {
        ;
    }
    if (SERCOM0_I2C_Write(SSD1306_I2C_ADDRESS, d, sizeof(d)) == true)
    {
        /* No action required. */
    }
    else
    {
        /* No action required. */
    }
    while (SERCOM0_I2C_IsBusy() == true) 
    {
        ;
    }
}

/* Write a single character using 5x8 font. */
void SSD1306_WriteCharacter5x8(char character)
{
    for (uint8_t i = 0U; i < 5U; i++)
    {
        SSD1306_SendData(ASCII5x8[((uint8_t)character - 0x20U)][i]);
    }  
    SSD1306_SendData(0x00U);    // 1-pixel space
}

/* Write a string using 5x8 font. */
void SSD1306_WriteString5x8(const char *str)
{
    while (*str != '\0')
    {
        SSD1306_WriteCharacter5x8(*str++);
    } 
}

void SSD1306_SetAddressingMode(uint8_t mode)
{
    SSD1306_SendCommand(SSD1306_CMD_SET_MEMORY_ADDR_MODE);  // 0x20
    SSD1306_SendCommand(mode);  // 0x00: Horizontal, 0x01: Vertical, 0x02: Page
}

void SSD1306_SetColumnRange(uint8_t start, uint8_t end)
{
    SSD1306_SendCommand(SSD1306_CMD_SET_COLUMN_RANGE);  // 0x21
    SSD1306_SendCommand(start);
    SSD1306_SendCommand(end);
}

void SSD1306_SetPageRange(uint8_t start, uint8_t end)
{
    SSD1306_SendCommand(SSD1306_CMD_SET_PAGE_RANGE); // 0x22
    SSD1306_SendCommand(start);
    SSD1306_SendCommand(end);
}

void SSD1306_SetPage(uint8_t page_num)
{
    SSD1306_SendCommand(SSD1306_CMD_SET_PAGE_ADDR | (page_num & 0x07U));    // 0xB0 | page
}

void SSD1306_SetColumn(uint8_t column)
{
    SSD1306_SendCommand(SSD1306_CMD_SET_LOWER_COLUMN_ADDR | (column & 0x0FU));  // 0x00 | lower nibble
    SSD1306_SendCommand(SSD1306_CMD_SET_HIGHER_COLUMN_ADDR | ((column >> 4U) & 0x0FU)); // 0x10 | upper nibble
}

/* Clear the entire OLED screen. */
void SSD1306_Clear(void)
{
    SSD1306_SetAddressingMode(0x00U);    // Horizontal
    SSD1306_SetColumnRange(0U, SSD1306_COLUMNS - 1U);
    SSD1306_SetPageRange(0U, SSD1306_PAGES - 1U);

    for (uint16_t i = 0U; i < (SSD1306_COLUMNS * SSD1306_PAGES); i++)
    {
        SSD1306_SendData(0x00U);
    }
}

/* Clear a single line (page). */
void SSD1306_ClearLine(uint8_t page_num)
{
    SSD1306_SetAddressingMode(0x02U); // Page
    SSD1306_SetPage(page_num);
    SSD1306_SetColumn(0U);

    for (uint8_t col = 0U; col < SSD1306_COLUMNS; col++)
    {
        SSD1306_SendData(0x00U);
    }
}

/* Write a single character using 8x16 font at (page, column). */
void SSD1306_WriteCharacter8x16(char character, uint8_t page, uint8_t column)
{
    SSD1306_SetAddressingMode(0x02U);
    SSD1306_SetPage(page);
    SSD1306_SetColumn(column);
    for (uint8_t i = 0U; i < 8U; i++)
    {
        SSD1306_SendData(ASCII8x16[((uint8_t)character - 0x20U)][i]);
    }
    SSD1306_SetPage(page + 1U);
    SSD1306_SetColumn(column);
    for (uint8_t i = 8U; i < 16U; i++)
    {
        SSD1306_SendData(ASCII8x16[((uint8_t)character - 0x20U)][i]);
    }
}

/* Write a string using 8x16 font at (page, column). */
void SSD1306_WriteString8x16(const char *str, uint8_t page, uint8_t column)
{
    uint8_t col = column;
    while (*str != '\0')
    {
        if (col > (SSD1306_COLUMNS - 8U)) 
        {
            break;
        }
        SSD1306_WriteCharacter8x16(*str, page, col);
        col += 8U;
        str++;
    }
}

void SSD1306_DrawBitmap(const uint8_t* bitmap)
{
    SSD1306_SetAddressingMode(0x00U); // Horizontal
    SSD1306_SetColumnRange(0U, SSD1306_COLUMNS - 1U);
    SSD1306_SetPageRange(0U, SSD1306_PAGES - 1U);

    for (uint16_t i = 0U; i < (SSD1306_COLUMNS * SSD1306_PAGES); i++)
    {
        SSD1306_SendData(bitmap[i]);
    }
}

void SSD1306_DrawBitmapCustom(
    uint8_t x, uint8_t y,
    uint8_t width, uint8_t height,
    const uint8_t* bitmap)
{
    SSD1306_SetAddressingMode(0x02U);

    uint8_t startPage = y / 8U;
    uint8_t pages = (height + 7U) / 8U;
    uint16_t byteIndex = 0U;

    for (uint8_t page = 0U; page < pages; page++)
    {
        SSD1306_SetPage(startPage + page);
        SSD1306_SetColumn(x);

        for (uint8_t col = 0U; col < width; col++)
        {
            SSD1306_SendData(bitmap[byteIndex++]);
        }
    }
}

size_t SSD1306_BuildI2CBuffer(uint8_t page, const char* displayBuf, uint8_t* oledBuffer)
{
    size_t pos = 0U;
    size_t strLen = strlen(displayBuf);
    /* Set page addressing mode. */
    oledBuffer[pos++] = SSD1306_CONTROL_BYTE_CMD_SINGLE;
    oledBuffer[pos++] = SSD1306_CMD_SET_MEMORY_ADDR_MODE;   // 0x20
    oledBuffer[pos++] = SSD1306_CONTROL_BYTE_CMD_SINGLE;
    oledBuffer[pos++] = 0x02U;   // 0x02 = page addressing mode
    /* Set page address. */
    oledBuffer[pos++] = SSD1306_CONTROL_BYTE_CMD_SINGLE;
    oledBuffer[pos++] = SSD1306_CMD_SET_PAGE_ADDR | (page & 0x07U);
    /* Set column address to 0. */
    oledBuffer[pos++] = SSD1306_CONTROL_BYTE_CMD_SINGLE;
    oledBuffer[pos++] = SSD1306_CMD_SET_LOWER_COLUMN_ADDR | (0x00U & 0x0FU);
    oledBuffer[pos++] = SSD1306_CONTROL_BYTE_CMD_SINGLE;
    oledBuffer[pos++] = SSD1306_CMD_SET_HIGHER_COLUMN_ADDR | ((0x00U >> 4U) & 0x0FU);
    /* Write the string. */
    oledBuffer[pos++] = SSD1306_CONTROL_BYTE_DATA_STREAM;
    for (size_t i = 0U; i < strLen; i++)
    {
        uint8_t c = (uint8_t)displayBuf[i];
        for (uint8_t j = 0U; j < 5U; j++)
        {
            oledBuffer[pos++] = ASCII5x8[c - 0x20U][j];
        }
        oledBuffer[pos++] = 0x00U; // 1-pixel space
    }
    return pos; // Total length of buffer to send via DMA
}


