/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  

#ifndef SSD1306_H
#define SSD1306_H

#include "font.h"
#include "definitions.h"
#include <string.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
#define SUCCESS                                1U
    
// Some fundamental define for screen controller
#define SSD1306_PAGES                          8U
#define SSD1306_HEIGHT                         64U
#define SSD1306_COLUMNS                        128U
#define SSD1306_CLEAR_SIZE                     1024U
#define SSD1306_I2C_ADDRESS                    0x3DU

// Control byte
#define SSD1306_CONTROL_BYTE_CMD_SINGLE        0x80U
#define SSD1306_CONTROL_BYTE_CMD_STREAM        0x00U
#define SSD1306_CONTROL_BYTE_DATA_STREAM       0x40U

// Fundamental commands (pg.28)
#define SSD1306_CMD_SET_CONTRAST               0x81U    // follow with 0x7F
#define SSD1306_CMD_DISPLAY_RAM                0xA4U
#define SSD1306_CMD_DISPLAY_ALLON              0xA5U
#define SSD1306_CMD_DISPLAY_NORMAL             0xA6U
#define SSD1306_CMD_DISPLAY_INVERTED           0xA7U
#define SSD1306_CMD_DISPLAY_OFF                0xAEU
#define SSD1306_CMD_DISPLAY_ON                 0xAFU

// Display Scrolling Parameters
#define SSD1306_CMD_RIGHT_HORIZONTAL_SCROLL              0x26U  // Init rt scroll
#define SSD1306_CMD_LEFT_HORIZONTAL_SCROLL               0x27U  // Init left scroll
#define SSD1306_CMD_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29U  // Init diag scroll
#define SSD1306_CMD_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2AU  // Init diag scroll
#define SSD1306_CMD_DEACTIVATE_SCROLL                    0x2EU  // Stop scroll
#define SSD1306_CMD_ACTIVATE_SCROLL                      0x2FU  // Start scroll
#define SSD1306_CMD_SET_VERTICAL_SCROLL_AREA             0xA3U  // Set scroll range

// Addressing Command Table (pg.30)
#define SSD1306_CMD_SET_LOWER_COLUMN_ADDR      0x00U    // Set Lower Column Start Address for Page Addressing Mode, using X[3:0]
#define SSD1306_CMD_SET_HIGHER_COLUMN_ADDR     0x10U    // Set Higher Column Start Address for Page Addressing Mode, using X[3:0]
#define SSD1306_CMD_SET_MEMORY_ADDR_MODE       0x20U    // follow with 00b= Horizontal Addressing Mode; 01b=Vertical Addressing Mode; 10b= Page Addressing Mode (RESET)
#define SSD1306_CMD_SET_COLUMN_RANGE           0x21U    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define SSD1306_CMD_SET_PAGE_RANGE             0x22U    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7
#define SSD1306_CMD_SET_PAGE_ADDR              0xB0U

// Hardware Config (pg.31)
#define SSD1306_CMD_SET_DISPLAY_START_LINE     0x40U
#define SSD1306_CMD_SET_SEGMENT_REMAP          0xA1U
#define SSD1306_CMD_SET_MUX_RATIO              0xA8U    // follow with 0x3F = 64 MUX
#define SSD1306_CMD_SET_COM_SCAN_MODE_NORMAL   0xC0U
#define SSD1306_CMD_SET_COM_SCAN_MODE_REMAP    0xC8U
#define SSD1306_CMD_SET_DISPLAY_OFFSET         0xD3U    // follow with 0x00
#define SSD1306_CMD_SET_COM_PIN_MAP            0xDAU    // follow with 0x12
#define SSD1306_CMD_NOP                        0xE3U    // NOP

// Timing and Driving Scheme (pg.32)
#define SSD1306_CMD_SET_DISPLAY_CLK_DIV        0xD5U    // follow with 0x80
#define SSD1306_CMD_SET_PRECHARGE              0xD9U    // follow with 0xF1
#define SSD1306_CMD_SET_VCOMH_DESELCT          0xDBU    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP               0x8DU    // follow with 0x14

#define SSD1306_COLUMN_ADDR     0x00U
#define SSD1306_LOWER_ADDRESS   (SSD1306_CMD_SET_LOWER_COLUMN_ADDR | (SSD1306_COLUMN_ADDR & 0x0FU));
#define SSD1306_HIGHER_ADDRESS  (SSD1306_CMD_SET_HIGHER_COLUMN_ADDR | ((SSD1306_COLUMN_ADDR & 0xF0U) >> 4U));

void SSD1306_Init(void);
void SSD1306_SendCommand(uint8_t command);
void SSD1306_SendData(uint8_t data);
void SSD1306_WriteCharacter5x8(char character);
void SSD1306_WriteString5x8(const char *str);
void SSD1306_WriteCharacter8x16(char character, uint8_t page, uint8_t column);
void SSD1306_WriteString8x16(const char *str, uint8_t page, uint8_t column);
void SSD1306_SetAddressingMode(uint8_t mode);
void SSD1306_Clear(void);
void SSD1306_ClearLine(uint8_t page_num);
void SSD1306_SetPage(uint8_t page_num);
void SSD1306_SetPageRange(uint8_t start, uint8_t end);
void SSD1306_SetColumn(uint8_t column);
void SSD1306_SetColumnRange(uint8_t start, uint8_t end);
void SSD1306_DrawBitmap(const uint8_t* bitmap);
void SSD1306_DrawBitmapCustom(
    uint8_t x, uint8_t y,           // Top-left position in pixels
    uint8_t width, uint8_t height,  // Bitmap size in pixels
    const uint8_t* bitmap);         // Bitmap array
size_t SSD1306_BuildI2CBuffer(uint8_t page, const char* displayBuf, uint8_t* oledBuffer);

#ifdef	__cplusplus
}
#endif /* __cplusplus */



#endif //SSD1306_H
