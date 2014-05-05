/*
 * Copyright (C) 2014, Michele Balistreri
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef CONFIG_H
#define	CONFIG_H

// PIC18F14K50 Configuration Bit Settings

// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = OFF     // USB Clock Selection bit (USB clock comes directly from the OSC1/OSC2 oscillator block; no divide)

// CONFIG1H
#pragma config FOSC = IRC       // Oscillator Selection bits (Internal RC oscillator)
#pragma config PLLEN = OFF      // 4 X PLL Enable bit (PLL is under software control)
#pragma config PCLKEN = ON      // Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = NOSLP    // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only and disabled in Sleep mode (SBOREN is disabled))
#pragma config BORV = 22        // Brown-out Reset Voltage bits (VBOR set to 2.2 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 256    // Watchdog Timer Postscale Select bits (1:256)

// CONFIG3H
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up bit (The system clock is held off until the HFINTOSC is stable.)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RA3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = OFF      // Boot Block Size Select bit (1kW boot block size)
#pragma config XINST = OFF       // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF       // Table Write Protection bit (Block 1 not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)

// Definitions
#define _XTAL_FREQ 16000000
#define TLV_MAX_INT_16_BIT

#define EUI_64_ADDR 0
#define PAN_ID_ADDR 8
#define SHORT_ADDRESS_ADDR 10
#define CHANNEL_ADDR 12
#define AES_KEY 16

// Includes
#include <xc.h>
#include "MRF24J40.h"
#include "osnp.h"

// MRF24J40 HAL Configuration
#define mrf24j40_set_ie(v) (INTCON3bits.INT1E = v)
#define mrf24j40_get_ie() (INTCON3bits.INT1E)

#define mrf24j40_reset_pin(v) (PORTCbits.RC0 = v)
#define mrf24j40_wake_pin(v) (PORTBbits.RB5 = v)
#define mrf24j40_cs_pin(v) (PORTCbits.RC2 = v)

#define mrf24j40_spi_read() spi_read()
#define mrf24j40_spi_write(val) spi_write(val)
#define mrf24j40_delay_us(v) __delay_us(v)
#define mrf24j40_delay_ms(v) __delay_ms(v)

// OSNP HAL Configuration
#define osnp_load_eui(buf) load_eeprom(buf, EUI_64_ADDR, 8); mrf24j40_set_eui(buf)
#define osnp_load_pan_id(buf) load_eeprom(buf, PAN_ID_ADDR, 2); mrf24j40_set_pan(buf)
#define osnp_load_short_address(buf) load_eeprom(buf, SHORT_ADDRESS_ADDR, 2); mrf24j40_set_short_addr(buf)
#define osnp_load_channel(ch) load_eeprom(ch, CHANNEL_ADDR, 1)
#define osnp_load_aes_key(buf) load_eeprom(buf, AES_KEY, 16)

#define osnp_write_pan_id(buf) write_eeprom(buf, PAN_ID_ADDR, 2); mrf24j40_set_pan(buf)
#define osnp_write_short_address(buf) write_eeprom(buf, SHORT_ADDRESS_ADDR, 2); mrf24j40_set_short_addr(buf)
#define osnp_write_channel(ch) write_eeprom(ch, CHANNEL_ADDR, 1)
#define osnp_process_command(frame, in_off, resp_frame, resp_off, associated) process_command(frame, in_off, resp_frame, resp_off, associated)

#define osnp_switch_channel(ch) mrf24j40_set_channel(ch)
#define osnp_transmit_frame(frame) mrf24j40_txpkt((frame)->backing_buffer, (frame)->header_len, (frame)->payload_len);
#define osnp_get_pending_frames() mrf24j40_get_pending_frame()

#define osnp_start_channel_scanning_timer() enable_wdt(1, 0)
#define osnp_start_association_wait_timer() enable_wdt(10, 0)
#define osnp_start_poll_timer() enable_wdt(2, 1)
#define osnp_start_pending_data_wait_timer() enable_wdt(1, 0)
#define osnp_stop_active_timer() disable_wdt()

#define OSNP_DEVICE_CAPABILITES RX_POLL_DRIVEN


// Function prototypes
unsigned char spi_read(void);
void spi_write(unsigned char data);
void load_eeprom(unsigned char *buf, unsigned int addr, unsigned int len);
void write_eeprom(unsigned char *buf, unsigned int addr, unsigned int len);
void enable_wdt(unsigned char periods, unsigned char radio_can_sleep);
void disable_wdt(void);
void process_command(struct ieee802_15_4_frame *frame, unsigned int *in_off, struct ieee802_15_4_frame *resp_frame, unsigned int *resp_off, unsigned char associated);
#endif	/* CONFIG_H */

