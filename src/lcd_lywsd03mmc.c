#include "tl_common.h"
#include "app_config.h"
#if (DEV_SERVICES & SERVICE_SCREEN) && (DEVICE_TYPE == DEVICE_LYWSD03MMC)
#include "drivers.h"
#include "drivers/8258/gpio_8258.h"
#include "app.h"
#include "i2c.h"
#include "lcd.h"
/*
 *  LYWSD03MMC LCD buffer:  byte.bit

         --5.4--         --4.4--            --3.4--          BLE
  |    |         |     |         |        |         |        2.4
  |   5.5       5.0   4.5       4.0      3.5       3.0  
  |    |         |     |         |        |         |      o 2.5
 5.3     --5.1--         --4.1--            --3.1--          +--- 2.5
  |    |         |     |         |        |         |     2.5|
  |   5.6       5.2   4.6       4.2      3.6       3.2       ---- 2.6
  |    |         |     |         |        |         |     2.5|
         --5.7--         --4.7--     *      --3.7--          ---- 2.7
                                    4.3
                                        --1.4--         --0.4--
                                      |         |     |         |
          2.0      2.0               1.5       1.0   0.5       0.0
          / \      / \                |         |     |         |
    2.2(  ___  2.1 ___  )2.2            --1.1--         --0.1--
          2.1  / \ 2.1                |         |     |         |
               ___                   1.6       1.2   0.6       0.2     %
               2.0                    |         |     |         |     0.3
                                        --1.7--         --0.7--
                           BAT 1.3
*/


RAM u8 lcd_i2c_addr;

#define lcd_send_i2c_byte(a)  send_i2c_byte(lcd_i2c_addr, a)
#define lcd_send_i2c_buf(b, a)  send_i2c_buf(lcd_i2c_addr, (u8 *) b, a)

/* t,H,h,L,o,i  0xe2,0x67,0x66,0xe0,0xC6,0x40 */
// #define LCD_SYM_H	0x67	// "H"
// #define LCD_SYM_i	0x40	// "i"
// #define LCD_SYM_L	0xE0	// "L"
// #define LCD_SYM_o	0xC6	// "o"


// #define LCD_SYM_H	0x67	// "H"
// #define LCD_SYM_i	0x40	// "i"
#define SEG_NUM_DASH	16	// "o
#define SEG_NUM_L		17	// "o
#define SEG_NUM_o		18	// "o
#define SEG_NUM_H		19	// "o
#define SEG_NUM_I		1	// "o

#define LCD_SYM_BLE	 0x10	// connect
#define LCD_SYM_MAIL 0x10	// connect

#define LCD_SYM_BAT	0x08	// battery

const u8 lcd_init_cmd_b14[] =	{0x80,0x3B,0x80,0x02,0x80,0x0F,0x80,0x95,0x80,0x88,0x80,0x88,0x80,0x88,0x80,0x88,0x80,0x19,0x80,0x28,0x80,0xE3,0x80,0x11};
								//	{0x80,0x40,0xC0,byte1,0xC0,byte2,0xC0,byte3,0xC0,byte4,0xC0,byte5,0xC0,byte6};
const u8 lcd_init_clr_b14[] =	{0x80,0x40,0xC0,0,0xC0,0,0xC0,0,0xC0,0,0xC0,0,0xC0,0,0xC0,0,0xC0,0};

/* Test cmd ():
 * 0400007ceaa49cacbcf0fcc804ffffffff,
 * 0400007c0449
 * 0400007cf3c8 - blink
 */
const u8 lcd_init_b19[]	=	{
0xC8, 0xB4, 0xF0 
};

typedef struct __attribute__((packed)) _dma_uart_buf_t {
	volatile u32 dma_len;
	u8 start;
	u8 data[6];
	u8 chk;
	u8 end;
} dma_uart_buf_t;

RAM dma_uart_buf_t utxb;

/* 0,1,2,3,4,5,6,7,8,9,A,b,C,d,E,F*/
const u8 display_numbers[] = {
    0x5F, // 0
    0x06, // 1
    0x3D, // 2
    0x2F, // 3
    0x66, // 4
    0x6B, // 5
    0x7B, // 6
    0x0E, // 7
    0x7F, // 8
    0x6F, // 9
    0x7E, // A
    0x73, // B
    0x59, // C
    0x37, // D
    0x79, // E
    0x78, // F
	0x20, // -
	0x51, // L
	0x33, //o
	0x76, // H

};

// const u8 display_numbers_small[] = {
// 	0b10101111, // 0xAF
// 	0b00000110, // 0x06
// 	0b01101101, // 0x6D
// 	0b01001111, // 0x4F
// 	0b11000110, // 0xC6
// 	0b11001011, // 0xCB
// 	0b11101011, // 0xEB
// 	0b00001110, // 0x0E
// 	0b11101111, // 0xEF
// 	0b11001111, // 0xCF
// 	0b11101110, // 0xEE
// 	0b11100011, // 0xE3
// 	0b10101001, // 0xA9
// 	0b01100111, // 0x67
// 	0b11101001, // 0xE9
// 	0b11101000  // 0xE8

// };

const u8 display_numbers_small[] = {
    0xA0, // Originally 0xAF
    0x00, // Originally 0x06
    0x60, // Originally 0x6D
    0x40, // Originally 0x4F
    0xC0, // Originally 0xC6
    0xC0, // Originally 0xCB
    0xE0, // Originally 0xEB
    0x00, // Originally 0x0E
    0xE0, // Originally 0xEF
    0xC0, // Originally 0xCF
    0xE0, // Originally 0xEE
    0xE0, // Originally 0xE3
    0xA0,  // Originally 0xA9
    0x60, // Originally 0x67
    0xE0, // Originally 0xE9
    0xE0,  // Originally 0xE8
	0x40, // Dash
	0xA0, //L
	0x60,  //o
	0xE0, // H

};
// static _attribute_ram_code_ u8 reverse(u8 revByte) {
//    revByte = (revByte & 0xF0) >> 4 | (revByte & 0x0F) << 4;
//    revByte = (revByte & 0xCC) >> 2 | (revByte & 0x33) << 2;
//    revByte = (revByte & 0xAA) >> 1 | (revByte & 0x55) << 1;
//    return revByte;
// }


//---------------------
// B1.6 (UART/SPI)
_attribute_ram_code_
static void lcd_set_buf_uart_spi(u8 *p) {
	utxb.start = 0xAA;
	utxb.data[5] = *p++;
	utxb.data[4] = *p++;
	utxb.data[3] = *p++;
	utxb.data[2] = *p++;
	utxb.data[1] = *p++;
	utxb.data[0] = *p;
	utxb.chk = utxb.data[0]^utxb.data[1]^utxb.data[2]^utxb.data[3]^utxb.data[4]^utxb.data[5];
	utxb.end = 0x55;
}

//---------------------
// new B1.6 (SPI LCD)

#define CLK_DELAY_US	24 // 48 -> 10 kHz

_attribute_ram_code_
static void lcd_send_spi_byte(u8 b) {
	u32 x = b;
	for(int i = 0; i < 8; i++) {
		BM_CLR(reg_gpio_out(GPIO_LCD_CLK), GPIO_LCD_CLK & 0xff); // CLK down
		if(x & 0x01)
			BM_SET(reg_gpio_out(GPIO_LCD_SDI), GPIO_LCD_SDI & 0xff); // data "1"
		else
			BM_CLR(reg_gpio_out(GPIO_LCD_SDI), GPIO_LCD_SDI & 0xff); // data "0"
		sleep_us(CLK_DELAY_US);
		BM_SET(reg_gpio_out(GPIO_LCD_CLK), GPIO_LCD_CLK & 0xff); // CLK Up
		sleep_us(CLK_DELAY_US);
		x >>= 1;
	}
	sleep_us(CLK_DELAY_US);
}

// send spi buffer (new B1.6 (SPI LCD))
_attribute_ram_code_
static void lcd_send_spi(void) {
	BM_CLR(reg_gpio_oen(GPIO_LCD_SDI), GPIO_LCD_SDI & 0xff); // SDI output enable
	u8 * pd = &utxb.start;
	do {
		lcd_send_spi_byte(*pd++);
	} while(pd <= &utxb.end);
	BM_SET(reg_gpio_oen(GPIO_LCD_SDI), GPIO_LCD_SDI & 0xff); // SDI output disable
}

//-----------------------
// B1.5, B1.6 UART LCD
// UART 38400 BAUD
#define UART_BAUD 38400
#if CLOCK_SYS_CLOCK_HZ == 16000000
#define uartCLKdiv 51 // 16000000/(7+1)/(51+1)=38461.538...
#define bwpc 7
#elif CLOCK_SYS_CLOCK_HZ == 24000000
#define uartCLKdiv 124 // 24000000/(4+1)/(124+1)=38400
#define bwpc 4
#elif CLOCK_SYS_CLOCK_HZ == 32000000
#define uartCLKdiv 103 // 32000000/(7+1)/(103+1)=38461.538...
#define bwpc 7
#elif CLOCK_SYS_CLOCK_HZ == 48000000
#define uartCLKdiv 124 // 48000000/(9+1)/(124+1)=38400
#define bwpc 9
#endif

// B1.5, B1.6 (UART LCD)
_attribute_ram_code_
static void lcd_send_uart(void) {
	// init uart
	reg_clk_en0 |= FLD_CLK0_UART_EN;
	///reg_clk_en1 |= FLD_CLK1_DMA_EN;
	uart_reset();

	utxb.dma_len = sizeof(utxb) - sizeof(utxb.dma_len);

	// reg_dma1_addr/reg_dma1_ctrl
	REG_ADDR32(0xC04) = (unsigned short)((u32)(&utxb)) //set tx buffer address
		| 	(((sizeof(utxb)+15)>>4) << 16); //set tx buffer size
	///reg_dma1_addrHi = 0x04; (in sdk init?)

	// reg_uart_clk_div/reg_uart_ctrl0
	REG_ADDR32(0x094) = MASK_VAL(FLD_UART_CLK_DIV, uartCLKdiv, FLD_UART_CLK_DIV_EN, 1)
		|	((MASK_VAL(FLD_UART_BPWC, bwpc)	| (FLD_UART_TX_DMA_EN)) << 16) // set bit width, enable UART DMA mode
			| ((MASK_VAL(FLD_UART_CTRL1_STOP_BIT, 0)) << 24) // 00: 1 bit, 01: 1.5bit 1x: 2bits;
		;
	reg_dma_chn_en |= FLD_DMA_CHN_UART_TX;
	///reg_dma_chn_irq_msk |= FLD_DMA_IRQ_UART_TX;

	// GPIO_PD7 set TX UART pin
	REG_ADDR8(0x5AF) = (REG_ADDR8(0x5AF) &  (~(BIT(7)|BIT(6)))) | BIT(7);
	BM_CLR(reg_gpio_func(GPIO_LCD_URX), GPIO_LCD_URX & 0xff);
	// GPIO_PDB set RX UART pin
	REG_ADDR8(0x5AB) = (REG_ADDR8(0x5AB) &  (~(BIT(7)|BIT(6)))) | BIT(7);
	BM_CLR(reg_gpio_func(GPIO_LCD_UTX), GPIO_LCD_UTX & 0xff);
	// start send DMA
	reg_dma_tx_rdy0 |= FLD_DMA_CHN_UART_TX; // start tx
	// wait send 9 tx + 1 rx bytes * 10 bits / 38400 baud = 0.002604166 sec = 2.605 ms
	if(wrk.ota_is_working)
		sleep_us(2600); // power ~3.5 mA
	else
		pm_wait_us(2600); // power ~3.1 mA

	while (reg_dma_tx_rdy0 & FLD_DMA_CHN_UART_TX);
	while (!(reg_uart_status1 & FLD_UART_TX_DONE));

	/* wait rx 1 bytes ok = 0xAA */
	// Time rx 1 bytes * 10 bits / 38400 baud = 0.0002604166 sec = 260.5 us power ~3.6 mA
	u32 wt = clock_time();
	do
	{
		if(reg_uart_buf_cnt & FLD_UART_RX_BUF_CNT) {
			utxb.end = reg_uart_data_buf0;
			break;
		}
	} while(!clock_time_exceed(wt, 512));
	// set low/off power UART
	reg_uart_clk_div = 0;
}

_attribute_ram_code_
void send_to_lcd(void){
	unsigned int buff_index;
	// u8 ttmp_buff[]={0,display_numbers[4],display_numbers[5],display_numbers[7],255,255,255,255,255};
	// show_big_number_x10(-7);
	// for (int i = 5; i<9; i++)  display_buff[i] = 0;
	// u8 * p = ttmp_buff;
	// u8 * p = display_buff;

	if(cfg.flg2.screen_off)
		return;
		if ((reg_clk_en0 & FLD_CLK0_I2C_EN)==0)
			init_i2c();
		else {
			gpio_setup_up_down_resistor(I2C_SCL, PM_PIN_PULLUP_10K);
			gpio_setup_up_down_resistor(I2C_SDA, PM_PIN_PULLUP_10K);
		}
		// if (lcd_i2c_addr == (B14_I2C_ADDR << 1)) {
			// B1.4, B1.7, B2.0
			reg_i2c_speed = (u8)(CLOCK_SYS_CLOCK_HZ/(4*700000)); // 700 kHz
			reg_i2c_id = lcd_i2c_addr;
			reg_i2c_adr = 0x00;
			reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR ;//| FLD_I2C_CMD_DO;
			while (reg_i2c_status & FLD_I2C_CMD_BUSY);
			// reg_i2c_adr = 0xC0;
			for(buff_index = 0; buff_index < sizeof(display_buff); buff_index++) {
				reg_i2c_do = display_buff[buff_index];
				reg_i2c_ctrl = FLD_I2C_CMD_DO;//FLD_I2C_CMD_ADDR
				while (reg_i2c_status & FLD_I2C_CMD_BUSY);
			}
			reg_i2c_ctrl = FLD_I2C_CMD_STOP;
		while (reg_i2c_status & FLD_I2C_CMD_BUSY);
}

//const u8 lcd_init_cmd[] = {0xea,0xf0, 0xc0, 0xbc}; // sleep all 9.4 uA

void init_lcd(void){
	memset(display_buff, BIT(1), sizeof(display_buff)); // display off "--- ---"
	lcd_i2c_addr = (u8) scan_i2c_addr(B14_I2C_ADDR << 1);
	if (lcd_i2c_addr) { // B1.4, B1.7, B2.0
// 		GPIO_PB6 set in app_config.h!
//		gpio_setup_up_down_resistor(GPIO_PB6, PM_PIN_PULLUP_10K); // LCD on low temp needs this, its an unknown pin going to the LCD controller chip
		if(!cfg.flg2.screen_off) {
			pm_wait_ms(50);
			lcd_send_i2c_buf((u8 *) lcd_init_cmd_b14, sizeof(lcd_init_cmd_b14));
			lcd_send_i2c_buf((u8 *) lcd_init_clr_b14, sizeof(lcd_init_clr_b14));
		}
		return;
	}
	lcd_i2c_addr = (u8) scan_i2c_addr(B19_I2C_ADDR << 1);
	if (lcd_i2c_addr) { // B1.9
		if(cfg.flg2.screen_off) {
			// lcd_send_i2c_byte(0xEA); // BU9792AFUV reset
		} else {
			// lcd_send_i2c_buf((u8 *) lcd_init_b19, sizeof(lcd_init_b19));
			lcd_send_i2c_buf((u8 *) lcd_init_b19, sizeof(lcd_init_b19));
			u8 buff[9] = {0, 255,255,255,255,255,255,255,255};

			// // u16* tmp = buff+5;
			// lcd_send_i2c_buf(buff, 9);
			// pm_wait_ms(500);
			// for(u8 i = 0; i < 9 ; i++){

			// 	buff[5] = (display_numbers[i] & 0x0F )| (buff[5] & 0xF0 );
			// 	buff[6] = (display_numbers_small[i]) | (display_numbers[i+1] & 0x0F ) | (buff[6] & 0b00010000) ;
				
			// 	// buff[6] =  (buff[6] &0xF0 );
			// 	buff[7] = (display_numbers_small[i+1])|(display_numbers[i+2] & 0x0F)| (buff[7] & 0b00010000) ;
				
			// 	// buff[7] = (buff[7] &0xF0 );
			// 	buff[8] = (display_numbers_small[i+2]) |( buff[8] & 0x1F) ;
			// 	lcd_send_i2c_buf(buff, 9);
			// 	pm_wait_ms(500);
			
			
			
			
			
			// }

		}
		return;
	}
	lcd_set_buf_uart_spi(display_buff);
	if (sensor_cfg.sensor_type == TH_SENSOR_SHTC3) { // B1.5 (UART)
		lcd_send_uart();
		return;
	}
	// B1.6 (UART/SPI)
	// Test SPI/UART ?
	gpio_setup_up_down_resistor(GPIO_LCD_SDI, PM_PIN_PULLDOWN_100K);
	sleep_us(256);
	if(BM_IS_SET(reg_gpio_in(GPIO_LCD_SDI), GPIO_LCD_SDI & 0xff) == 0) { // SPI/UART ?
		gpio_setup_up_down_resistor(GPIO_LCD_SDI, PM_PIN_PULLUP_1M);
		lcd_i2c_addr = N16_I2C_ADDR; // SPI
	} else {
		// lcd_i2c_addr = 0
		gpio_setup_up_down_resistor(GPIO_LCD_SDI, PM_PIN_PULLUP_1M);
		lcd_send_uart();
		if(utxb.end == 0xAA)
			return; // UART LCD
		BM_SET(reg_gpio_func(GPIO_LCD_SDI), GPIO_LCD_SDI & 0xff); // GPIO_PB7 set GPIO pin
		BM_SET(reg_gpio_func(GPIO_LCD_CLK), GPIO_LCD_CLK & 0xff); // GPIO_PD7 set GPIO pin
	}
	lcd_i2c_addr = N16_I2C_ADDR; // SPI LCD
	pm_wait_us(1024);
	lcd_send_spi();
}

/* 0x00 = "  "
 * 0x20 = "°Г"
 * 0x40 = " -"
 * 0x60 = "°F"
 * 0x80 = " _"
 * 0xA0 = "°C"
 * 0xC0 = " ="
 * 0xE0 = "°E" */
_attribute_ram_code_
void show_temp_symbol(u8 symbol) {
	switch (symbol)
	{
	case TMP_SYM_F:
		display_buff[3] |= 0x80;
		display_buff[6] &= 0xEF;
		break;
	// case TMP_SYM_C:
	// 	display_buff[6] |= 0x10;
	// 	display_buff[3] &= 0x7F;
	// 	break;
	default:		
		// display_buff[3] &= 0x7F;
		// display_buff[6] &= 0xEF;
	display_buff[5] |= 0x10;
	display_buff[3] &= 0x7F;
		break;
	}

}

/* 0 = "     " off,
 * 1 = " ^-^ "
 * 2 = " -^- "
 * 3 = " ooo "
 * 4 = "(   )"
 * 5 = "(^-^)" happy
 * 6 = "(-^-)" sad
 * 7 = "(ooo)" */
_attribute_ram_code_
void show_smiley(u8 state){
	if (state == 5)
		display_buff[7] |= LCD_SYM_MAIL;
	else
		display_buff[7] &= ~LCD_SYM_MAIL;
}

_attribute_ram_code_
void show_ble_symbol(bool state){
	if (state)
		display_buff[6] |= LCD_SYM_BLE;
	else 
		display_buff[6] &= ~LCD_SYM_BLE;
}

_attribute_ram_code_
void show_battery_symbol(bool state){
	display_buff[4] &= 0x0F;
	if(state >= 80) display_buff[4] |= 0xF0;  
	else if(state >= 60) display_buff[4] |= 0x70;  
	else if(state >= 40) display_buff[4] |= 0x30;  
	else if(state >= 20) display_buff[4] |= 0x10;  

	// if (state)
	// 	display_buff[1] |= LCD_SYM_BAT;
	// else 
	// 	display_buff[1] &= ~LCD_SYM_BAT;
}

/* number in 0.1 (-995..19995), Show: -99 .. -9.9 .. 199.9 .. 1999 */
_attribute_ram_code_
__attribute__((optimize("-Os"))) void show_big_number_x10(s16 number){
//	display_buff[4] = point?0x08:0x00;

   		display_buff[3] &= 0x80;
	if (number > 9999) {
   		display_buff[2] = display_numbers[SEG_NUM_I]; // "i"
   		display_buff[1] = display_numbers[SEG_NUM_H]; // "H"
	} else if (number < -999) {
   		display_buff[2] = display_numbers[SEG_NUM_o]; // "o"
   		display_buff[1] = display_numbers[SEG_NUM_L]; // "L"
	} else {
		display_buff[0] = 0;
		display_buff[1] = 0;
		display_buff[2] = 0;

		if (number < 0){
			number = -number;
			if(number > 100) {
				display_buff[0] = 0b100000; 
				display_buff[2] = 0b10000000; // point,
			}
			else display_buff[1] = 0b100000; // "-"
		}
		else if (number <1995 )
			display_buff[2] = 0b10000000; // point,

		// /* number: -995..19995 */
		// if (number > 1995 || number < -95) {
		// 	display_buff[2] = 0; // no point, show: -99..1999
		// 	if (number < 0){
		// 		number = -number;
		// 		display_buff[0] = 0b100000; // "-"
		// 	}
		// 	number = (number + 5) / 10; // round(div 10)
		// } else { // show: -9.9..199.9
		// 	display_buff[2] = 0b10000000; // point,
		// 	if (number < 0){
		// 		number = -number;
		// 		display_buff[0] = 0b100000; // "-"
		// 	}
		// }
		/* number: -99..1999 */
		if (number > 999) display_buff[0] |= display_numbers[number/ 1000 % 10];// "1" 1000..1999
		if (number > 99) display_buff[1] |= display_numbers[number / 100 % 10];
		if (number > 9) display_buff[2] |= display_numbers[number / 10 % 10];
		else display_buff[2] |= display_numbers[0]; // "0"
	    display_buff[3] |= display_numbers[number %10];
	}
}

/* -9 .. 99 */
_attribute_ram_code_
__attribute__((optimize("-Os"))) void show_small_number(s16 number, bool percent){
	// display_buff[1] = display_buff[1] & 0x08; // and battery
	// display_buff[0] = percent?0x08:0x00;
	u8 tmp_dsp[3] = {0};
	if (number > 999) {
		tmp_dsp[1] = SEG_NUM_I; // "i"
		tmp_dsp[2] = SEG_NUM_H; // "H"
		tmp_dsp[0] = SEG_NUM_DASH;
	} else if (number < -90) {
		tmp_dsp[1] = SEG_NUM_o; // "o"
		tmp_dsp[2] = SEG_NUM_L; // "L"
		tmp_dsp[0] = SEG_NUM_DASH;
	} else {
		if (number < 0) {
			number = -number;
			tmp_dsp[2] = SEG_NUM_DASH; // "-"
		}
		if (number > 99) tmp_dsp[2] = (number / 100 % 10);
		if (number > 9) tmp_dsp[1] = (number /10 %10);
		tmp_dsp[0] = (number  %10);
	}
	display_buff[4] = (display_numbers[tmp_dsp[0]] & 0x0F )| (display_buff[4] & 0xF0 );
	display_buff[5] = (display_numbers_small[tmp_dsp[0]])|(display_numbers[tmp_dsp[1]] & 0x0F )|(display_buff[5] & 0b00010000) ;
	if(!tmp_dsp[2]){
		display_buff[6] = (display_numbers_small[tmp_dsp[1]])|(display_buff[6] & 0b00010000);
		display_buff[7] &= 0x1F ;
	}
	display_buff[6] = (display_numbers_small[tmp_dsp[1]])|(display_numbers[tmp_dsp[2]] & 0x0F )|(display_buff[6] & 0b00010000) ;	
	display_buff[7] = (display_numbers_small[tmp_dsp[2]]) |( display_numbers[7] & 0x1F) ;


}

void show_ota_screen(void) {
	// memset(&display_buff, 0, sizeof(display_buff));
	// display_buff[2] = BIT(4); // "ble"
	// display_buff[3] = BIT(7); // "_"
	// display_buff[4] = BIT(7); // "_"
	// display_buff[5] = BIT(7); // "_"
	// send_to_lcd();
	// if(lcd_i2c_addr == B19_I2C_ADDR << 1)
	// 	lcd_send_i2c_byte(0xf2);
}

// #define SHOW_REBOOT_SCREEN()
void show_reboot_screen(void) {
	// memset(&display_buff, 0xff, sizeof(display_buff));
	// send_to_lcd();
}

#if	USE_DISPLAY_CLOCK
_attribute_ram_code_
void show_clock(void) {
// 	u32 tmp = wrk.utc_time_sec / 60;
// 	u32 min = tmp % 60;
// 	u32 hrs = tmp / 60 % 24;
// 	display_buff[0] = display_numbers[min % 10];
// 	display_buff[1] = display_numbers[min / 10 % 10];
// 	display_buff[2] = 0;
// 	display_buff[3] = display_numbers[hrs % 10];
// 	display_buff[4] = display_numbers[hrs / 10 % 10];
// 	display_buff[5] = 0;
}
#endif // USE_DISPLAY_CLOCK

#endif // DEVICE_TYPE == DEVICE_LYWSD03MMC
