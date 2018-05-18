//Simple PIC12F675 + HC4040 Frequency Counter with binary readout
//
//connection (deoupling capacitors not shown)
//
//
//
//                                                              |--------------------|
//                                                              |                    |
//      Fin>--------------------[220R]------------------------->| CLK                |
//                                          |                   |                    |
//                                          |                   |                    |
//                                         ===                  |                    |
//                                         \|/                  |                    |
//                                         ===                  |                    |
//                                          |                   |     HC4040/4020    |
//                  |--------------|        |                   |                    |
//                  |     CNTR_CLK |>-------|                   |                    |    LEDx8 (or more. W/Optional serieal resistors)
//                  |              |                            |          Pin 3/ Q4 |----->|-----|
//                  |              |                            |          Pin 2/ Q5 |----->|-----|
//                  |              |                            |          Pin 4/ Q6 |----->|-----|
//                  |  PIC12F675   |                            |          Pin13/ Q7 |----->|-----|
//                  |              |                            |          Pin12/ Q8 |----->|-----|
//                  |              |                            |          Pin14/ Q9 |----->|-----|
//                  |     CNTR_MR  |>-------------------------->| MR       Pin15/Q10 |----->|-----|
//                  |              |                            |          Pin 1/Q11 |----->|-----|----[220R - 1K]----> CNTR_CLK
//                  |              |                            |                    |
//                  |--------------|                            |--------------------|
//
//
//User configuration:
//three key macros need to be configured by the user:
//1. F_CLK: the crystal clock, in hz
//2. F_MSB: the input frequency needed to light up pin1/Q11 on HC4040/4020
//3. PS_TMR:prescaler setting on PIC's TMR0
//
//the following must be true, when picking the three macros:
//1. F_CLK >= 4Mhz, and fully divisible by 1000000ul. ie. it must in whole numbers of Mhz. 12Mhz, 16Mhz and 20Mhz recommended
//2. F_MSB: recommended in 2's power: 3.2Mhz / 6.4Mhz for two digit math, and 5.12Mhz in three digit math
//3. PS_TMR: must be valid prescaler settings: 2/4/8/16/32/64/128/256
//4. (F_CLK / 1000000ul / 4) * (1000ul * 2048) / (F_MSB / 1000ul) must be 1) a whole number, 2) divisible by PS_TMR, and 3) the division is less than or equal to 25.
//
//suggested values:
//F_CLK = 20Mhz, F_MSB = 6.4Mhz, PS_TMR = 64 -> tmr_prx1 = 25, top range = 12.8Mhz, resolution = 50Khz
//F_CLK = 20Mhz, F_MSB = 3.2Mhz, PS_TMR =128 -> tmr_prx1 = 25, top range =  6.4Mhz, resolution = 25Khz
//F_CLK = 16Mhz, F_MSB = 6.4Mhz, PS_TMR = 64 -> tmr_prx1 = 20, top range = 12.8Mhz, resolution = 50Khz
//F_CLK = 16Mhz, F_MSB = 3.2Mhz, PS_TMR =128 -> tmr_prx1 = 20, top range =  6.4Mhz, resolution = 25Khz
//F_CLK = 12Mhz, F_MSB = 6.4Mhz, PS_TMR = 64 -> tmr_prx1 = 15, top range = 12.8Mhz, resolution = 50Khz
//F_CLK = 12Mhz, F_MSB = 3.2Mhz, PS_TMR =128 -> tmr_prx1 = 15, top range =  6.4Mhz, resolution = 25Khz
//
//the use of HC4020:
// when a HC4020 (14-stage counter) is used, the upper-end of the frequency measurement is extended by 4x.
// wiring remains the same, but only used the highest 8 (or 10) digits

#include "config.h"							//configuration words - HS crystal (20Mhz)
#include "gpio.h"                           //we use gpio functions
#include "delay.h"                          //we use software delays

//hardware configuration
#define F_CLK			12000000ul			//crystal clock, in hz
#define F_MSB			6400000ul			//frequency represented by the highest bit, in hz
#define PS_TMR			64					//prescaler for timebase: valid values are 256/128/64/32/16/8/4/2

//pin assignment to the counter
#define CNTR_PORT		GPIO
#define CNTR_DDR		TRISIO
#define CNTR_CLK		(1<<0)				//HC4040 CLK input enable: output high enables input; output low disables input
#define CNTR_MR			(1<<1)				//HC4040 master reset, active high
#define CNTR_DLY()		NOP16()				//delay, to ensure sufficient width for the MR pulse

//buttons / indicators for low frequency measurement
//#define BTN_PORT		GPIO				//comment out if not used
#define BTN_DDR			TRISIO
#define BTN_PIN			(1<<3)				//10x button: active low
#define LED_PIN			(1<<2)				//10x indicator: active high
//end hardware configuration

//global defines

//global variables
char tmr_pr=0x00;							//tmr0 period
char tmr_prx1;

//isr for tmr0
void interrupt isr(void) {
	T0IF = 0;								//clera the flag
	TMR0 = tmr_pr;							//load the offset
	//IO_FLP(CNTR_DDR, CNTR_CLK);				//flip HC4040 input-pullup or output-low
	IO_FLP(CNTR_PORT, CNTR_CLK);
	if (IO_GET(CNTR_PORT, CNTR_CLK)) {		//if CLK is input, reset the counter
		IO_SET(CNTR_PORT, CNTR_MR);	//set MR
		//ensure adequate pulse width on the MR pin: 20ns - 110ns @ 5Vcc
		CNTR_DLY()
		IO_CLR(CNTR_PORT, CNTR_MR);			//reset MR
	}	
}

//reset frequency meter
void freq_init(void) {
	uint16_t cyc;							//time base / cycles for measurement at 1x, in cycles

	cyc = (uint32_t) (F_CLK / 1000000ul / 4) * (1000ul << 11) / (F_MSB / 1000ul);		//tmr0 period, before prescalers
	
	//reset the gpio pins
	nGPPU = 0;								//0->enable global weak pull-up
	WPU |= CNTR_CLK;						//1->enable pin weak pull-up
	
	//CLK: idles output low
	//MR: idles output high
	IO_CLR(CNTR_PORT, CNTR_CLK); IO_SET(CNTR_PORT, CNTR_MR); IO_OUT(CNTR_DDR, CNTR_CLK | CNTR_MR);	//clk + MR as output, active high

#if defined(BTN_PORT)
	//configure button pin
	WPU |= BTN_PIN;							//1->enable pin weak pull-up
	IO_IN(BTN_DDR, BTN_PIN);				//pin as input
	IO_OUT(BTN_DDR, LED_PIN);				//led as output
#endif
		
	//reset timer0
	//TMR0 = 0;								//reset timer
	T0CS = 0;								//0->count on Fcy
	PSA = 0;								//0->prescaler assignment to tmr0
	//adjust tmr period based on prescaler settings
	switch (PS_TMR) {
		case 256: OPTION_REGbits.PS=7; tmr_prx1 = -(cyc / 256); break;
		case 128: OPTION_REGbits.PS=6; tmr_prx1 = -(cyc / 128); break;
		case  64: OPTION_REGbits.PS=5; tmr_prx1 = -(cyc /  64); break;
		case  32: OPTION_REGbits.PS=4; tmr_prx1 = -(cyc /  32); break;
		case  16: OPTION_REGbits.PS=3; tmr_prx1 = -(cyc /  16); break;
		case   8: OPTION_REGbits.PS=2; tmr_prx1 = -(cyc /   8); break;
		case   4: OPTION_REGbits.PS=1; tmr_prx1 = -(cyc /   4); break;
		case   2: OPTION_REGbits.PS=0; tmr_prx1 = -(cyc /   2); break;
		default: while (1);			//no way out -> fault
	}
	tmr_pr = tmr_prx1;
	
	T0IF = 0;								//0->clear the flag
	T0IE = 1;								//1->enable interrupt
	//TMR0 now running
}
	
int main(void) {
	
	mcu_init();							    //initialize the mcu
	freq_init();							//reset the frequency meter
	ei();									//enable global interrupt -> all measurements are done in the ISR
	while (1) {
#if defined(BTN_PORT)
		//if BTN is low, tmr_pr is 10x of its normal range -> for low frequency measurement, and LED_PIN lights up
		if (IO_GET(BTN_PORT, BTN_PIN)==0) {tmr_pr = 10 * tmr_prx1; IO_SET(BTN_PORT, LED_PIN);}
		else {tmr_pr = tmr_prx1; IO_CLR(BTN_PORT, LED_PIN);}
#endif
	}
}

