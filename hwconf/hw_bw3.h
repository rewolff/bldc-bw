/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_BW3_H_
#define HW_BW3_H_

#define HW_NAME "bw3"

// Macros
#define ENABLE_GATE()			palSetPad(GPIOC, 10)
#define DISABLE_GATE()			palClearPad(GPIOC, 10)
#define DCCAL_ON()				palSetPad(GPIOB, 12)
#define DCCAL_OFF()				palClearPad(GPIOB, 12)
#define IS_DRV_FAULT()			(0) // No drv -> no faults.

#define LED_GREEN_ON()				palSetPad(GPIOC, 4)
#define LED_GREEN_OFF()				palClearPad(GPIOC, 4)
#define LED_RED_ON()				palSetPad(GPIOA, 7)
#define LED_RED_OFF()				palClearPad(GPIOA, 7)

//#error "test"

/*
 * ADC Vector
 *
 * 0:	IN0		SENS3
 * 1:	IN1		SENS2
 * 2:	IN2		SENS1
 * 3:	IN5		CURR2
 * 4:	IN6		CURR1
 * 5:	IN3		NC
 * 6:	Vrefint
 * 7:	IN8		ADC_EXT2
 * 8:	IN12	AN_IN
 * 9:	IN4		TEMP_MOSFET
 * 10:	IN9	ADC_EXT
 * 11:	IN10	TEMP_MOTOR
 */

#define HW_ADC_CHANNELS				12
#define HW_ADC_INJ_CHANNELS            2
#define HW_ADC_NBR_CONV				4

// ADC Indexes
#define ADC_IND_SENS1				2
#define ADC_IND_SENS2				1
#define ADC_IND_SENS3				0
#define ADC_IND_CURR1				4
#define ADC_IND_CURR2				3
#define ADC_IND_VIN_SENS			5
#define ADC_IND_EXT				10
#define ADC_IND_EXT2				7
#define ADC_IND_TEMP_MOS			11
#define ADC_IND_TEMP_MOTOR			9
#define ADC_IND_VREFINT				6

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
//#define V_REG				3.3
#endif
#ifndef VIN_R1
#define VIN_R1				133000.0
#endif
#ifndef VIN_R2
#define VIN_R2				2200.0
#endif


// BW3 has an ACS758. 
// The ACS758-200B has a sensitivity of 10mV/A, or equivalent to 1mOhm and
// then  multiplied by 10. 
// However, we resistor-divide the value by 10/(10+4.7) = 0.6803. 
// so we claim the AMP_GAIN is 6.8 instead of 10. 

// Latest news: Power the '758 with 3.3V and all your troubles are gone!
// The sensitivity is specified as 10mV/A, but should read: 0.2%VCC / A. This
// works out to 6.6mV/A nominally at 3.3V nominally. 

#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN	6.6
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES	0.001
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()	((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// Voltage on ADC channel
#define ADC_VOLTS(ch)		((float)ADC_Value[ch] / 4095.0 * V_REG)

// NTC Termistors
//#define NTC_RES(adc_val)	(10000.0 / ((4096.0 / (float)adc_val) - 1.0))
#define NTC_RES(adc_val)	((4095.0 * 10000.0) / adc_val - 10000.0)
//#define NTC_TEMP(adc_ind)	 ((float)(ADC_Value[11])/10.0)
#define NTC_TEMP(adc_ind)	 (((float)(ADC_Value[ADC_IND_TEMP_MOS]) * 3.3/4096.0 - 0.5)/0.010)
//(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15)

#define NTC_TEMP_motor(adc_ind)	 (((float)(ADC_Value[ADC_IND_TEMP_MOTOR]) * 3.3/4096.0 - 0.5)/0.010)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE	0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE	0
#endif

// Number of servo outputs
#define HW_SERVO_NUM		2

// UART Peripheral
#define HW_UART_DEV			UARTD6
#define HW_UART_GPIO_AF		GPIO_AF_USART6
#define HW_UART_TX_PORT		GPIOC
#define HW_UART_TX_PIN		6
#define HW_UART_RX_PORT		GPIOC
#define HW_UART_RX_PIN		7

// ICU Peripheral for servo decoding
#define HW_ICU_DEV			ICUD3
#define HW_ICU_CHANNEL		ICU_CHANNEL_2
#define HW_ICU_GPIO_AF		GPIO_AF_TIM3
#define HW_ICU_GPIO			GPIOB
#define HW_ICU_PIN			5

// I2C Peripheral
#define HW_I2C_DEV			I2CD2
#define HW_I2C_GPIO_AF		GPIO_AF_I2C2
#define HW_I2C_SCL_PORT		GPIOB
#define HW_I2C_SCL_PIN		10
#define HW_I2C_SDA_PORT		GPIOB
#define HW_I2C_SDA_PIN		11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOB
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOB
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		11
#define HW_ENC_TIM				TIM4
#define HW_ENC_TIM_AF			GPIO_AF_TIM4
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource11
#define HW_ENC_EXTI_CH			EXTI15_10_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line11
#define HW_ENC_EXTI_ISR_VEC		EXTI15_10_IRQHandler
#define HW_ENC_TIM_ISR_CH               TIM4_IRQn
#define HW_ENC_TIM_ISR_VEC              TIM4_IRQHandler


// NRF pins
#define NRF_PORT_CSN	GPIOB
#define NRF_PIN_CSN		11
#define NRF_PORT_SCK	GPIOC
#define NRF_PIN_SCK		5
#define NRF_PORT_MOSI	GPIOB
#define NRF_PIN_MOSI	10
#define NRF_PORT_MISO	GPIOB
#define NRF_PIN_MISO	1

// Measurement macros
#define ADC_V_L1					ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2					ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3					ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO					(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()				palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()				palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()				palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)


#define DEBUG_PORT GPIOB
#define DEBUG_PIN  6

#undef MCPWM_DEAD_TIME_CYCLES 
#define MCPWM_DEAD_TIME_CYCLES                  80              // Dead time

// Setting limits
#define HW_LIM_CURRENT                 -120.0, 120.0
#define HW_LIM_CURRENT_IN              -100.0, 100.0
#define HW_LIM_CURRENT_ABS             0.0, 180.0
#define HW_LIM_VIN                             6.0, 190.0
#define HW_LIM_ERPM                            -200e3, 200e3
#define HW_LIM_DUTY_MIN                        0.0, 0.1
#define HW_LIM_DUTY_MAX                        0.0, 0.95
#define HW_LIM_TEMP_FET                        -60.0, 100.0


#endif /* HW_BW_H_ */
