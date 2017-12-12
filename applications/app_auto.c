/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "packet.h"
#include "commands.h"
#include "mc_interface.h" // Motor control functions

#include <string.h>

// for sprintf
#include <stdio.h>




#define LED_PORT GPIOB
#define LED_PIN  2


#define MYSPI SPID1

#if 0 
// on STM32 CPU
#define NCS_PORT GPIOB
#define NCS_PIN  11
#else
// ON STM32_LM5109
#define NCS_PORT GPIOA
#define NCS_PIN  15
#endif


/*
 * Low speed SPI configuration (140.625kHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
  NULL,
  NCS_PORT,
  NCS_PIN,
  SPI_CR1_BR_2|SPI_CR1_BR_1  // 1.5MHz. 
};


#define NLINES 4
#define NCHARS 20
char display[NLINES][24];
char olddisplay[NLINES][24];
int addr = 0x94;


#if 0

// Settings
#define BAUDRATE					115200
#define PACKET_HANDLER				1
#define SERIAL_RX_BUFFER_SIZE		1024

#endif
// Threads
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);

static volatile bool is_running = false;


#if 0
static thread_t *process_tp = 0;


// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;

// Private functions
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);

#endif


void delay_us (int n)
{
  n *= 36;
  while (n--) 
    __asm__("  nop\n");

}




int delay=15;

void spitx (char *buf, int len)
{
  int i; 
  spiSelect(&MYSPI);                  /* Slave Select assertion.          */
  delay_us (15);
  for (i=0;i<len;i++) {
    spiPolledExchange(&MYSPI, buf[i]);          /* Atomic transfer operations.      */
    delay_us (delay);
  }
  delay_us (10);
   //    spiExchange(&SPID2, 1, buf+i, buf+i);          /* Atomic transfer operations.      */
  spiUnselect(&MYSPI);           
}


char debug_buf[0x10];

void spitxrx (char *buf, int len)
{
  int i; 
  spiSelect(&MYSPI);                  /* Slave Select assertion.          */
  delay_us (15);
  for (i=0;i<len;i++) {
    buf[i] = spiPolledExchange(&MYSPI, buf[i]);          /* Atomic transfer operations.      */
    delay_us (delay);
  }
  delay_us (10);
   //    spiExchange(&SPID2, 1, buf+i, buf+i);          /* Atomic transfer operations.      */
  memcpy (debug_buf+1, buf, len);
  debug_buf[0] = len;
  spiUnselect(&MYSPI);           
}



void print_text (char *buf, int len)
{
  char sbuf[0x22];

  sbuf[0] = addr;
  sbuf[1] = 0;
  memcpy (sbuf+2, buf, len);
  spitx (sbuf, len+2);
}


void setreg8 (int reg, int val)
{
  char sbuf[0x4];
  sbuf[0] = addr;
  sbuf[1] = reg;
  sbuf[2] = val;
  spitx (sbuf, 3);
}



int getreg8 (int reg)
{
  char sbuf[0x4];
  sbuf[0] = addr + 1;
  sbuf[1] = reg;
  sbuf[2] = 0;
  spitxrx (sbuf, 3);
  return sbuf[2];
}


#if 0

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}



/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) 
{
  (void)uartp;
  serial_rx_buffer[serial_rx_write_pos++] = c;

  if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
    serial_rx_write_pos = 0;
  }

  chSysLockFromISR();
  chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};

static void process_packet(unsigned char *data, unsigned int len) 
{
  commands_set_send_func(send_packet_wrapper);
  commands_process_packet(data, len);
}

static void send_packet_wrapper(unsigned char *data, unsigned int len) 
{
  packet_send_packet(data, len, PACKET_HANDLER);
}

static void send_packet(unsigned char *data, unsigned int len) 
{
  // Wait for the previous transmission to finish.
  while (HW_UART_DEV.txstate == UART_TX_ACTIVE) {
    chThdSleep(1);
  }

  // Copy this data to a new buffer in case the provided one is re-used
  // after this function returns.
  static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
  memcpy(buffer, data, len);

  uartStartSend(&HW_UART_DEV, len, buffer);
}

#endif


void update_lcd (void)
{
  static int nn;
  static int wait;
  static int l;

  int j, k;

  if (wait > 0) {
    wait--;
    return;
  }
  if (nn++ > 500) {
    setreg8 (0x10, 0);
    memset (olddisplay, ' ', sizeof (olddisplay));
    nn=0;
    wait = 1;
    return;
  }

  for (j=0;j<NCHARS;j++) 
    if (display[l][j] == 0)
      display[l][j] = ' ';

  for (j=0;j<NCHARS;j++) 
    if (olddisplay[l][j] != display[l][j]) break;
  if (j < NCHARS) {
    for (k=NCHARS-1;k>j;k--)
      if (olddisplay[l][k] != display[l][k]) break;
    setreg8 (0x11, (l << 5) + j);
    chThdSleep(1);
    print_text (display[l]+j, k-j+1);
    memcpy (olddisplay[l]+j, display[l]+j, k-j+1);
    wait = 1;
  }
  if (l < (NLINES-1)) l++;
  else l = 0;
}



int probe (int testaddr)
{
  int rv;

  addr = testaddr;
  delay_us (50);
  rv = getreg8 (0x12);
  if ((rv == 0xff) || (rv == 0x00))
    return 0;
  
  return 1;
}




void app_custom_start(void) 
{
  if (!is_running) {
    chThdCreateStatic(packet_process_thread_wa, sizeof(packet_process_thread_wa),
		      NORMALPRIO, packet_process_thread, NULL);
    is_running = true;
  }
  
  palSetPadMode(NCS_PORT, NCS_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  
  palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE( GPIO_AF_SPI1));
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE( GPIO_AF_SPI1));
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE( GPIO_AF_SPI1));
  
  spiStart(&MYSPI, &ls_spicfg); 

  memset (display, ' ', sizeof(display));

  if (!probe (0x82))
    probe (0x94);

}




char *fc_to_string (mc_fault_code fault) 
{
  switch (fault) {
  case FAULT_CODE_NONE: return "NONE"; break;
  case FAULT_CODE_OVER_VOLTAGE: return "Vin high"; break;
  case FAULT_CODE_UNDER_VOLTAGE: return "Vin low"; break;
  case FAULT_CODE_DRV: return "DRV"; break;
  case FAULT_CODE_ABS_OVER_CURRENT: return "ABS I hi"; break;
  case FAULT_CODE_OVER_TEMP_FET: return "FET hot"; break;
  case FAULT_CODE_OVER_TEMP_MOTOR: return "mot hot"; break;
  default: return "???"; break;
  }
}



void app_custom_stop(void) 
{

  // Notice that the processing thread is kept running in case this call is made from it.
}


void app_custom_configure(app_configuration *conf) 
{
  (void) conf;
}



static THD_FUNCTION(packet_process_thread, arg) 
{ 
  (void)arg;
  //  float rpm;
  float rpmtot, dutytot, vintot;
  float dc, ic;
  int t;
  mc_fault_code fc;

  is_running = true;
  chRegSetThreadName("APP_LCD");
  t=0;
  rpmtot = dutytot = vintot = 0;
  palSetPadMode (GPIOB, 2, PAL_MODE_OUTPUT_PUSHPULL);
  for(;;) {

    if (t < 99) t++;
    else t=0;
    chThdSleepMilliseconds (10);

#if 0 
    if (stop_now) {
      is_running = false;
      return;
    }
#endif

    dc       = mc_interface_get_duty_cycle_now();

    rpmtot  += mc_interface_get_rpm();
    dutytot += dc;
    vintot  += GET_INPUT_VOLTAGE();

    if ((t==0) || (t == 50)) {
#define INTTIME (50.0 )
      //sprintf (&display[0][0], "%.1fkm/h", (double) (rpmtot/500.0/INTTIME));
      sprintf (&display[0][0], "%.0fRPM", (double) (rpmtot/6/INTTIME));

      sprintf (&display[0][12], "duty:%.0f", (double) (dutytot*100.0/INTTIME));
      sprintf (&display[1][0], "vin: %.2f ", (double) (vintot / INTTIME));

      sprintf (&display[1][12], "thr: %.0f ", (double) (99 * app_adc_get_decoded_level()) );


      // 3rd line: left side: battery and motor current
      sprintf (&display[2][0], "I: %.1f/%.1lf", 
	       (double) mc_interface_read_reset_avg_input_current (), 
	       (double) mc_interface_read_reset_avg_motor_current ());

      // 3rd line: right side: energy consumed. 
      sprintf (&display[2][14], "%.0fkJ", 
		(double) 3.6*(double) mc_interface_get_watt_hours(0) );

      // bottom left: power. 
      sprintf (&display[3][0], "P: %.0f", (double) (ic * vintot / INTTIME));

      // bottom line: temp/fault. 
      fc = mc_interface_get_fault ();
      if (fc == FAULT_CODE_NONE) {
	sprintf (&display[3][8], "t: %.0fC", 
		 (double) ((((float)(ADC_Value[11]) * 3.3/4096.0 - 0.5)/0.010)));
      } else {
	sprintf (&display[3][8], "flt: %s", fc_to_string (fc));
      }

      // status, "we're running" led. 
      palTogglePad (GPIOB, 2);
      rpmtot = dutytot = vintot = 0;
    }

    // Run this loop at 100Hz
    update_lcd ();

    // Reset the timeout
    //   timeout_reset();
  }

}
