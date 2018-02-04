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
  // XXX we can stop the thread. 

  // Notice that the processing thread is kept running in case this call is made from it.
}


void app_custom_configure(app_configuration *conf) 
{
  (void) conf;
}


void set_relay (int val)
{
  char sbuf[8];

  sbuf[0] = 0x7e;  // The address.
  sbuf[1] = 10;    // CMD_SETPWM
  sbuf[2] = 0;     // relay number
  sbuf[3] = val >> 8; // the value
  sbuf[4] = val & 0xff;
  spitx (sbuf, 5);
}


#define ERPM_PER_RPM 6

#define DEBUG_CHG

#ifdef DEBUG_CHG
struct dbg {
  int rpm, vin, cur;
};

struct dbg dbg_log[1000];
int dbg_head;

#endif



// For testing: 433 is reached on 30V. 
#define MAXRPM 433
#define CHARGE_RAMP_START_RPM 310
#define CHARGE_RAMP_STOP_RPM 350

#define CHARGE_MOTORCURRENT -1.0
#define CHARGE_VOLTAGE_RAMP_START 42.0
#define CHARGE_VOLTAGE_RAMP_STOP  44.0


#define MAXERPM                (MAXRPM*ERPM_PER_RPM)
#define CHARGE_RAMP_START_ERPM (CHARGE_RAMP_START_RPM*ERPM_PER_RPM)
#define CHARGE_RAMP_STOP_ERPM  (CHARGE_RAMP_STOP_RPM *ERPM_PER_RPM)


typedef enum {CHGSTATE_OFF, CHGSTATE_CHARGE} ChargeState;


//typedef enum {RELSTAT_OFF, RELSTAT_WAIT, RELSTAT_ON} RelayStatus;
RelayStatus relay_state;
ChargeState charge_state;

void handle_relay (float rpm)
{
  static int nhi, nlo;

  if (rpm > MAXERPM) {
    nhi++;
    nlo = 0;
    if (nhi == 3) { // 30 ms
      mc_interface_set_current (0.0); // Should happen immediately. 
      set_relay (0); // this will take some time. 
      relay_state = RELSTAT_OFF;
    }
  } else {
    nlo++;
    nhi = 0;
    if (nlo == 500) { // 5 seconds. 
      set_relay (2000); // this will take some time.
      relay_state = RELSTAT_WAIT; // First "powertime": full blast. 
    }
    if (nlo == 600) { // 6 seconds.
      set_relay (500); // this will take some time.
      relay_state = RELSTAT_ON;
    }
  }
}



void handle_charging (float rpm, float vin)
{
  float chg_perc;

#ifdef DEBUG_CHG
  if (++dbg_head >= 1000) dbg_head = 0;

  dbg_log[dbg_head].rpm = rpm;
  dbg_log[dbg_head].vin = vin*1000;
  dbg_log[dbg_head].cur = -555;
#endif

  if (overall_state == STATE_ELEKTRISCH_RIJDEN) {
    charge_state = CHGSTATE_OFF;
    return;
  }

  // We can't charge if the relays are not on. 
  if (relay_state != RELSTAT_ON) {
    charge_state = CHGSTATE_OFF;
    return;
  }

  // RPM is too low to charge. or battery is full. 
  if ((rpm < CHARGE_RAMP_START_ERPM) ||
      (vin > CHARGE_VOLTAGE_RAMP_STOP)) {

    // if necessary stop charging. 
    if (charge_state == CHGSTATE_CHARGE) {
#ifdef DEBUG_CHG
      dbg_log[dbg_head].cur = 0;
#endif
      mc_interface_set_current (0.0);
      charge_state = CHGSTATE_OFF;
    }
    return;
  }

  // charging current ramps up from 0 to max from 
  // CHARGE_RAMP_START_ERPM to CHARGE_RAMP_STOP_ERPM . 
  // (This means you won't feel the charge suddenly 
  //  starting once you pass the threshold RPM).

  if (rpm > CHARGE_RAMP_STOP_ERPM) {
    chg_perc = 1.0;
  } else {
    chg_perc = (rpm - CHARGE_RAMP_START_ERPM) / 
      (CHARGE_RAMP_STOP_ERPM - CHARGE_RAMP_START_ERPM);
  }

  // charging current ramps back down from 100% to 0 from
  // CHARGE_VOLTAGE_RAMP_START to CHARGE_VOLTAGE_RAMP_STOP

  if (vin > CHARGE_VOLTAGE_RAMP_START) {
    chg_perc *= (vin - CHARGE_VOLTAGE_RAMP_START) / 
      (CHARGE_VOLTAGE_RAMP_STOP - CHARGE_VOLTAGE_RAMP_START);
  }

#ifdef DEBUG_CHG
  dbg_log[dbg_head].cur = 1000 * (chg_perc * CHARGE_MOTORCURRENT);
#endif
  mc_interface_set_current (chg_perc * CHARGE_MOTORCURRENT);
  charge_state = CHGSTATE_CHARGE;
}



static THD_FUNCTION(packet_process_thread, arg) 
{ 
  (void)arg;
  //  float rpm;
  float rpmtot, dutytot, vintot, currpm, vin;
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

    currpm   = mc_interface_get_rpm();
    rpmtot  += currpm; 

    dc       = mc_interface_get_duty_cycle_now();
    dutytot += dc;

    vin      = GET_INPUT_VOLTAGE();
    vintot  += vin;

    handle_relay (currpm);
    handle_charging (currpm, vin);

    if ((t==0) || (t == 50)) {
#define INTTIME (50.0 )
      //sprintf (&display[0][0], "%.1fkm/h", (double) (rpmtot/500.0/INTTIME));
      sprintf (&display[0][0], "%.0fRPM", (double) (rpmtot/6/INTTIME));

      sprintf (&display[0][12], "duty:%.0f", (double) (dutytot*100.0/INTTIME));
      sprintf (&display[1][0], "vin: %.2f ", (double) (vintot / INTTIME));

      sprintf (&display[1][12], "thr: %.0f ", (double) (99 * app_adc_get_decoded_level()) );

      ic =  mc_interface_read_reset_avg_input_current ();
      // 3rd line: left side: battery and motor current
      sprintf (&display[2][0], "I: %.1f/%.1lf", 
	       (double) ic,
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
