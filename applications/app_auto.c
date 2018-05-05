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
//#include "packet.h"
//#include "commands.h"
#include "timeout.h"
#include "mc_interface.h" // Motor control functions

#include <string.h>

// for sprintf
#include <stdio.h>



/**********************************************************
                          CONFIG
 **********************************************************/

#define ERPM_PER_RPM 6




// For testing: 433 is reached on 30V. 
// With a KV of 14.57 at 160V we get 2333 RPM. 
#define MAXRPM 2333
#define CHARGE_RAMP_START_RPM (610)
#define CHARGE_RAMP_STOP_RPM  (650)

#define CHARGE_MOTORCURRENT -40.0
#define CHARGE_VOLTAGE_RAMP_START 166.0
#define CHARGE_VOLTAGE_RAMP_STOP  170.0

#define MAXERPM                (MAXRPM*ERPM_PER_RPM)
#define CHARGE_RAMP_START_ERPM (CHARGE_RAMP_START_RPM*ERPM_PER_RPM)
#define CHARGE_RAMP_STOP_ERPM  (CHARGE_RAMP_STOP_RPM *ERPM_PER_RPM)


#define START_CURRENT 150.0
#define MAX_ELEC_CURRENT 150.0


// Enums. 


typedef enum { GS_STARTUP, GS_READY, GS_ELEKTRISCH_RIJDEN, GS_STOPPING, GS_STARTING} GeneralState;
typedef enum { R_MOTOR, R_PRECHARGE, R_MAINPOWER, R_12V} relay_indx;
typedef enum { CHGSTATE_OFF, CHGSTATE_CHARGE} ChargeState;

//typedef enum {RELSTAT_OFF, RELSTAT_WAIT, RELSTAT_ON} RelayStatus;

GeneralState general_state;
RelayStatus relay_state;
ChargeState charge_state;




// SPI stuff


#define LED_PORT GPIOB
#define LED_PIN  2

#define MYSPI SPID1

#define AUTO_INTERFACE_SPI_ADDRESS 0x7e

#if 0 
// on STM32 CPU
#define NCS_PORT GPIOB
#define NCS_PIN  11
#else
// ON STM32_LM5109
// ON auto_interface. 
#define NCS_PORT GPIOA
#define NCS_PIN  15
#endif

/*
 * SPI configuration (1.5MHz, CPHA=0, CPOL=0, MSb first).
 * Tested 1.3MHz, should've worked, but doesn't. Too bad. 
 */
static const SPIConfig ls_spicfg = {
  NULL,
  NCS_PORT,
  NCS_PIN,
  6 * SPI_CR1_BR_0  // pclk runs at 84MHz, this selects (84/2) /2^6 = 0.66Mhz
//SPI_CR1_BR_2|SPI_CR1_BR_1  // 1.5MHz. 
};


#define NLINES 4
#define NCHARS 20
char display[NLINES][24];
char olddisplay[NLINES][24];
int addr = 0x94;
int can_send_next;

// Threads
static THD_FUNCTION(auto_main_thread, arg);
static THD_WORKING_AREA(auto_main_thread_wa, 4096);

static volatile bool is_running = false;


// Busy wait for short periods. 
void delay_us (int n)
{
  n *= 36;
  while (n--) 
    __asm__("  nop\n");
}


int delay=15;

static void spitx (char *buf, int len)
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



#ifdef DEBUG_SPITXRX
char debug_buf[0x10];
#endif

static void spitxrx (char *buf, int len)
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

#ifdef DEBUG_SPITXRX
  memcpy (debug_buf+1, buf, len);
  debug_buf[0] = len;
#endif

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
    chThdCreateStatic(auto_main_thread_wa, sizeof(auto_main_thread_wa),
		      NORMALPRIO, auto_main_thread, NULL);
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


void set_relay (int relaynum, int val)
{
  char sbuf[8];

  sbuf[0] = AUTO_INTERFACE_SPI_ADDRESS;     // The address.
  sbuf[1] = 10;       // CMD_SETPWM
  sbuf[2] = relaynum; // relay number
  sbuf[3] = val >> 8; // the value
  sbuf[4] = val & 0xff;
  spitx (sbuf, 5);
}


#if 1
unsigned short crc16a(const unsigned char* data_p, unsigned char length)
{
  unsigned char x;
  unsigned short crc = 0xFFFF;

  while (length--) {
    x = crc >> 8 ^ *data_p++;
    x ^= x>>4;
    crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
  }
  return crc;
}
#endif


enum dig_idx {I_DUMMY, I_START, I_ELEC};

unsigned char digin[8];
unsigned int  ain[8];
int last_input;

int npkts, nserr, nderr;

char sbuf[8];

int got_data;

void get_inputs (int numanalog)
{
  int i;
  unsigned short crc;

  (void) numanalog;
  // XXX todo numanalog toevoegen. 

  sbuf[0] = AUTO_INTERFACE_SPI_ADDRESS+1;     // The address + READ
  sbuf[1] = 11;       // CMD_GET_INPUTS
  //  for (i=2;i<8;i++) sbuf[i]=i+1;

  // 2 header 1 digital data numanalog analog channels and 2 bytes CRC.
  spitxrx (sbuf, 2 + 1 + numanalog*2 + 2);
  
  npkts++;
  crc = crc16a ((unsigned char *)sbuf+2, 3);

  // XXX change 5 3+numanalog*2 
  if ((sbuf [5] != (crc & 0xff)) ||
      (sbuf [6] != (crc >> 8))) {
    nserr++;
    chThdSleepMilliseconds (1);
    sbuf[0] = AUTO_INTERFACE_SPI_ADDRESS+1;     // The address + READ
    sbuf[1] = 11;       // CMD_GET_INPUTS
    //  for (i=2;i<8;i++) sbuf[i]=i+1;
    spitxrx (sbuf, 7);
    crc = crc16a ((unsigned char *)sbuf+2, 3);
    if ((sbuf [5] != (crc & 0xff)) ||
        (sbuf [6] != (crc >> 8))) {
      nderr++;
      got_data = 0;
      return;
    }
  }
  got_data = 1;

  for (i=0;i<8;i++) 
    digin[i] = (sbuf[2]>>i) & 1;
  
  ain[0] = (sbuf[4] << 8) | sbuf[3];
  last_input = ain[0] | (sbuf[2] << 16);

}



//#define DEBUG_CHG

#ifdef DEBUG_CHG
struct dbg {
  int rpm, vin, cur;
};


struct dbg dbg_log[1000];
int dbg_head;

#endif

int my_last_current; // ma


void set_current (float c)
{
  my_last_current = c*1000;
  mc_interface_set_current (c);
}


void handle_motor_relay (float rpm)
{
  static int nhi, nlo;

  if (rpm > MAXERPM) {
    nhi++;
    nlo = 0;
    if (nhi == 3) { // 30 ms
      set_current (0.0); // Should happen immediately. 
      set_relay (R_MOTOR, 0); // this will take some time. 
      relay_state = RELSTAT_OFF;
    }
  } else {
    nlo++;
    nhi = 0;
    if (nlo == 500) { // 5 seconds. 
      set_relay (R_MOTOR, 2000); // this will take some time.
      relay_state = RELSTAT_WAIT; // First "powertime": full blast. 
    }
    if (nlo == 600) { // 6 seconds.
      set_relay (R_MOTOR, 500); // this will take some time.
      relay_state = RELSTAT_ON;
    }
  }
}


static void handle_charging (float rpm, float vin)
{
  float chg_perc;

#ifdef DEBUG_CHG
  if (++dbg_head >= 1000) dbg_head = 0;

  dbg_log[dbg_head].rpm = rpm;
  dbg_log[dbg_head].vin = vin*1000;
  dbg_log[dbg_head].cur = -555;
#endif

  if ((general_state == GS_ELEKTRISCH_RIJDEN) ||
      (general_state == GS_STARTING)) {
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
      set_current (0.0);
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
  set_current (chg_perc * CHARGE_MOTORCURRENT);
  charge_state = CHGSTATE_CHARGE;
}


// 100  ticks per second.
#define TIME_TO_TICK(tt) ((int)(tt*100))


static void handle_startup (void)
{
  static int t;

  switch (t) {
  case TIME_TO_TICK(0.1): set_relay (R_PRECHARGE, 2000);break;
  case TIME_TO_TICK(2.1): set_relay (R_MAINPOWER, 2000);break;
  case TIME_TO_TICK(2.6): set_relay (R_12V,       2000);break;
  case TIME_TO_TICK(3.0): set_relay (R_PRECHARGE,    0);break;
  case TIME_TO_TICK(3.1): set_relay (R_MAINPOWER,  500);break;
  case TIME_TO_TICK(3.19): get_inputs (1);break; // Dummy: first analog value is invalid.
  case TIME_TO_TICK(3.2): general_state = GS_READY;break;
  }
  t++;
}

#define START_RAMP_END   500.0
#define START_RAMP_START 400.0

static void handle_ECU_inputs (int rpm)
{
#if 0
  if ((general_state != GS_READY) &&
      (general_state != GS_ELEKTRISCH_RIJDEN))  return;
#endif
  if (general_state == GS_STARTUP) return;

  get_inputs (1);

  rpm = rpm / ERPM_PER_RPM;

  if (digin[I_START]) {
    general_state = GS_STARTING;
    if (rpm > START_RAMP_END)
      set_current (0.0);
    else if (rpm > START_RAMP_START)
      set_current (START_CURRENT * (START_RAMP_END-rpm)/(START_RAMP_END - START_RAMP_START));
    else 
      set_current (START_CURRENT);
    if (got_data) timeout_reset();
  } else {
    if (general_state == GS_STARTING) {
      general_state = GS_READY;
      set_current (0.0);
    }
  }

  if (digin[I_ELEC]) {
    if (general_state == GS_ELEKTRISCH_RIJDEN) {
      set_current ( MAX_ELEC_CURRENT * ain[0]/ 65536);
      if (got_data) timeout_reset ();
    }
    if ((general_state == GS_READY)  ||
        (general_state == GS_STOPPING))
      general_state = GS_ELEKTRISCH_RIJDEN;
  } else {
    if (general_state == GS_ELEKTRISCH_RIJDEN) {
      general_state = GS_STOPPING;
    } else {
      // do not set the current immediately on first niet-elek.
      if (general_state == GS_STOPPING) {
        set_current (0.0);
        general_state = GS_READY;
      }
    }
  }
}




static int handle_lcd (float rpmtot, float dutytot, float vintot)
{
  static int t;
  float ic;
  mc_fault_code fc;

  if (t < 99) t++;
  else t=0;

  if ((t==0) || (t == 50)) {
#define INTTIME (50.0 )
    //sprintf (&display[0][0], "%.1fkm/h", (double) (rpmtot/500.0/INTTIME));
    sprintf (&display[0][0], "%.0fRPM", (double) (rpmtot/6/INTTIME));

    sprintf (&display[0][12], "duty:%.0f", (double) (dutytot*100.0/INTTIME));
    sprintf (&display[1][0], "vin: %.2f ", (double) (vintot / INTTIME));

    //    sprintf (&display[1][12], "thr: %.0f ", (double) (99 * app_adc_get_decoded_level()) );
    sprintf (&display[1][12], "%d", general_state  );

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
    palTogglePad (LED_PORT, LED_PIN);
    //    rpmtot = dutytot = vintot = 0;
    return 1;
  }

  // Run this loop at 100Hz
  update_lcd ();
  return 0;
}


/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) 
{
  (void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) 
{
  can_send_next = 1;
  (void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) 
{
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
  (void)c;
#if 0 
  serial_rx_buffer[serial_rx_write_pos++] = c;
  
  if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
    serial_rx_write_pos = 0;
  }
  
  chSysLockFromISR();
  chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
#endif
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) 
{
  (void)uartp;
}

#define BAUDRATE 1000000
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


struct packet {
  int som;      // 0x1234567
  systime_t ts; // in 1/10000th of a second.
  int rpm;      // in 0.001 RPM. 
  int duty;     // in .1%
  int vin;      // in mV. 
  int /*mc_fault_code*/ fault;
  uint8_t state[4];// 
  int lastcurrent;
  int lastinput;
  int temp;     // mili Celcius. 
} pkt;


#define MYUART UARTD6

void send_next_logging_packet (float currpm, float dc, float vin, mc_fault_code fc)
{
  uartStopSend(&MYUART);

  pkt.som = 0x1234567;
  pkt.ts = chVTGetSystemTimeX ();
  pkt.rpm = currpm * 1000;
  pkt.duty = dc * 1000;
  pkt.vin = vin * 1000;
  pkt.fault = fc;
  pkt.state[0] = general_state;
  pkt.state[1] = relay_state;
  pkt.state[2] = charge_state;
  pkt.state[3] = 0xaa;
  pkt.lastcurrent = my_last_current;
  pkt.lastinput = last_input;
  pkt.temp =   1000* (((float)(ADC_Value[11]) * 3.3/4096.0 - 0.5)/0.010);

  can_send_next = 0;
  uartStartSend(&MYUART, sizeof (pkt), &pkt);
}



static THD_FUNCTION(auto_main_thread, arg) 
{ 
  (void)arg;
  //  float rpm;
  float rpmtot, dutytot, vintot, currpm, vin;
  float dc;
  mc_fault_code fc;

  is_running = true;
  chRegSetThreadName("APP_LCD");

  rpmtot = dutytot = vintot = 0;

  general_state = GS_STARTUP;

  palSetPadMode (LED_PORT, LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);

  palSetPadMode (GPIOC, 6, PAL_MODE_ALTERNATE (8));
  palSetPadMode (GPIOC, 7, PAL_MODE_ALTERNATE (8));

  uartStart(&MYUART, &uart_cfg);
  can_send_next = 1;


  //uartStartReceive(&MYUART, 16, buffer);

  for(;;) {
    chThdSleepMilliseconds (10);

#if 0 
    if (stop_now) {
      is_running = false;
      return;
    }
#endif

    /* Bookkeeping: Get info from mc interface */

    currpm   = mc_interface_get_rpm();
    rpmtot  += currpm; 

    dc       = mc_interface_get_duty_cycle_now();
    dutytot += dc;

    vin      = GET_INPUT_VOLTAGE();
    vintot  += vin;

    fc = mc_interface_get_fault ();

    if (can_send_next) 
      send_next_logging_packet (currpm, dc, vin, fc);

    handle_startup ();

    handle_motor_relay (currpm);

    if (fc == FAULT_CODE_NONE) {
      handle_charging (currpm, vin);
      handle_ECU_inputs (currpm);
    }
    if (handle_lcd (rpmtot, dutytot, vintot)) { // returns true when time to reset avg/tot
      rpmtot = dutytot = vintot = 0;
    }
  }
}
