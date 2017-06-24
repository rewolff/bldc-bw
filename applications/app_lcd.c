
#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "string.h"
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout

// for sprintf
#include <stdio.h>

// lcd thread
static THD_FUNCTION(lcd_thread, arg);
static THD_WORKING_AREA(lcd_thread_wa, 2048); // 2kb stack for this thread

#define LED_PORT GPIOB
#define LED_PIN  2

#if 0 
// on STM32 CPU
#define NCS_PORT GPIOB
#define NCS_PIN  11
#else
// ON STM32_LM5109
#define NCS_PORT GPIOA
#define NCS_PIN  15
#endif

#define MYSPI SPID1


/*
 * Low speed SPI configuration (140.625kHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
  NULL,
  NCS_PORT,
  NCS_PIN,
  SPI_CR1_BR_2|SPI_CR1_BR_1  // 1.5MHz. 
};


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


int addr = 0x94;


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


#define NLINES 4
#define NCHARS 20
char display[NLINES][24];
char olddisplay[NLINES][24];

void print_at (int x, int y, char *s)
{
  int l;
  if (y >= NLINES) return;
  l = strlen (s);
  if (l > (NCHARS-x))
    l = NCHARS-x;

  memcpy (display[y]+x , s, l);
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


void app_lcd_init(void) 
{
  // Set the UART TX pin as an input with pulldown
  palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLDOWN);
  
  // The led on the bitwizard board. 
  palSetPadMode(LED_PORT, LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  
  palSetPadMode(NCS_PORT, NCS_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  
  palSetPadMode(NCS_PORT, NCS_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  
  palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE( GPIO_AF_SPI1));
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE( GPIO_AF_SPI1));
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE( GPIO_AF_SPI1));
  
  spiStart(&MYSPI, &ls_spicfg); 

  memset (display, ' ', sizeof(display));

  if (!probe (0x82))
    probe (0x94);

  // Start the lcd thread
  chThdCreateStatic(lcd_thread_wa, sizeof(lcd_thread_wa),
		    NORMALPRIO, lcd_thread, NULL);
}

char *fc_to_string (mc_fault_code fault) 
{
  switch (fault) {
  case FAULT_CODE_NONE: return "NONE"; break;
  case FAULT_CODE_OVER_VOLTAGE: return "Vin high"; break;
  case FAULT_CODE_UNDER_VOLTAGE: return "Vin low"; break;
  case FAULT_CODE_DRV8302: return "DRV"; break;
  case FAULT_CODE_ABS_OVER_CURRENT: return "ABS I hi"; break;
  case FAULT_CODE_OVER_TEMP_FET: return "FET hot"; break;
  case FAULT_CODE_OVER_TEMP_MOTOR: return "mot hot"; break;
  default: return "???"; break;
  }
}

extern float app_adc_get_decoded_level(void);


//static char buf[0x20];
 
static THD_FUNCTION(lcd_thread, arg) 
{
  (void)arg;
  //  float rpm;
  float ptot, rpmtot, dutytot, vintot, curtot, batcurtot;
  float dc;
  int t;
  mc_fault_code fc;

  chRegSetThreadName("APP_LCD");
  t=0;
  ptot = rpmtot = dutytot = vintot = curtot = batcurtot = 0;

  for(;;) {
    if (t <99) t++;
    else t=0;
    chThdSleepMilliseconds (10);

    //    rpm = mc_interface_get_rpm() / 7;
    //sprintf (buf, "rpm: %.0f", rpm);
    //setreg8 (0x10, 0);
    //chThdSleepMilliseconds(10);
    //print_text (buf, strlen (buf));

    ptot    += GET_INPUT_VOLTAGE() * mc_interface_get_duty_cycle_now () * mc_interface_get_tot_current();
    rpmtot  += mc_interface_get_rpm();
    dc       = mc_interface_get_duty_cycle_now();
    dutytot += dc;
    vintot  += GET_INPUT_VOLTAGE();
    curtot  += mc_interface_get_tot_current();

    //    if (dc) 
    batcurtot += mc_interface_get_tot_current() * dc;
    fc = mc_interface_get_fault ();

    if ((t==0) || (t == 50)) {
#define INTTIME (50.0 )
      sprintf (&display[0][0], "%.1fkm/h", (double) (rpmtot/500.0/INTTIME));

      sprintf (&display[0][12], "duty:%.0f", (double) (dutytot*100.0/INTTIME));
      sprintf (&display[1][0], "vin: %.2f ", (double) (vintot / INTTIME));

      sprintf (&display[1][12], "thr: %.0f ", (double) (99 * app_adc_get_decoded_level()) );
      // app_adc_get_decoded_level() 
      // app_adc_get_voltage ()


      sprintf (&display[2][0], "I: %.1f/%.1lf", 
	(double) mc_interface_read_reset_avg_input_current (), 
 	(double) mc_interface_read_reset_avg_motor_current ());

      sprintf (&display[2][14], "%.0fkJ", 
		(double) 3.6*(double) mc_interface_get_watt_hours(0) );

      sprintf (&display[3][0], "P: %.0f", (double) (ptot/INTTIME));
      sprintf (&display[3][8], "flt: %s", fc_to_string (fc));

      palTogglePad (GPIOB, 2);
      ptot = rpmtot = dutytot = vintot = curtot = batcurtot = 0;
    }

    // Run this loop at 10Hz
    update_lcd ();
    //    chThdSleepMilliseconds(500);

    // Reset the timeout
    timeout_reset();
  }
}
