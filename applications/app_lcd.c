
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

#if 1 
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


int addr = 0x82;


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

  // Start the lcd thread
  chThdCreateStatic(lcd_thread_wa, sizeof(lcd_thread_wa),
		    NORMALPRIO, lcd_thread, NULL);
}



//static char buf[0x20];
 
static THD_FUNCTION(lcd_thread, arg) 
{
  (void)arg;
  //  float rpm;
  double ptot, rpmtot, dutytot, vintot, curtot;
  int t;

  chRegSetThreadName("APP_LCD");
  t=0;
  ptot = rpmtot = dutytot = vintot = curtot = 0;

  for(;;) {
    if (t <99) t++;
    else t=0;
    chThdSleepMilliseconds (10);

    //    rpm = mc_interface_get_rpm() / 7;
    //sprintf (buf, "rpm: %.0f", rpm);
    //setreg8 (0x10, 0);
    //chThdSleepMilliseconds(10);
    //print_text (buf, strlen (buf));

    ptot    += (double) GET_INPUT_VOLTAGE() * (double) mc_interface_get_duty_cycle_now () * (double) mc_interface_get_tot_current();
    rpmtot  += (double) mc_interface_get_rpm();
    dutytot += (double) mc_interface_get_duty_cycle_now();
    vintot  += (double) GET_INPUT_VOLTAGE();
    curtot  += (double) mc_interface_get_tot_current();

// use  mc_interface_read_reset_avg_input_current
// as the function to get the input current. Should be more accurate than what
// I whipped up here. 


    if ((t==0) || (t == 50)) {
#define INTTIME ((double)50.0 )
      sprintf (&display[0][0], "rpm: %.0f", (double) rpmtot/(double)7.0/INTTIME);
      sprintf (&display[0][12], "duty: %.0f", (double) dutytot*(double)100.0/INTTIME);
      sprintf (&display[1][0], "vin: %.2f", (double) vintot / INTTIME);
      sprintf (&display[2][0], "current: %.1f", (double) curtot/INTTIME);
      sprintf (&display[3][0], "power: %.1f", (double) ptot/INTTIME);
      palTogglePad (GPIOB, 2);
      ptot = rpmtot = dutytot = vintot = curtot = 0;
    }

    // Run this loop at 10Hz
    update_lcd ();
    //    chThdSleepMilliseconds(500);

    // Reset the timeout
    timeout_reset();
  }
}
