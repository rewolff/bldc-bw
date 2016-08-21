
#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "string.h"
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
 
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
  
  // Start the lcd thread
  chThdCreateStatic(lcd_thread_wa, sizeof(lcd_thread_wa),
		    NORMALPRIO, lcd_thread, NULL);
}



static char buf[0x20];
 
static THD_FUNCTION(lcd_thread, arg) 
{
  (void)arg;
  float rpm;

  chRegSetThreadName("APP_LCD");
 
  for(;;) {
    rpm = mc_interface_get_rpm();
    sprintf (buf, "rpm: %.0f", rpm);
    setreg8 (0x10, 0);
    chThdSleepMilliseconds(10);
    print_text (buf, strlen (buf));

    // Run this loop at 10Hz
    chThdSleepMilliseconds(500);

    palTogglePad (GPIOB, 2);
    // Reset the timeout
    timeout_reset();
  }
}
