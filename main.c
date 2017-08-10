
//******************************************************************************
//  MSP430G2xx3 Demo - USCI_A0 IrDA External Loopback Test, 8MHz SMCLK
//
//  Description: This example transmits bytes through the USCI module
//  configured for IrDA mode, and receives them using an external loopback
//  connection. The transfered sequence is 00h, 01h, 02h, ..., ffh. The
//  received bytes are also stored in memory starting at address RxData.
//  In the case of an RX error the LED is lighted and program execution stops.
//  An external loopback connection has been used as it allows for the
//  connection of a scope to monitor the communication, which is not possible
//  when using the internal loopback.
//  ACLK = n/a, MCLK = SMCLK = BRCLK = CALxxx_8MHZ = 8MHz
//
//              MSP430G2xx3
//            -----------------
//        /|\|              XIN|-
//         | |                 |
//         --|RST          XOUT|-
//           |                 |
//           |     P1.1/UCA0RXD|--+   external
//           |     P1.2/UCA0TXD|--+   loopback connection
//           |                 |
//           |                 |
//           |             P1.0|--->  LED
//           |                 |
//
//  D. Dang
//  Texas Instruments Inc.
//  February 2011
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
#include <msp430.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

//PINOUT
#define ADDR_SEL    BIT0//P1
#define RXD    BIT1//P1
#define TXD    BIT2//P1
#define SD    BIT3//P1
#define SCL    BIT6//P1
#define SDA    BIT7//P1
#define ERR_LED    BIT4//P3
#define RX_LED    BIT5//P3
#define TX_LED    BIT6//P3
#define IRDA_INT    BIT7//P3



//self registers



//uart irda
volatile unsigned char RxData[256];
uint8_t RxByte;
uint8_t TxByte;
volatile uint16_t i;







//i2c below




#define NUM_BYTES  2                        // How many bytes?
//**** Please note this value needs to be the same as NUM_BYTES_RX in the
//     associated master code. This definition lets the slave know when to
//     switch from RX interrupt sources to TX interrupt sources. This is
//     important since the interrupt vectors are shared by TX and RX flags.

unsigned char *PTxData;                     // Pointer to TX data
unsigned char *PRxData;                     // Pointer to RX data
volatile unsigned char RxBuffer[128];       // Allocate 128 byte of RAM
char SLV_Data = 0x11;
volatile unsigned char TXByteCtr, RXByteCtr, RX = 0;
volatile unsigned char RxBuffer[128];       // Allocate 128 byte of RAM


void USCI_SLAVE_SETUP(void);
void Setup_RX(void);
void Receive(void);




///i2c end






static void clocks_init(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    if (CALBC1_1MHZ==0xFF)                    // If calibration constant erased
    {
      while(1);                               // do not load, trap CPU!!
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;                     // Load 1MHz constants
}

static void gpio_init(void)
{

    P1OUT  &= ~SD;
    P1DIR  |= SD;

    P3OUT &= ~ERR_LED + RX_LED + TX_LED;                           // Clear
    P3DIR |= ERR_LED + RX_LED + TX_LED;                            //  output

    P1SEL |= RXD + TXD + SCL + SDA;                     // Use P3.4/P3.5 for USCI_A0
    P1SEL2 |= RXD + TXD + SCL + SDA;                  // Both P1SEL & P1SEL2 bits must be set for UCA functions

}

static void IRDA_UART_init(void)
{
    UCA0CTL1 |= UCSWRST;                      // Set SW Reset
    UCA0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
    UCA0BR0 = 10;                             // 1MHz/10=100KHz
    UCA0BR1 = 0;
    UCA0MCTL = UCBRF_1 + UCOS16;              // Set 1st stage modulator to 1
                                              // 16-times oversampling mode
    UCA0IRTCTL = UCIRTXPL2 + UCIRTXPL0 + UCIRTXCLK + UCIREN;
                                              // Pulse length = 6 half clock cyc
                                              // Enable BITCLK16, IrDA enc/dec
    UCA0CTL1 &= ~UCSWRST;                     // Resume operation



    TxByte = 0x00;                            // TX data and pointer, 8-bit

}

void I2C_Receive(void){
    PRxData = (unsigned char *)RxBuffer;    // Start of RX buffer
    RXByteCtr = 0;                          // Clear RX byte count
    TXByteCtr = 0;
    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
                                            // Remain in LPM0 until master
                                            // finishes TX
}


static void I2C_init(void)
{
    __disable_interrupt();
    RX = 1;
    IE2 &= ~UCB0TXIE;                         // Disable TX interrupt

        UCB0CTL1 |= UCSWRST;                      // Enable SW reset
        UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
        //CHECK ADDRESS PIN
        if (ADDR_SEL & P1IN)
        {
            UCB0I2COA = 0x91;                         // Own Address
        }
        else
        {
            UCB0I2COA = 0x90;                         // Own Address
        }
        UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation

        UCB0I2CIE |= UCSTPIE + UCSTTIE;           // Enable STT and STP interrupt
        IE2 |= UCB0RXIE;                          // Enable RX interrupt



}



int main(void)
{
  clocks_init();
  gpio_init();
  I2C_init();
  IRDA_UART_init();

  while(1)
  {
      I2C_Receive();
  }

//uart irda code below
  while (1)
   {
     for (i = 1000; i; i--);                 // Small delay
     while (!(IFG2 & UCA0TXIFG));            // USCI_A0 TX buffer ready?
     UCA0TXBUF = TxByte;                     // TX character

     __disable_interrupt();
     IE2 |= UCA0RXIE;                        // Enable RX int
     __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts

     RxData[TxByte] = RxByte;                // Store RXed character in RAM
     if (TxByte != RxByte)                   // RX OK?
     {
       P1OUT |= BIT0;                        // LED P1.0 on
       while (1);                            // Trap PC here
     }
     TxByte++;                               // Next character to TX
   }


}


//uart irda stuff below

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)

{
  RxByte = UCA0RXBUF;                       // Get RXed character
  IE2 &= ~UCA0RXIE;                         // Disable RX int
  __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
}



///i2c stuff below


//------------------------------------------------------------------------------
// The USCI_B0 data ISR is used to move data from MSP430 memory to the
// I2C master. PTxData points to the next byte to be transmitted, and TXByteCtr
// keeps track of the number of bytes transmitted.
//------------------------------------------------------------------------------

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)

{
    if(IFG2&UCB0TXIFG)                        // Check for I2C TX
    {
      if(RX == 0){ UCB0TXBUF = SLV_Data++;      // Transmit data at address PTxData
          TXByteCtr++;                              // Increment TX byte counter
      }
      if(RX == 1)
      {
          *PRxData++ = UCB0RXBUF;       // Move RX data to address PRxData
          RXByteCtr++;                              // Increment RX byte count
          if(RXByteCtr >= NUM_BYTES){               // Received enough bytes to switch
              RX = 0;                                   // to TX?
              IE2 &= ~UCB0RXIE;
              IE2 |= UCB0TXIE;
              RXByteCtr = 0;
          }
      }
      IFG2 &= ~UCB0TXIFG;                   // Clear USCI_B0 TX int flag
      __bic_SR_register_on_exit(CPUOFF + GIE);      // Exit LPM0
      IE2 |= UCA0TXIE;                      // Enable USCI_A0 TX interrupt
    }


    if((IFG2&UCA0TXIFG)&&(IE2&UCA0TXIE))      // Check for UART TX
    {
///      UCA0TXBUF = UART_Data++;                // Load TX buffer
      IE2 &= ~UCA0TXIE;                       // Clear USCI_A0 TX interrupt
    }

}

//------------------------------------------------------------------------------
// The USCI_B0 state ISR is used to wake up the CPU from LPM0 in order to do
// processing in the main program after data has been transmitted. LPM0 is
// only exit in case of a (re-)start or stop condition when actual data
// was transmitted.
//------------------------------------------------------------------------------

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)

{
  if(RX == 0){ UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);       // Clear interrupt flags
  if (TXByteCtr)                            // Check TX byte counter
   __bic_SR_register_on_exit(CPUOFF);       // Exit LPM0 if data was
}                                           // transmitted
  if(RX == 1){UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);       // Clear interrupt flags
}
}


???



//new combined USCIAB0RX_ISR

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
    if(IFG2&UCB0RXIFG)                        // Check for I2C TX
    {

    }




}
