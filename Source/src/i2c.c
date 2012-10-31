#include "mb96395rw.h"
#include "i2c.h"

void I2C_Init(void)
{
  ICCR0_EN  = 0;              // stop I2C interface
  ICCR0_CS4 = 1;              // CS4..0 : set prescaler
  ICCR0_CS3 = 1;
  ICCR0_CS2 = 1;
  ICCR0_CS1 = 1;
  ICCR0_CS0 = 1;
  ICCR0_EN  = 1;              // enable I2C interface
  
  IDAR0 = 0;                  // clear data register
  
  IBCR0_BER  = 0;             // clear bus error interrupt flag
  IBCR0_BEIE = 0;             // bus error interrupt disabled
  IBCR0_ACK  = 0;             // no acknowledge generated
  IBCR0_GCAA = 0;             // no call acknowledge is generated
  IBCR0_INTE = 0;             // disable interrupt
  IBCR0_INT  = 0;             // clear transfer interrupt request flag
  
  PIER04_IE4 = 1;             // SDA0 pin to input (QFP-100)
  PIER04_IE5 = 1;             // SCL0 pin to input (QFP-100)
}

void I2C_Acknowlegde()
{
  while(IBSR0_LRB == 1);      // no anwser from slave, program stucks here
                              // a timeout mechanism should be implemented here
}

void I2C_Start(unsigned char slave_address)
{
  do
  {
    IBCR0_BER = 0;                // clear bus error interrupt flag
    ICCR0_EN  = 1;                // enable I2C interface
    IDAR0 = slave_address;        // slave_address is sent out with start condition
    
    IBCR0_MSS = 1;                // set master mode and set start condition
    IBCR0_INT = 0;                // clear transfer end interrupt flag
    
    while(IBCR0_INT == 0);        // look if transfer is in process
  }
  while (IBCR0_BER == 1);         // retry if Bus-Error detected
  
  while(IBSR0_LRB == 1)           // no acknowledge means device not ready
  {                               // maybe last write cycle not ended yet
    IBCR0_SCC = 1;                // try restart (= continue)
    while (IBCR0_INT == 0);       // wait that transfer is finished
  }
}

void I2C_Continue(unsigned char slave_address)
{
  IDAR0 = slave_address;          // slave_address is sent out with start condition
  IBCR0_SCC = 1;                  // restart (= continue)
  while (IBCR0_INT == 0);         // wait that transfer is finished
}

void I2C_Stop(void)
{
  while (IBCR0_INT == 0);         // wait that transfer is finished
  IBCR0_MSS = 0;                  // change to slave and release stop condition
  IBCR0_INT = 0;                  // clear transfer end interrupt flag
  while (IBSR0_BB);               // wait till bus free
}

void I2C_Write(unsigned char value)
{
  IDAR0 = value;                  // load data or address in to register
  IBCR0_INT = 0;                  // clear transfer end intrerupt flag
  while (IBCR0_INT == 0);         // look if transfer is in process
  I2C_Acknowlegde();              // wait for Acknowledge
}

unsigned char I2C_Read(void)
{
  IBCR0_ACK = 1;                  // no acknowledge has to be send
  IBCR0_INT = 0;                  // clear transfer end interrupt flag
  while (IBCR0_INT == 0);         // wait that transfer is finished
  return(IDAR0);                  // read received data out
}

unsigned char I2C_LastRead(void)
{
  IBCR0_ACK = 0;                  // acknowledge has to be sent after last byte
  IBCR0_INT = 0;                  // clear transfer end interrupt flag
  while(IBCR0_INT == 0);          // wait that transfer is finished
  return(IDAR0);                  // read received data out
}