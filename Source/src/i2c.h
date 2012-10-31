#ifndef __I2C_H
#define __I2C_H

#define WRITE   0x00
#define READ    0x01

void I2C_Init(void);
void I2C_Start(unsigned char slave_address);
void I2C_Continue(unsigned char slave_address);
void I2C_Stop(void);
void I2C_Write(unsigned char value);
unsigned char I2C_Read(void);
unsigned char I2C_LastRead(void);

#endif // __I2C_H