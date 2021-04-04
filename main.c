//  Lectura del IMU y envio de datos por USART

#include <stdbool.h>
#include <stdint.h>

#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
volatile static uint32_t adcResult = 0;

#define MPU_ADDRESS 0x68
#define MPU_ACC_XH 0x3B

void Timer0A_Handler(void){
        TIMER0_ICR_R |= (1<<0); 
        GPIO_PORTF_DATA_R ^= (1<<2); // toggle value
}

void ADC1SS3_Handler(void){
  adcResult = ADC1_SSFIFO3_R;
  ADC1_ISC_R = (1<<3);
//GPIO_PORTF_DATA_R |= (1<<2);
}


void printChar(char c){
  while( ( UART0_FR_R & (1<<5) ) != 0);
    UART0_DR_R = c;
}

void printString(char *string){
  while(*string){
    printChar(*(string++));
  }
}

char readChar(){
  char recByte;
  while( ( UART0_FR_R & (1<<6) ) == 0);
  recByte = UART0_DR_R;
  return recByte;
}
/*

char readChar(){
  char recByte;
   if ( ( UART0_FR_R & (1<<6) ) != 0 ) {
    recByte = UART0_DR_R;
    return recByte;
  }
  else
    return -1;
}

*/
void configure_GPIO_IO(char port, uint8_t io){
  
  switch(port){
    case 'A':
      SYSCTL_RCGCGPIO_R |= (1<<0);
      GPIO_PORTA_LOCK_R = GPIO_LOCK_KEY; // 0x4C4F434B
      GPIO_PORTA_CR_R = 0xFF;
      GPIO_PORTA_PUR_R = ~io;
      GPIO_PORTA_DIR_R = io;
      GPIO_PORTA_DEN_R = 0xFF;
      break;
    case 'B':
      SYSCTL_RCGCGPIO_R |= (1<<1);
      GPIO_PORTB_LOCK_R = GPIO_LOCK_KEY; // 0x4C4F434B
      GPIO_PORTB_CR_R = 0xFF;
      GPIO_PORTB_PUR_R = ~io;
      GPIO_PORTB_DIR_R = io;
      GPIO_PORTB_DEN_R = 0xFF;
      break;
    case 'C': 
      SYSCTL_RCGCGPIO_R |= (1<<2);
      GPIO_PORTC_LOCK_R = GPIO_LOCK_KEY; // 0x4C4F434B
      GPIO_PORTC_CR_R = 0xFF;
      GPIO_PORTC_PUR_R = ~io;
      GPIO_PORTC_DIR_R = io;
      GPIO_PORTC_DEN_R = 0xFF;
      break;
    case 'D': 
      SYSCTL_RCGCGPIO_R |= (1<<3);
      GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY; // 0x4C4F434B
      GPIO_PORTD_CR_R = 0xFF;
      GPIO_PORTD_PUR_R = ~io;
      GPIO_PORTD_DIR_R = io;
      GPIO_PORTD_DEN_R = 0xFF;
      break;
    case 'E': 
      SYSCTL_RCGCGPIO_R |= (1<<4);
      GPIO_PORTE_LOCK_R = GPIO_LOCK_KEY; // 0x4C4F434B
      GPIO_PORTE_CR_R = 0xFF;
      GPIO_PORTE_PUR_R = ~io;
      GPIO_PORTE_DIR_R = io;
      GPIO_PORTE_DEN_R = 0xFF;
      break;
    case 'F': 
      SYSCTL_RCGCGPIO_R |= (1<<5);
      GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // 0x4C4F434B
      GPIO_PORTF_CR_R = 0xFF;
      GPIO_PORTF_PUR_R = ~io;
      GPIO_PORTF_DIR_R = io;
      GPIO_PORTF_DEN_R = 0xFF;
      break;
    default:
      break;
  }
  

}

void configure_UART(char port, int vel){

  switch(port){
    case 'A':
      SYSCTL_RCGCUART_R |= (1<<0);
      SYSCTL_RCGCGPIO_R |= (1<<0);
      GPIO_PORTA_AFSEL_R = (1<<1) | (1<<0);
      GPIO_PORTA_PCTL_R = (1<<0) | (1<<4);
      GPIO_PORTA_DEN_R = (1<<0) | (1<<1);
      UART0_CTL_R &= ~(1<<0);
      UART0_IBRD_R = 8; // vel 115200
      UART0_FBRD_R = 44;
      UART0_LCRH_R = (0x3<<5);
      UART0_CC_R = 0x5;
      UART0_CTL_R = (1<<0) | (1<<8) | (1<<9);
      break;
    default:
      break;
  }

}



void configure_I2C1(void){

  SYSCTL_RCGCI2C_R |= (1<<1);

  SYSCTL_RCGCGPIO_R |= (1<<0);
  
      GPIO_PORTA_LOCK_R = GPIO_LOCK_KEY; // 0x4C4F434B
      GPIO_PORTA_CR_R = 0xFF;
  GPIO_PORTA_AFSEL_R = (1<<6) | (1<<7);
  GPIO_PORTA_DEN_R = (1<<6) | (1<<7);
  
  GPIO_PORTA_ODR_R = (1<<7);

  GPIO_PORTA_PCTL_R &= ~0xFF000000;
  GPIO_PORTA_PCTL_R = (3<<28) | (3<<24);

  I2C1_MCR_R = 0x00000010; // (1<<4)
  
  I2C1_MTPR_R = 0x07; // master or slave
  
//  I2C1_MSA_R = (0x68<<1); // 0x68 es el slave address
//  I2C1_MSA_R &= ~(1<<0);
  
//  I2C1_MDR_R = 0x3B; // Data to transmit
  
//  I2C1_MCS_R = 0x07; // stop start run 
  
}

void sendbyte_i2c1(uint8_t dir, uint8_t byte ){
  I2C1_MSA_R = (dir<<1);
  I2C1_MSA_R &= ~(1<<0); // 0 transmit
  
  I2C1_MDR_R = byte; // Data to transmit
  
  I2C1_MCS_R = 0x07; // stop start run 

  while ( ( I2C1_MCS_R &= I2C_MCS_BUSBSY ) != 0);
}

unsigned char readbyte_i2c1(uint8_t dir){
  unsigned char x;
  
  I2C1_MSA_R = (dir<<1);
  I2C1_MSA_R |= (1<<0); // 1 receive
    
  while( ( I2C1_SCSR_R & (1<<0) ) == 0 );
  x = I2C1_SDR_R;
  return x;
}

 
int main(void)
{
 
  unsigned char a;

  configure_UART('A', 115200);
  configure_I2C1();

  while(1){
    sendbyte_i2c1(MPU_ADDRESS, 'a' );

    a = readbyte_i2c1(MPU_ADDRESS);
    printChar(a);
  }

  return 0;
}




