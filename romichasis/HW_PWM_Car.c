//Project 3: Bluetooth Car 
//by Ramon Alaysa, Skye Rogers, Derin Le

//This is the starting file to control the hardware pwm car with bluetooth module

/////////////////////////////////////////////////////////////////////////////
// Course Number: CECS 447
// Example project for Hardware PWM controlled Car
// Description: 
// Hardware connections: assume L298N is used for DC motor driver
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
/////////////////////////////////////////////////////////////////////////////
#include "tm4c123gh6pm.h"
#include "PWM.h"
#include "GPIO.h"

// standard ASCII symbols
#define CR   0x0D
#define LF   0x0A
#define BS   0x08
#define ESC  0x1B
#define SP   0x20
#define DEL  0x7F
#define NULL 0

// Function Prototypes
void DisableInterrupts(void); // Disable interrupts globally
void EnableInterrupts(void);  // Enable interrupts globally
void WaitForInterrupt(void);  // Wait for an interrupt to occur
void Delay(void);            // A delay function, which might pause execution
void UART_Init(void);        // Initialize UART1 for communication
unsigned char UART1_InChar(void); // Receive a character via UART1
void BLT_InString(unsigned char *bufPt); // Receive a string from Bluetooth module
void UART0_OutChar(unsigned char data); // Send a character via UART0
void UART0_OutString(unsigned char *pt); // Send a string via UART0

void Forward(void);          // Move forward function
void Back(void);             // Move backward function
void Left(void);             // Turn left function
void Right(void);            // Turn right function
void BackLeft(void);         // Backward left turn function
void BackRight(void);        // Backward right turn function
void Stop(void); 						 // Stop function
int START_SPEED;

// MAIN: This main is meant for the command configuration of the HC-05 Bluetooth module.
int main(void){ 
	START_SPEED = 4800;
	LED_Init();                  // Initialize LEDs
  Car_Dir_Init();              // Initialize car's direction
  PWM_PB76_Init();             // Initialize PWM control for motors
  PWM_PB76_Duty(START_SPEED, START_SPEED); // Set initial motor speed
	
	PortE_Init();
  unsigned char control_symbol;  // Variable for Bluetooth-controlled LEDs
  UART_Init(); // Initialize UART1
  UART0_OutString((unsigned char *)">>> Welcome to Bluetooth Controlled LED App <<<\n\r");
  
  while(1) {
    control_symbol = UART1_InChar(); // Receive control symbol from Bluetooth
    UART0_OutChar(control_symbol);   // Send the received symbol to UART0
    UART0_OutChar(CR);              // Send Carriage Return character
    UART0_OutChar(LF);              // Send Line Feed character

    switch (control_symbol){
      case 'f' | 'F': 
				Forward();
        break; 
      case 'b' |'B':
        Back();
        break; 
      case 'l' | 'L':
				if(DIRECTION == BACKWARD)
					BackRight();
				else
					Right();
        break; 
      case 'r' | 'R':
				if(DIRECTION == BACKWARD)
					BackLeft();
				else
					Left();
        break; 
      case 's' | 'S':
				Stop();
        break; 
      case 'u' | 'U':
        PWM_PB76_Duty(START_SPEED + 1600, START_SPEED + 1600);
				START_SPEED += 1600;
        break; 
      case 'd' | 'D':
        PWM_PB76_Duty(START_SPEED - 1600, START_SPEED - 1600);
				START_SPEED -= 1600;      
				break; 
      default:
        break;
    }
	}
}

// Functions to control car's movements

void Forward(void){
  LED = Green;
  DIRECTION = FORWARD;
  PWM0_ENABLE_R |= 0x00000003; // Enable both wheels
}

void Back(void){
  LED = Blue;
  DIRECTION = BACKWARD;
  PWM0_ENABLE_R |= 0x00000003; // Enable both wheels
}

void Left(void){
  LED = Yellow;
  DIRECTION = FORWARD;
  PWM0_ENABLE_R &= ~0x00000002; // Enable right wheel, Disable left wheel
}

void BackLeft(void){
  LED = Yellow;
  PWM0_ENABLE_R |= 0x00000002; // Enable right wheel
  PWM0_ENABLE_R &= ~0x00000001; // Disable left wheel
}

void Right(void){
  LED = Purple;
  DIRECTION = FORWARD;
  PWM0_ENABLE_R |= 0x00000002; // Enable right wheel, Disable left wheel
}

void BackRight(void){
  LED = Purple;
  PWM0_ENABLE_R &= ~0x00000002; // Enable right wheel
  PWM0_ENABLE_R |= 0x00000001; // Disable left wheel
}

void Stop(void){
  LED = Dark;
  PWM0_ENABLE_R &= ~0x00000003; // Stop both wheels
}

// Subroutine to wait 0.25 sec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay(void){
	unsigned long volatile time;
  time = 727240*500/91;  // 0.25sec
  while(time){
		time--;
  }
}

//------------UART_Init------------
// Initialize the UART for 19200 baud rate (assuming 16 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART_Init(void){
	// Activate Clocks
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1; // activate UART1
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // activate port B
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
	
	
	UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART0_IBRD_R = 17;                    // IBRD = int(16,000,000 / (16 * 57600)) = int(17.3611111)
  UART0_FBRD_R = 23;                     // FBRD = round(3611111 * 64) = 27
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= 0x301;                 // enable UART for both Rx and Tx

  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1,PA0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1,PA0
                                        // configure PA1,PA0 as UART0
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA1,PA0
	
  UART1_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
	
	// Data Communication Mode, Buad Rate = 57600
  UART1_IBRD_R = 17;                    // IBRD = int(16,000,000 / (16 * 57600)) = int(17.3611111)
  UART1_FBRD_R = 23;                     // FBRD = round(3611111 * 64) = 27
	
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART1_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART1_CTL_R |= 0x301;                 // enable UART for both Rx and Tx
  
  GPIO_PORTB_AFSEL_R |= 0x03;           // enable alt funct on PB1,PB0
  GPIO_PORTB_DEN_R |= 0x03;             // enable digital I/O on PB1,PB0
                                        // configure PB1,PB0 as UART1
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTB_AMSEL_R &= ~0x03;          // disable analog functionality on PB1,PB0

}

//------------UART0_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART0_OutChar(unsigned char data){
  while((UART0_FR_R&UART_FR_TXFF) != 0);
  UART0_DR_R = data;
}

//------------UART0_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART0_OutString(unsigned char *pt){
  while(*pt){
    UART0_OutChar(*pt);
    pt++;
  }
}

//------------UART1_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
unsigned char UART1_InChar(void){
  while((UART1_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART1_DR_R&0xFF));
}

// This function reads response from HC-05 Bluetooth module.
void BLT_InString(unsigned char *bufPt) {
  unsigned char length=0;
  bufPt[length] = UART1_InChar();
  
  // Two possible endings for a reply from HC-05: OK\r\n, ERROR:(0)\r\n
  while (bufPt[length]!=LF) {
    length++;
    bufPt[length] = UART1_InChar();
  };
    
  // add null terminator
  length++;
  bufPt[length] = 0;
}