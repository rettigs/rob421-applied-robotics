/*
 * Applied_Robotics.c
 *
 * Created: 4/7/2015 6:06:12 PM
 *  Author: Marty
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>

//#define F_CPU 16000000
//#define BAUD 115200

uint8_t lowByte, highByte = 0;
uint16_t prevTimeStamp, timeStamp = 0; 

volatile bool first;
volatile bool triggered;
volatile unsigned long overflowCount;
volatile unsigned long startTime = 0;
volatile unsigned long finishTime = 0;
volatile unsigned long freq; 
volatile unsigned char tick;
volatile float rotation = 0;
unsigned long lastRotation = 0; 
unsigned int clockRate;
volatile bool rotationUpdated = 0;
volatile float RPM; 

volatile uint8_t uartData[3] = {0,0,0};
volatile char i = 0;
volatile bool uartPacketReady = false; 

/*PID Global Variables*/
unsigned long lastTime;
double PIDinput, PIDoutput, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 50; //50ms
double outMin, outMax;
bool inAuto = false;

#define MANUAL 0
#define AUTOMATIC 1

#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;
//end PID

void uartInit(void){
	//RXD0 is PORTE bit 0
	//TXD0 is PORTE bit 1
	//Jumper J7 and J9 (mega128.2)
	//must be in place for the MAX232 chip to get data.
	//rx and tx enable, 8bit characters
	UCSR0B |= (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
	//async operation, no parity, one stop bit, 8-bit characters
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
	//enable usart interrupts
	UCSR0B |= (1<<RXCIE0);
	//set baud to 9600
//	clockRate = (F_CPU/(16*BAUD))-1;
//	UBRR0H = (unsigned char)(clockRate>>8);
//	UBRR0L = (unsigned char)clockRate;
	UBRR0H = 0x00;
	UBRR0L = 0x67;
}


void timer0Init(){
	//Timekeeper timer	
	TIMSK0 |= (1<<TOIE0);
	//61 ticks is a second. 6.1 ticks is a .1 sec
	//Prescaler set to:    F_CPU = 16000000
	TCCR0B |= (1<<CS02) | (1<<CS00);
	/*
	//Put in CTC mode
	TCCR0A |= (1<<WGM01);
	//Prescaler set to:1024   F_CPU = 16000000
	//With top of 156 should CTC at 100Hz
	TCCR0B |= (1<<CS02) | (1<<CS00);
	//Enable interrupt
	TIMSK0 |= 
	//Set timer TOP to 156 (100Hz)
	OCR0A = 156;
	//start timer at bottom
	*/
	TCNT0 = 0;
	
}

void timer1Init(){
	//Set up Timer1 as 16bit PWM for launcher/ swatter
	//OC1A is Digital 11 (PB5) Launcher
	//OC1b is Digital 12 (PB6) Swatter
	//Fast PWM Top set by OCR1A (Non-Inverting PWM)
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11) | (1<<WGM10);
	//Clock divider 256. (overflow every 1.048 sec)
	TCCR1B |= (1<<WGM12) | (1<<WGM12) | (1<<CS10);
	DDRB |= (1<<PB5) | (1<<PB6);
	//set register 
	OCR1A = 0;
	OCR1B = 0;
}

void timer2Init(){
	//Fast 8bit PWM timer
	//OC2A is PIN 10 on Arduino.
	//OC2B is PIN 9 on Arduino. 
	//Timer will reset when it reaches TOP. 
	//Clear OCA2 on Compare Match. Set OC2A at bottom. 
	TCCR2A |= (1<<COM2A1) | (1<<COM2B1) | (1<<WGM20) | (1<<WGM21);
	//Put in PWM waveform Gen mode. Clock divider set to CLK/256
//	TCCR2B |= (1<<WGM22) | (1<<CS22) | (1<<CS21);
	TCCR2B |= (1<<CS20); 
	//TOP is set by value in OCR2A register
	//Should be able to set OCR2A to PIDoutput 
	OCR2A = 0; 
	//set OCA2(PB4) to output
	DDRB |= (1<<PB4); 
	//set OCB2(PH6) to output
	DDRH |= (1<<PH6);
		
}

void timer3Init(){
	//This timer is set for 20ms periods for servo control. 
	//Outputs on OC3A (DIGITAL 5)
	//Set to Fast PWM
	//Set to inverting PWM mode
	TCCR3A |= (1<<WGM31) | (1<<COM3A1) | (1<<COM3A0);
	//Set to PWM mode with clk divider of 256. 
	TCCR3B |= (1<<WGM32) | (1<<WGM33) | (1<<CS32);
	//Set TOP of counter
	ICR3 = 1250; 
	//Set toggle point
	OCR3A = 1250;
	//set to output
	DDRE |= (1<<PE3);
}

void timer5Init() {
	//Count on external pulses for motor feedback.
	//Alt function T5(DIGITAL 47)
	//In normal mode. Normal pin operation. 
	//Set clk to be external rising edge
	TCCR5B |= (1<<CS52) | (1<<CS51) | (1<<CS50);
	//set counter to 0
	TCNT5 = 0;
}

void externalInterrupts(void){
	//PE4 is Digital Pin 2
	//PE5 is Digital Pin 3
	//PD2 is Int2 on Digital Pin 19
	DDRD &= ~(1<<PD2);
	PORTD |= (1<<PD2);
	//Set PE5 to input and enable pullup
	DDRE &= ~(1<<PE5);
	PORTE |= (1<<PE5);
	//Trigger INT2 on falling edge
	EICRA |= (1<<ISC21); 
	//Set pin 4 to input can be input or output to work
	//Falling edge on INT4 or rising edge on INT5 generates interrupt
	EICRB |= (1<<ISC41) | (1<<ISC51); // comment out so low level sets interrupt
	//Look for interrupts on DIGITAL PIN 4 
	EIMSK |= (1<<INT4) | (1<<INT5) | (1<<INT2);
	
}

char uartGetc(void) {
	//Gets a byte from UART
	uint16_t timer = 0;
	while (!(UCSR0A & (1<<RXC0))) {
		timer++;
		if(timer >= 16000) return 0;
	} // Wait for byte to arrive
	return UDR0;
}

void uartSendc(uint8_t u8Data){
	//Sends a byte of data out on UART
	// Wait until last byte has been transmitted
	while((UCSR0A &(1<<UDRE0)) == 0);

	// Transmit data
	UDR0 = u8Data;
}

void uartSends(char s[]){
	//Sends a string our on UART
	int j = 0;
	while(s[j] != 0){
		uartSendc(s[j]);
		j++;
	}
}

void rampMotorSpeed(uint8_t newSpeed){
	static int motorSpeed = 0;
	//Ramp up the motor
	/*
	
	
	THIS FUNCTION IS BROKEN WITH 16BIT PWM
	
	
	*/
	while(lastRotation <= newSpeed){
		//if rotation of motor is less than desired speed increase speed 
		//write new speed to PWM control and wait until new measurement is in. 
		//Check a flag if rotation has been updated. Dont want to change 
		//speed before motor can respond.
		//WARNING: This function blocks other operation. 
		//TODO: Ramp up faster but do not overflow. 
		if(rotationUpdated == 1){
			if(motorSpeed < 255){
				motorSpeed++;
			}
			OCR2A = motorSpeed;	
			rotationUpdated = 0;
		}
		
	}
	while(lastRotation >= newSpeed){
		if(rotationUpdated == 1){
			if(motorSpeed > 0){
				motorSpeed--;
			}
			OCR2A = motorSpeed;
			rotationUpdated = 0; 
		}
	}
}

void driveStepper(uint16_t steps, bool direction){
	//Reset PC0-PC3 to zero
	PORTC &= ~((1<<PC0) | (1<<PC1) | (1<<PC2) | (1<<PC3));
	PORTC |= (1<<PC0) | (1<<PC1);
	//PC0-PC3 are used for stepper control
	//PC0=37=E2,   PC1=36=E1,   PC2=35=M2,   PC3=34=M1 
	//YELLOW and RED go to POSITIVE Terminal
	//direction = 1 is counter clockwise
	if(direction == 1){
		for(int i=0; i<steps; i++){
			PORTC |= (1<<PC2) | (1<<PC3);
			_delay_ms(2);
			PORTC &= ~(1<<PC2);
			_delay_ms(2);
			PORTC &= ~(1<<PC3);
			_delay_ms(2);
			PORTC |= (1<<PC2);
			_delay_ms(2);
		}
	}
	//Check this order - it grinds the gears/ is jumpy
	if(direction == 0){
		for(int i=0; i<steps; i++){
			PORTC |= (1<<PC2);
			PORTC &= ~(1<<PC3);
			_delay_ms(2);
			PORTC &= ~(1<<PC2);
			_delay_ms(2);
			PORTC |= (1<<PC3);
			_delay_ms(2);
			PORTC |= (1<<PC2);
			_delay_ms(2);
		}
	}
	PORTC &= ~((1<<PC0) | (1<<PC1) | (1<<PC2) | (1<<PC3));
}
 
void PIDcompute()
{
	if(!inAuto) return;
		/*Compute all the working error variables*/
		double error = Setpoint - PIDinput;
		ITerm+= (ki * error);
		if(ITerm > outMax) ITerm= outMax;
		else if(ITerm < outMin) ITerm= outMin;
		double dInput = (PIDinput - lastInput);
		
		/*Compute PID Output*/
		PIDoutput = kp * error + ITerm- kd * dInput;
		if(PIDoutput > outMax) PIDoutput = outMax;
		else if(PIDoutput < outMin) PIDoutput = outMin;
		
		//Set PIDoutput to controlling PWM register
		OCR2A = PIDoutput; 
		
		/*Remember some variables for next time*/
		lastInput = PIDinput;
}

void PIDinitialize()
{
	lastInput = PIDinput;
	ITerm = PIDoutput;
	if(ITerm > outMax) ITerm= outMax;
	else if(ITerm < outMin) ITerm= outMin;
}

void PIDsetTunings(double Kp, double Ki, double Kd)
{
	if (Kp<0 || Ki<0|| Kd<0) return;
	
	double SampleTimeInSec = ((double)SampleTime)/1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;
	
	if(controllerDirection ==REVERSE)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

void PIDsetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio  = (double)NewSampleTime
		/ (double)SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long)NewSampleTime;
	}
}

void PIDsetOutputLimits(double Min, double Max)
{
	if(Min > Max) return;
	outMin = Min;
	outMax = Max;
	
	if(PIDoutput > outMax) PIDoutput = outMax;
	else if(PIDoutput < outMin) PIDoutput = outMin;
	
	if(ITerm > outMax) ITerm= outMax;
	else if(ITerm < outMin) ITerm= outMin;
}

void PIDsetMode(int Mode){
	bool newAuto = (Mode == AUTOMATIC);
	if(newAuto == !inAuto){
		  /*we just went from manual to auto*/
		PIDinitialize();
	}
	inAuto = newAuto;
}

void PIDsetControllerDirection(int Direction)
{
	controllerDirection = Direction;
}

void PIDenable(void){
	//Turn on PID and start setting it
	//Allow PID to automatically control settings
	PIDsetMode(AUTOMATIC);
	//Positive proportional control
	PIDsetControllerDirection(DIRECT);
	//set sample time in ms - should be handled in interrupt?
	PIDsetSampleTime(50);
	//Minimum and Maximum output values
	PIDsetOutputLimits(0,255);
	//Set PID params (Kp, Ki, Kd)
	PIDsetTunings(6,0.6,0.6);
	//Start controller
	PIDinitialize();
	//Set Setpoint to 0 RPM
	Setpoint = 0;
}

int main(void)
{
	uartInit();
	timer0Init();
	timer1Init();
//	timer2Init();
	timer3Init();
	timer5Init(); 
	externalInterrupts();
	//PB7 is Digital 13 (also LED)
	DDRB |= (1<<PB4) | (1<<PB7);
	//set PC0-3 to output for stepper control
	//PC0-PC3 are used for stepper control
	//PC0=37,   PC1=36,   PC2=35,   PC3=34
	DDRC |= (1<<PC0) | (1<<PC1) | (1<<PC2) | (1<<PC3);
		
	sei();
	
    while(1)
	    {
				if(uartPacketReady == true){
					//Echo back received data
					uartSendc(uartData[0]);
					uartSendc(uartData[1]);
					uartSendc(uartData[2]);
					//Serial Command Packet: TTIIIIID
					//TT=00 (motor). IIIII=00000 (launcher motor). D=0/1 (forward/backward)
					
					//Motor 0 (launcher) forward control
					//HEX CODE: 00 XX XX
					if(uartData[0] == 0b00000000){
						//NOTE TOP IS 0X3FF!!!!
						PORTB &= ~(1<<PB7);
						OCR1A = (uartData[1]<<8) | uartData[2];
	//					OCR1AL = uartData[2];
	//					uartSendc(uartData[1]);
	//					uartSendc(uartData[2]);
					}	
					//Motor 0 (launcher) backward control
					//HEX CODE: 01	XX	XX	
					if(uartData[0] == 0b00000001){
						PORTB |= (1<<PB7);
						OCR1A = (uartData[1]<<8) | uartData[2];
//						OCR1AL = uartData[2]; 

					}
					//Reload Command
					//TT= 01
					//HEX CODE: 40 00 00
					if(uartData[0] == 0b01000000){
						//Move servo backward.
						OCR3A = 1235;
						//Set up external interrupt to stop servo when sensor is touched
						
						//Reverse direction of Servo and move backward.

					}
					//Carriage (Motor 1) forward control
					//HEX CODE: 02 XX
					if(uartData[0] == 0b00000010){
						//ToDo: need stepper motor/weight estimate for chassis.
						//counterclockwise rotation
						driveStepper(((uartData[1]<<8) | uartData[2]), 1);
					}
					//Carriage (Motor 1) backward control
					//HEX CODE: 03 XX
					if(uartData[0] == 0b00000011){
						//ToDo: need stepper motor/weight estimate for chassis. 	
						//clockwise rotation
						driveStepper(((uartData[1]<<8) | uartData[2]), 0);
					}
					uartPacketReady = false;
				}
				_delay_ms(250);
		}
}


ISR(TIMER0_OVF_vect){
	//60 ticks is a second. 6 is a .1 sec
	//only send speed once a second.
	/*
		Ticks every 4 seconds.....
	*/
	if(tick == 60){
		//Read number of pulses counted
		rotation = TCNT5; 
		//reset counter. 
		TCNT5 = 0; 
		//(pulse/sec)*(rotation/2000)*(60sec/min)
		RPM = (rotation*(3/100));		
//		uartSendc(rotation);
		//Send back data
//		uartSendc((uint16_t)RPM>>8);
//		uartSendc((uint8_t)RPM);
	}
	tick++;
}

ISR(INT4_vect){
	//Optical encoder feedback from Launching motor
	//Input pin is Digital 2
	rotation++;
}
ISR(INT5_vect){
	//Forward limit switch - stop the motor from moving.
	//Switch should be attached to ground and Digital 3
	OCR3A = 1171; 
//	uartSendc(0xff);
}
ISR(INT2_vect){
	//Rear limit switch - reverses direction of motor.
	//Should be attached to ground and Digital 19.
	OCR3A = 1171;
//	uartSendc(0x0f);
	_delay_ms(20);
	OCR3A = 1157;
}

ISR(USART0_RX_vect){
	uartData[i] = UDR0;
	i++;
	if(i >= 3){
		i=0;
		uartPacketReady = true;
	}
}
