/*
 * Applied_Robotics DEMO CODE!!!!!.c
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

typedef enum {
	INIT,
	MYTURN,
	IDCUP,
	TAKEAIM,
	LAUNCHBALL,
	HITMISS,
	ERRORCORRECTION,
	TRACKBALL,
	BLOCKSHOT
	
} states;

states state = INIT;
uint8_t lowByte, highByte = 0;
uint16_t prevTimeStamp, timeStamp = 0; 

volatile bool first;
volatile bool triggered;
volatile bool nextState = false; 
volatile bool direction = false;
volatile unsigned long overflowCount;
volatile unsigned long startTime = 0;
volatile unsigned long finishTime = 0;
volatile unsigned long freq; 
volatile unsigned char tick;
unsigned long rotation = 0;
unsigned long lastRotation = 0; 
unsigned int clockRate;
volatile bool rotationUpdated = 0;
unsigned int RPM; 

volatile uint8_t uartData[2] = {0,0};
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
	//Enable overflow interrupt
	TIMSK0 |= (1<<TOIE0);
	//61 ticks is a second. 6.1 ticks is a .1 sec
	//Prescaler set to:    F_CPU = 16000000
	TCCR0B |= (1<<CS02) | (1<<CS00);
	TCNT0 = 0;
	
}


void timer2Init(){
	//OCA2 is PIN 10 on Arduino.
	//Fast PWM timer
	//Timer will reset when it reaches TOP. 
	//Clear OCA2 on Compare Match. Set OC2A at bottom. 
	TCCR2A |= (1<<COM2A1) | (1<<WGM20) | (1<<WGM21);
	//Put in PWM waveform Gen mode. Clock divider set to CLK/256
//	TCCR2B |= (1<<WGM22) | (1<<CS22) | (1<<CS21);
	TCCR2B |= (1<<CS20); 
	//TOP is set by value in OCR2A register
	//Should be able to set OCR2A to PIDoutput 
	OCR2A = 0; 
	//set OCA2(PB4) to output
	DDRB |= (1<<PB4); 
//	PORTB |= (1<<PB4);
		
}
/*
void timer1Init(void){
	// Timer frequency is 160000000/1024 = 15625Hz
	//Normal operation 
	TCCR1A |= 0x00;
	//Trigger on rising edge, prescaler 1024
	TCCR1B |= (1<<ICES1) | (1<<CS12) | (1<<CS10);
	//Enables Input capture interrupt, Enables timer overflow interrupt
	TIMSK1 |= (1<<ICIE1) | (1<<TOIE1);
	//clear all interrupt flags.
	TIFR1 |=  (1<<ICF1) | (1<<TOV1);
	//Eet counter to 0
	TCNT1 = 0;
	//set ICP1(PD4) as input
	//PD4 is pin 47 on the atmega
	DDRD &= ~(1<<PD4);
}
*/

void externalInterrupts(void){
	//PE4 is Digital Pin 2
	//Set pin 4 to input can be input or output to work
	DDRE &= ~(1<<PE4);
	//Turns on Pin 0 of 
//	PCMSK0 |= (1<<PCINT7);
	//Enable interrupt
//	PCICR |= (1<<PCIE0);
	//Falling edge on INT4 generates interrupt
//	EICRB |= (1<<ISC41); // comment out so low level sets interrupt
	//Interrupt on any logical change
	EICRB |= (1<<ISC40); 
	//Look for interrupts on DIGITAL PIN 4 
	EIMSK |= (1<<INT4);
	
}

char uartGetc(void) {
//	cli();
	//Gets a byte from UART
	uint16_t timer = 0;
	while (!(UCSR0A & (1<<RXC0))) {
		timer++;
		if(timer >= 16000) return 0;
	} // Wait for byte to arrive
//	sei();
	return UDR0;
}

void uartSendc(uint8_t u8Data){
//	cli();
	//Sends a byte of data out on UART

	// Wait until last byte has been transmitted
	while((UCSR0A &(1<<UDRE0)) == 0);

	// Transmit data
	UDR0 = u8Data;
//	sei();
}

void uartSends(char s[]){
//	cli();
	//Sends a string our on UART
	int i = 0;
	while(s[i] != 0){
		uartSendc(s[i]);
		i++;
	}
//	sei();
}
/*
void prepareForInterrupts() {
	 cli();  // protected code
	 first = true;
	 triggered = false;  // re-arm for next time
	 // reset Timer 1
	 TCCR1A = 0;
	 TCCR1B = 0;

	 TIFR1 |= (1<<ICF1) | (1<<TOV1);  // clear flags so we don't get a bogus interrupt
	 TCNT1 = 0;          // Counter to zero
	 overflowCount = 0;  // Therefore no overflows yet

	 // Timer 1 - counts clock pulses
	 TIMSK1 |=  (1<<TOIE1) |  (1<<ICIE1);   // interrupt on Timer 1 overflow and input capture
	 // start Timer 1, no prescaler
	 TCCR1B |=  (1<<CS10) |  (1<<ICES1);  // plus Input Capture Edge Select (rising on D8)
	 sei ();
}
*/

void rampMotorSpeed(uint8_t newSpeed){
	static int motorSpeed = 0;
	//Ramp up the motor
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

void driveStepper(uint8_t steps){
	/*TODO: Make this a step forward function
			Make a step backwards function
			Look up how stepper drivers work
	*/
	//PC0-PC3 are used for stepper control
	//PC0=37,   PC1=36,   PC2=35,   PC3=34 
	if(direction == 1){
		//start with 1 on and 3 off. 
		PORTC |= (1<<PC0);
		PORTC &= ~((1<<PC1) | (1<<PC3) | (1<<PC2));
		for(int i=0; i<steps; i++){
			PORTC ^= (1<<PC0) | (1<<PC2);
		}
	}
	if(direction == 0){
		
	}
	
}


void PIDcompute()
{
	if(!inAuto) return;
//	unsigned long now = millis();
//	int timeChange = (now - lastTime);
//	if(timeChange>=SampleTime)
//	{
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
//		lastTime = now;
//	}
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

int main(void)
{
	uartInit();
	timer0Init();
	timer2Init();
	externalInterrupts();
	DDRB |= (1<<PB4) | (1<<PB5) | (1<<PB7);
	//set PC0-3 to output for stepper control
	DDRC |= (1<<PC0) | (1<<PC1) | (1<<PC2) | (1<<PC3);
		
	sei();
	
    while(1)
	    {
		switch(state){
			case INIT:
				uartInit();
				uartSends("Is it my turn?\nEnter '2' to start targeting mode\n");
				//stop the motors
				OCR2A = 0;
				state = HITMISS;
				break;
			
			case MYTURN:
				if(i >= 2){
					uartSendc(uartData[0]);
					uartSendc(uartData[1]);
					if(uartData[0] == 1){
						OCR2A = uartData[1];
						
						
					}				
					if(uartData[0] == 2){
						uartSends("Test\n");
						rampMotorSpeed(uartData[1]);
//						PORTB ^= (1<<PB5);
					}
					if(uartData[0] == 3){
						uartSends("To IDCUP\n");
						PORTB &= ~(1<<PB7);
						state = IDCUP;
						break;
					} else {state = MYTURN;}
//					uartPacketReady = false; 
					i = 0;
				}else {state = MYTURN;}
//				uartSendc(255);
				_delay_ms(250);
				state = MYTURN; 
				break;
			
			case IDCUP:
				uartSends("In IDCUP\n");
				state = TAKEAIM;
				break;
				
			case TAKEAIM:
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
				
				state = LAUNCHBALL;
				break;
				
				
			case LAUNCHBALL:
				//Write print tachometer output
				//Number of cycles divided by (16000000/1024)
//				RPM = 60/(timeStamp/ 15625);
//				uartSendc(rotation);

				if(i >= 2){
					if(uartData[0] == 1){
						//go back to beginning
						PIDsetMode(MANUAL);
						state = MYTURN;
						break;
					}
					if(uartData[0] == 3){
						uartSendc(uartData[1]);
						//byte 1 is setpoint
						Setpoint = (uartData[1]);
						uartSendc((uint8_t)Setpoint);
					}
					i = 0;
				}else {	state = LAUNCHBALL;}
				_delay_ms(100);
				state = LAUNCHBALL;
				break;
				
				
			case HITMISS:
				//Do nothing
				_delay_ms(100);
				//stop the motors
		//		OCR2A = 0; 
				uartSendc(1);
				if(nextState == 1){		
					nextState = 0;
					//spin motor forward
					//PWM output is on pin 10
					OCR2A = 200;
					//PB5 (pin 11) is direction control
					PORTB |= (1<<PB5);
					state = ERRORCORRECTION;
					break;
				}else {state = HITMISS;}
				break;
				
			case ERRORCORRECTION:
				_delay_ms(100);
				//spin motor forward
				//PWM output is on pin 10
//				OCR2A = 150;
				//PB5 (pin 11) is direction control
//				PORTB |= (1<<PB5);
				uartSendc(2);
				if(nextState == 1){
					nextState = 0;
					OCR2A = 0;
					state = TRACKBALL;
					break;
					}else {state = ERRORCORRECTION;}
				break;
				
			case TRACKBALL:
				//do nothing
				_delay_ms(100);
				//stop motors
			//	OCR2A = 0;
				uartSendc(3);
				if(nextState == 1){
					nextState = 0;
					//spin backwards & loop to HITMISS
					//set to low speed
					OCR2A = 200;
					//Reverse direction of motor
					PORTB &= ~(1<<PB5);
					state = BLOCKSHOT;
					break;
					}else {state = TRACKBALL;}
				break;
				
			case BLOCKSHOT:
				_delay_ms(100);
				//spin backwards & loop to HITMISS
				//set to low speed 
		//		OCR2A = 150;
				//Reverse direction of motor
		//		PORTB &= ~(1<<PB5);
				uartSendc(4);
				if(nextState == 1){
					nextState = 0;
					OCR2A = 0;
					state = HITMISS;
					break;
					}else {state = BLOCKSHOT;}
				break;
		
		}
    }
}


ISR(TIMER0_OVF_vect){
	//60 ticks is a second. 6 is a .1 sec
	//only send speed once a second.
	if(tick == 60){
		uartSendc(rotation);
	}
	//12 ticks is 200ms. This gives 300 RPM minimum or 5 RPS
	if(tick == 12){
		tick = 0;
		//rotations/200ms * 1000ms/1s * 60s/min
		RPM = rotation * 300; 
		lastRotation = rotation;
//		uartSendc(rotation);
//		if(state == LAUNCHBALL){
//			PIDinput = (rotation);
//			PIDcompute();
//		}
		//its been 1s check the counter
//		uartSendc((uint8_t)PIDoutput);
		rotation = 0;
		rotationUpdated = 1;
	}
	//computer new  PID output every 50ms
	//if in LAUNCHBALL state.
	/*
	if((tick == 3) && (state = LAUNCHBALL)){
		//(Rotation/50ms) * (1000ms/sec) * (60sec/min) = RPM - not correct anymore
		PIDinput = (rotation);
		PIDcompute();
	}
	*/
	tick++;
}

ISR(INT4_vect){
	rotation++;
	nextState = 1; 
//	PORTB ^= (1<<PB7);
//	_delay_ms(1);
}

ISR(USART0_RX_vect){
//	receivedByte = UDR0;
//	UDR0 = receivedByte;
	uartData[i] = UDR0;
//	uartSendc(255);
	i++;
	if(i >= 2){
		uartPacketReady = true;
	}
//	OCR2A = UDR0;
}
