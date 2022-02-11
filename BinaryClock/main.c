/*
 * BinaryClock.c
 *
 * Created: 27.01.2022 18:30:30
 * Author : Lukasz_Wiater
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


// MACROS --------------------------------

// Shift register
#define SHIFTR_PORT				PORTC
#define SHIFTR_DDRC				DDRC
#define SHIFTR_SER_PIN			PORTC0
#define SHIFTR_SRCLK_PIN		PORTC1
#define SHIFTR_RCLK_PIN			PORTC2
#define SHIFTR_ENABLE_PIN		PORTC3
#define SHIFTR_BUFFER_LENGTH		8

// I2C interface
#define I2C_OK					0
#define I2C_ERROR				1
#define I2C_START				0x08
#define I2C_SLA_W_ACK			0x18
#define I2C_MT_DATA_ACK			0x28
#define I2C_SLA_R_ACK			0x40
#define I2C_MR_DATA_ACK			0x50
#define I2C_MR_DATA_NACK		0x58
#define I2C_BUFFER_LENGTH		8

// Real Time Clock (DS1307)
#define DS1307_ADDR				0b1101000
#define RTC_BUTTON_PORT			PINB
#define RTC_BUTTON_PIN			PINB0
enum SD1307_REGISTERS			{SECONDS, MINUTES, HOURS, DAY, DATE, MONTH, YEAR, CONTROL};
enum RTC_STATES					{RTC_NORMAL_OP, RTC_HOUR1, RTC_HOUR2, RTC_MINUTES1,RTC_MINUTES2};
	
// Button
#define BUTTON_RELEASED			0
#define BUTTON_PRESSED			1
#define BUTTON_IDLE				0
#define BUTTON_SHORT_PRESSED	1
#define BUTTON_LONG_PRESSED		2
#define BUTTON_LONG_TRESHOLD	100			// 150 cycles of Timer0

// TYPEDEF ------------------------------

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	
} Time_BCD;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	
} Time_Dec;

// GLOBAL VARIABLES -----------------------

volatile Time_BCD rtc_current_time;
volatile uint8_t rtc_button_state = BUTTON_IDLE;
//volatile uint8_t rtc_state = RTC_NORMAL_OP;


// FUNCTIONS ------------------------------

void BlinkLed(volatile uint8_t *port, uint8_t pin);
void TogglePin(volatile uint8_t *port, uint8_t pin);
inline uint8_t DecToBDC(uint8_t decimal_num);
uint8_t ButtonState(uint8_t pin_register, uint8_t pin);

void ShiftRPinConfig();
void ShiftRSendByte(uint8_t byte);
void ShiftRUpdateDisplay();

uint8_t I2CSendData(uint8_t device_address, uint8_t data[], const uint8_t num_of_bytes);
uint8_t I2CReadData(uint8_t device_address, uint8_t data[], const uint8_t num_of_bytes);

Time_BCD RTCReadTime();
void RTCWriteTime(Time_BCD time_rtc);
void RTCConfig();
void RTCInterruptInit();
void RTCDisplayTime(Time_BCD time_rtc);
Time_BCD RTCTimeDecToBDC(Time_Dec time_dec);
void RTCButtonUpdate(uint8_t button_state);
void RTCStateMachine();

void Timer0Init();
void Timer2Init();
	
ISR(INT0_vect)
{
	/*
	static uint8_t i = 0;
	uint8_t buffer[1];
	buffer[0] = i;
	ShiftRSendByte(buffer[0]);
	
	++i;
	*/
	
	rtc_current_time = RTCReadTime();
	RTCDisplayTime(rtc_current_time);
	
}

ISR(INT1_vect)
{
	TogglePin(&PORTB, PINB1);
}

ISR(TIMER0_OVF_vect)
{
	// Number of cycles between ovf interrupts
	const uint8_t ovf_time = 20;
	
	// Updating the rtc button's state
	RTCButtonUpdate(ButtonState(RTC_BUTTON_PORT, RTC_BUTTON_PIN));
	
	RTCStateMachine();
	
	TCNT0 = 255 - ovf_time;
}

int main(void)
{
	
	Time_Dec time_dec;
	
	// Initialize rtc with the time below
	time_dec.hours = 12;
	time_dec.minutes = 59;
	time_dec.seconds = 0;
	
	RTCWriteTime(RTCTimeDecToBDC(time_dec));
	
	// configure pin as an output
	DDRB|=(1<<DDRB1);
	
	ShiftRPinConfig();
	RTCConfig();
	RTCInterruptInit();
	Timer0Init();
	Timer2Init();
	
    while (1) 
    {

    }
}

inline uint8_t DecToBDC(uint8_t decimal_num) {return (((decimal_num/10)%10 << 4) | (decimal_num%10));}

void BlinkLed(volatile uint8_t *port, uint8_t pin)
{

	// write pin high
	*port|=(1<<pin);
	_delay_ms(500);
	
	// write pin low
	*port&=~(1<<pin);
	_delay_ms(500);
}

void TogglePin(volatile uint8_t *port, uint8_t pin)
{
	*port^=(1<<pin);
	_delay_ms(500);
	*port^=(1<<pin);
	_delay_ms(500);
}



// ----------------------------------------------------------
// ShiftR ---------------------------------------------------

void ShiftRPinConfig()
{
	SHIFTR_DDRC |= (1<<SHIFTR_SER_PIN) | (1<<SHIFTR_SRCLK_PIN) | (1<<SHIFTR_RCLK_PIN) | (1<<SHIFTR_ENABLE_PIN);
	SHIFTR_PORT &= (~(1<<SHIFTR_RCLK_PIN)) & (~(1<<SHIFTR_SRCLK_PIN)) & (~(1<<SHIFTR_ENABLE_PIN));
	
}

void ShiftRSendByte(uint8_t byte)
{
	const uint8_t len = 8;
	uint8_t digit;
	uint8_t buffer[len];
		
	for(int i=0; i<len; ++i)
	{
		digit=byte%2;
		buffer[i]=digit;
		byte/=2;
	}
	
	for(int i=0; i<len; ++i)
	{
		if(buffer[i])
			SHIFTR_PORT|=(1<<SHIFTR_SER_PIN);
		else
			SHIFTR_PORT&=~(1<<SHIFTR_SER_PIN);
			
		//TogglePin(&SHIFTR_PORT, SHIFTR_SRCLK_PIN);
		SHIFTR_PORT^=(1<<SHIFTR_SRCLK_PIN);
		SHIFTR_PORT^=(1<<SHIFTR_SRCLK_PIN);
	}
	
	//TogglePin(&SHIFTR_PORT, SHIFTR_RCLK_PIN);
	//SHIFTR_PORT^=(1<<SHIFTR_RCLK_PIN);
	//SHIFTR_PORT^=(1<<SHIFTR_RCLK_PIN);
}

void ShiftRUpdateDisplay()
{
	SHIFTR_PORT^=(1<<SHIFTR_RCLK_PIN);
	SHIFTR_PORT^=(1<<SHIFTR_RCLK_PIN);
}



// -------------------------------------------------------
// I2C ---------------------------------------------------

uint8_t I2CSendData(uint8_t device_address, uint8_t data[], const uint8_t num_of_bytes)
{
	// Enable I2C and set START condition
	TWCR = (1<<TWEN) | (1<<TWSTA) | (1<<TWINT);
	
	// Wait for TWINT flag set
	while(!(TWCR & (1<<TWINT)));
	
	// Check the status of transmission (START)
	if ((TWSR & 0xF8) != I2C_START)
		return I2C_ERROR;
		
	// Load data (device address) into TWDR register
	TWDR = (device_address << 1);
	TWCR = (1<<TWINT) | (1<<TWEN);		// Wait for TWINT flag set	while(!(TWCR & (1<<TWINT)));	
	// Check the status of transmission (SLAVE ACKNOWLEDGE)
	if ((TWSR & 0xF8) != I2C_SLA_W_ACK)
		return I2C_ERROR;
	
	for(int i=0; i<num_of_bytes; ++i)
	{	
		// Load data (byte) into TWDR register
		TWDR = data[i];
		TWCR = (1<<TWINT) | (1<<TWEN);				// Wait for TWINT flag set		while(!(TWCR & (1<<TWINT)));		
		// Check the status of transmission (SLAVE ACKNOWLEDGE)
		if ((TWSR & 0xF8) != I2C_MT_DATA_ACK)
			return I2C_ERROR;
	}
	
	// Transmit stop condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	return I2C_OK;
}

uint8_t I2CReadData(uint8_t device_address, uint8_t data[], const uint8_t num_of_bytes)
{
	// Enable I2C and set START condition
	TWCR = (1<<TWEN) | (1<<TWSTA) | (1<<TWINT);
	
	// Wait for TWINT flag set
	while(!(TWCR & (1<<TWINT)));
	
	// Check the status of transmission (START)
	if ((TWSR & 0xF8) != I2C_START)
		return I2C_ERROR;
	
	// Load data (device address) into TWDR register
	TWDR = ((device_address<<1) | 0x01);
	TWCR = (1<<TWINT) | (1<<TWEN);		// Wait for TWINT flag set	while(!(TWCR & (1<<TWINT)));	
	// Check the status of transmission (SLAVE ACKNOWLEDGE)
	if ((TWSR & 0xF8) != I2C_SLA_R_ACK)
		return I2C_ERROR;
	
	for(int i=0;i<num_of_bytes-1;i++)
	{
		// Load data (byte) from TWDR register with ACK
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Wait for TWINT flag set		while(!(TWCR & (1<<TWINT)));		
		// Check the status of transmission (SLAVE ACKNOWLEDGE)
		if ((TWSR & 0xF8) != I2C_MR_DATA_ACK)
			return I2C_ERROR;
			
		data[i] = TWDR;
	}
	
	
	// Load data (byte) from TWDR register with NACK
	TWCR = (1<<TWINT) | (1<<TWEN);				// Wait for TWINT flag set	while(!(TWCR & (1<<TWINT)));			
	// Check the status of transmission (SLAVE ACKNOWLEDGE)
	if ((TWSR & 0xF8) != I2C_MR_DATA_NACK)
		return I2C_ERROR;
	
	data[num_of_bytes-1] = TWDR;
	
	// Transmit stop condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	return I2C_OK;	
}



// -------------------------------------------------------
// RTC ---------------------------------------------------

Time_BCD RTCReadTime()
{
	Time_BCD time_rtc;
	uint8_t buffer[I2C_BUFFER_LENGTH] = {0,0,0,0,0,0,0,0};
	
	buffer[0] = SECONDS;
	if(I2CSendData(DS1307_ADDR, buffer, 1) != I2C_OK)
		TogglePin(&PORTB, PINB1);

	if(I2CReadData(DS1307_ADDR, buffer, 3) != I2C_OK)
		TogglePin(&PORTB, PINB1);
		
	time_rtc.seconds = buffer[0];
	time_rtc.minutes = buffer[1];
	time_rtc.hours	 = buffer[2];
	
	return time_rtc;
}

void RTCWriteTime(Time_BCD time_rtc)
{
	uint8_t buffer[I2C_BUFFER_LENGTH] = {0,0,0,0,0,0,0,0};
		
	buffer[0] = SECONDS;
	buffer[1] = time_rtc.seconds;
	buffer[2] = time_rtc.minutes;
	buffer[3] = time_rtc.hours;
	
	if(I2CSendData(DS1307_ADDR, buffer, 4) != I2C_OK)
		TogglePin(&PORTB, PINB1);
}

void RTCConfig()
{	
	uint8_t buffer[I2C_BUFFER_LENGTH] = {0,0,0,0,0,0,0,0};

	buffer[0] = CONTROL;
	buffer[1] = 0b00010000;		//square wave output is on (1 Hz)

	if(I2CSendData(DS1307_ADDR, buffer, 2) != I2C_OK)
		TogglePin(&PORTB, PINB1);
		
	// Button input configuration
	DDRB &= ~(1<<PINB0);
	PORTB &= ~(1<<PINB0);
}	

void RTCInterruptInit()
{
	// Rising edge on PD2 triggers INT0 and falling edge on PD3 triggers INT1
	MCUCR |= (1<<ISC00) | (1<<ISC01);
	
	// Enable IT0 and interrupts
	GICR |= (1<<INT0);
	sei();
}

void RTCDisplayTime(Time_BCD time_rtc)
{
	const uint8_t buffer_size = 3;
	uint8_t buffer[buffer_size];
	buffer[0] = time_rtc.seconds;
	buffer[1] = time_rtc.minutes;
	buffer[2] = time_rtc.hours;
	
	for(uint8_t i=0; i<buffer_size; ++i)
		ShiftRSendByte(buffer[i]);
		
	ShiftRUpdateDisplay();
}

Time_BCD RTCTimeDecToBDC(Time_Dec time_dec)
{
	Time_BCD time_bcd;
	
	time_bcd.seconds = DecToBDC(time_dec.seconds);
	time_bcd.minutes = DecToBDC(time_dec.minutes);
	time_bcd.hours   = DecToBDC(time_dec.hours);
	
	return time_bcd;
}

void RTCButtonUpdate(uint8_t button_state)
{
	static uint8_t prev_button_state = BUTTON_RELEASED;
	static uint8_t counter = 0;
	
	if(button_state == BUTTON_PRESSED && prev_button_state == BUTTON_RELEASED)
	{
		prev_button_state = BUTTON_PRESSED;
		counter = 0;
	}
	else if(button_state == BUTTON_PRESSED)
	{
		++counter;
	}
	else if(button_state == BUTTON_RELEASED && prev_button_state == BUTTON_PRESSED)
	{
		prev_button_state = BUTTON_RELEASED;
		
		if(counter >= BUTTON_LONG_TRESHOLD)
			rtc_button_state = BUTTON_LONG_PRESSED;
		else
			rtc_button_state = BUTTON_SHORT_PRESSED;
	}
}

void RTCStateMachine()
{
	static uint8_t rtc_state = RTC_NORMAL_OP;
	static Time_BCD temp_time, empt_time;
	static uint8_t counter = 0;
	
	if(rtc_button_state == BUTTON_LONG_PRESSED && rtc_state == RTC_NORMAL_OP)
	{
		rtc_state = RTC_HOUR1;
		temp_time = RTCReadTime();
		empt_time = temp_time;
		empt_time.hours &= 0x0F;
		
		PORTB ^= (1<<PORTB1);
	}
	else if(rtc_button_state == BUTTON_LONG_PRESSED && rtc_state == RTC_HOUR1)
		rtc_state = RTC_HOUR2;

	else if(rtc_button_state == BUTTON_LONG_PRESSED && rtc_state == RTC_HOUR2)
		rtc_state = RTC_MINUTES1;

	else if(rtc_button_state == BUTTON_LONG_PRESSED && rtc_state == RTC_MINUTES1)
		rtc_state = RTC_MINUTES2;

	else if(rtc_button_state == BUTTON_LONG_PRESSED && rtc_state == RTC_MINUTES2)
		rtc_state = RTC_NORMAL_OP;
		
	switch (rtc_state)
	{
	case RTC_HOUR1:
		if(rtc_button_state == BUTTON_SHORT_PRESSED)
			++temp_time.hours;
	
		if(counter == 25)
		{
			RTCWriteTime(empt_time);
		}
		else if(counter == 50)
		{
			RTCWriteTime(temp_time);
			counter = 0;
		}
	
		++counter;
		break;
	}
	
	rtc_button_state = BUTTON_IDLE;
}

uint8_t ButtonState(uint8_t pin_register, uint8_t pin)
{
	static uint8_t counter = 0;
	static uint8_t is_pressed = 0;
	
	if(counter==0)
	{
		if(((pin_register & (1<<pin)) == 0) && (!is_pressed))
		{
			counter = 5; // number of function calls after button has been pressed
			is_pressed = 1;
		}
		else if(((pin_register & (1<<pin)) == 1) && (is_pressed))
		{
			counter = 5; // number of function calls after button has been released
			is_pressed = 0;
		}
	}
	else
		--counter;
		
	return is_pressed;
}

void Timer0Init()
{
	// Prescaler f_clk/1024 = 1000000/1024 ~ 1000 Hz
	TCCR0 |= (1<<CS02) | (1<<CS00);
	
	// Timer0 overflow interrupt enable
	TIMSK |= (1<<TOIE0);
}

void Timer2Init()
{
	DDRB |= (1<<DDRB3);
	
	OCR2 = 0xFF;	
	// FastPWM, prescaler fclk/8, inverted pwm
	TCCR2 = (1<<WGM20) | (1<<WGM21) | (1<<COM20) | (1<<COM21) | (1<<CS21);
	
}