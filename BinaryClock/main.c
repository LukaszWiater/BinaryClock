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
#define SHIFTR_PORT					PORTC
#define SHIFTR_DDRC					DDRC
#define SHIFTR_SER_PIN				PORTC0
#define SHIFTR_SRCLK_PIN			PORTC1
#define SHIFTR_RCLK_PIN				PORTC2
#define SHIFTR_ENABLE_PORT			PORTB	
//#define SHIFTR_ENABLE_PIN1			PORTB1
//#define SHIFTR_ENABLE_PIN2			PORTB2
#define SHIFTR_BUFFER_LENGTH		8
#define UP							1
#define DOWN						0
#define SHIFTR_MAX_BRIGHTNESS		255
enum SHIFT_REGISTERS				{SHIFT_R_HOURS, SHIFT_R_MINUTES};

// I2C interface
#define I2C_OK						0
#define I2C_ERROR					1
#define I2C_START					0x08
#define I2C_SLA_W_ACK				0x18
#define I2C_MT_DATA_ACK				0x28
#define I2C_SLA_R_ACK				0x40
#define I2C_MR_DATA_ACK				0x50
#define I2C_MR_DATA_NACK			0x58
#define I2C_BUFFER_LENGTH			8

// Real Time Clock (DS1307)
#define DS1307_ADDR					0b1101000
#define RTC_BUTTON_PORT				PINB
#define RTC_BUTTON_PIN				PINB0
#define RTC_BUZZER_DDR				DDRD
#define RTC_BUZZER_PORT				PORTD
#define RTC_BUZZER_PIN				PORTD0
#define RTC_BUZZER_BEEP_SHORT_TIME	3
enum SD1307_REGISTERS				{SECONDS, MINUTES, HOURS, DAY, DATE, MONTH, YEAR, CONTROL};
enum RTC_STATES						{RTC_NORMAL_OP, RTC_HOURS, RTC_MINUTES};
enum BUZZER_BEEP_TYPES				{BUZZER_NO_BEEP, BUZZER_BEEP_SHORT, BUZZER_BEEP_LONG, BUZZER_BEEP_CONFIRM};
	
// Button
#define BUTTON_RELEASED				0
#define BUTTON_PRESSED				1
#define BUTTON_IDLE					0
#define BUTTON_SHORT_PRESSED		1
#define BUTTON_LONG_PRESSED			2
#define BUTTON_LONG_TRESHOLD		100			// 150 cycles of Timer0

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

typedef struct
{
	volatile uint8_t *ddr;
	volatile uint8_t *port;
	uint8_t pin;
	uint8_t beep_short_time;
	uint8_t beep_long_time;
	volatile uint8_t active_beep_type;
	
} Buzzer;

// GLOBAL VARIABLES -----------------------

volatile Time_BCD rtc_current_time;
volatile uint8_t rtc_button_state = BUTTON_IDLE;
Buzzer rtc_buzzer;
//volatile uint8_t rtc_state = RTC_NORMAL_OP;


// FUNCTIONS ------------------------------

void BlinkLed(volatile uint8_t *port, uint8_t pin);
void TogglePin(volatile uint8_t *port, uint8_t pin);
inline uint8_t DecToBDC(uint8_t decimal_num);
inline uint8_t BDCToDec(uint8_t bdc_num);
uint8_t ButtonState(uint8_t pin_register, uint8_t pin);

void ShiftRPinConfig();
void ShiftRSendByte(uint8_t byte);
void ShiftRUpdateDisplay();
void ShiftRToggleDisplay(uint8_t shift_r_number);
inline void ShiftRSetMaxBrightness(uint8_t shift_register);

uint8_t I2CSendData(uint8_t device_address, uint8_t data[], const uint8_t num_of_bytes);
uint8_t I2CReadData(uint8_t device_address, uint8_t data[], const uint8_t num_of_bytes);

Time_BCD RTCReadTime();
void RTCWriteTime(Time_BCD time_rtc);
void RTCConfig();
void RTCInterruptInit();
void RTCDisplayTime(Time_BCD time_rtc);
Time_BCD RTCTimeDecToBCD(Time_Dec time_dec);
Time_Dec RTCTimeBDCToDec(Time_BCD time_bcd);
void RTCButtonUpdate(uint8_t button_state);
void RTCChangeTime();
void RTCBuzzerInit();

Buzzer BuzzerInit(volatile uint8_t *buzzer_ddr, volatile uint8_t *buzzer_port, uint8_t buzzer_pin, uint8_t beep_short_time);
inline void BuzzerTurnON(volatile uint8_t *buzzer_port, uint8_t buzzer_pin);
inline void BuzzerTurnOFF(volatile uint8_t *buzzer_port, uint8_t buzzer_pin);
inline void BuzzerBeep(Buzzer *buzzer, uint8_t beep_type);
void BuzzerHandler(Buzzer *buzzer);

void Timer0Init();
void Timer1Init();
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
	// Updating change-time-state-machine
	RTCChangeTime();
	// Handle the buzzer functioning
	BuzzerHandler(&rtc_buzzer);
	
	TCNT0 = 255 - ovf_time;
}

int main(void)
{
	
	Time_Dec time_dec;
	
	// Initialize rtc with the time below
	time_dec.hours = 12;
	time_dec.minutes = 59;
	time_dec.seconds = 0;
	
	RTCWriteTime(RTCTimeDecToBCD(time_dec));
	
	// configure pin as an output
	DDRB|=(1<<DDRB1);
	
	ShiftRPinConfig();
	RTCConfig();
	RTCInterruptInit();
	rtc_buzzer = BuzzerInit(&RTC_BUZZER_DDR, &RTC_BUZZER_PORT, RTC_BUZZER_PIN, RTC_BUZZER_BEEP_SHORT_TIME);
	Timer0Init();
	Timer1Init();
	//Timer2Init();
	
    while (1) 
    {

    }
}

inline uint8_t DecToBDC(uint8_t decimal_num) {return (((decimal_num/10)%10 << 4) | (decimal_num%10));}
inline uint8_t BDCToDec(uint8_t bdc_num) {return ((bdc_num>>4)*10+(bdc_num&0x0F));}

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

/* Configuring microcontroller's pins for usage with the shift register */
void ShiftRPinConfig()
{
	SHIFTR_DDRC |= (1<<SHIFTR_SER_PIN) | (1<<SHIFTR_SRCLK_PIN) | (1<<SHIFTR_RCLK_PIN);
	SHIFTR_PORT &= (~(1<<SHIFTR_RCLK_PIN)) & (~(1<<SHIFTR_SRCLK_PIN));
	
}

/* Sending one byte of data to the shift register */
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

void ShiftRToggleDisplay(uint8_t shift_r_number)
{
	static int16_t counter = 0;
	static uint8_t direction = UP;
	const uint8_t MAX_COUNTER = 200;
	const uint8_t MIN_COUNTER = 0;
	
	if(direction == UP)
		counter+=10;
	else
		counter-=10;
		
	if(counter >= MAX_COUNTER)
		direction = DOWN;
	else if(counter <= MIN_COUNTER)
		direction = UP;
		
	switch (shift_r_number)
	{
	case SHIFT_R_HOURS:
		OCR1A = counter;
		break;
		
	case SHIFT_R_MINUTES:
		OCR1B = counter;
		break;
	}
		
}

inline void ShiftRSetMaxBrightness(uint8_t shift_register){if(shift_register == SHIFT_R_HOURS) OCR1A = SHIFTR_MAX_BRIGHTNESS; else OCR1B = SHIFTR_MAX_BRIGHTNESS;}


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

Time_BCD RTCTimeDecToBCD(Time_Dec time_dec)
{
	Time_BCD time_bcd;
	
	time_bcd.seconds = DecToBDC(time_dec.seconds);
	time_bcd.minutes = DecToBDC(time_dec.minutes);
	time_bcd.hours   = DecToBDC(time_dec.hours);
	
	return time_bcd;
}

Time_Dec RTCTimeBDCToDec(Time_BCD time_bcd)
{
	Time_Dec time_dec;
	
	time_dec.seconds = BDCToDec(time_bcd.seconds);
	time_dec.minutes = BDCToDec(time_bcd.minutes);
	time_dec.hours   = BDCToDec(time_bcd.hours);
	
	return time_dec;
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
		if(rtc_button_state != BUTTON_LONG_PRESSED)
		{
			if(counter >= BUTTON_LONG_TRESHOLD)
			{
				rtc_button_state = BUTTON_LONG_PRESSED;
				prev_button_state = BUTTON_LONG_PRESSED;
				counter = 0;
			}
			else
				++counter;
		}
	}
	else if(button_state == BUTTON_RELEASED && prev_button_state == BUTTON_PRESSED)
	{
		rtc_button_state = BUTTON_SHORT_PRESSED;
		prev_button_state = BUTTON_RELEASED;
	}
	else if(button_state == BUTTON_RELEASED && prev_button_state == BUTTON_LONG_PRESSED)
		prev_button_state = BUTTON_RELEASED;

}

/* Function responsible for managing the manual time change (user's input) */
void RTCChangeTime()
{
	const uint8_t MAX_HOURS = 24;
	const uint8_t MAX_MINUTES = 60;
	static uint8_t rtc_state = RTC_NORMAL_OP;
	static Time_Dec temp_time_dec; 
	
	// If the conditions for entering the "change hours" state are met then enter this state
	if(rtc_button_state == BUTTON_LONG_PRESSED && rtc_state == RTC_NORMAL_OP)
	{
		rtc_state = RTC_HOURS;
		GICR &= ~(1<<INT0);
		temp_time_dec = RTCTimeBDCToDec(RTCReadTime());
		BuzzerBeep(&rtc_buzzer, BUZZER_BEEP_LONG);
		
		PORTB ^= (1<<PORTB1);
	}
	
	// If the conditions for entering the "change minutes" state are met then enter this state
	else if(rtc_button_state == BUTTON_LONG_PRESSED && rtc_state == RTC_HOURS)
	{
		rtc_state = RTC_MINUTES;
		ShiftRSetMaxBrightness(SHIFT_R_HOURS);
		BuzzerBeep(&rtc_buzzer, BUZZER_BEEP_LONG);
		
		PORTB ^= (1<<PORTB1);
	}
	
	// If the conditions for entering the "normal operation" state are met then enter this state
	else if(rtc_button_state == BUTTON_LONG_PRESSED && rtc_state == RTC_MINUTES)
	{
		rtc_state = RTC_NORMAL_OP;
		RTCWriteTime(RTCTimeDecToBCD(temp_time_dec));
		GICR |= (1<<INT0);
		ShiftRSetMaxBrightness(SHIFT_R_MINUTES);
		BuzzerBeep(&rtc_buzzer, BUZZER_BEEP_CONFIRM);
		
		PORTB ^= (1<<PORTB1);
	}
	
	
	switch (rtc_state)
	{
	case RTC_HOURS:
		if(rtc_button_state == BUTTON_SHORT_PRESSED)
		{
			++temp_time_dec.hours;
			if(temp_time_dec.hours == MAX_HOURS)
				temp_time_dec.hours = 0;
				
			RTCDisplayTime(RTCTimeDecToBCD(temp_time_dec));
			BuzzerBeep(&rtc_buzzer, BUZZER_BEEP_SHORT);
		}
			
		ShiftRToggleDisplay(SHIFT_R_HOURS);
		break;
		
	case RTC_MINUTES:
		if(rtc_button_state == BUTTON_SHORT_PRESSED)
		{
			++temp_time_dec.minutes;
			if(temp_time_dec.minutes == MAX_MINUTES)
				temp_time_dec.minutes = 0;
			
			RTCDisplayTime(RTCTimeDecToBCD(temp_time_dec));
			BuzzerBeep(&rtc_buzzer, BUZZER_BEEP_SHORT);
		}
			
		ShiftRToggleDisplay(SHIFT_R_MINUTES);
		break;
	}
	
	
	rtc_button_state = BUTTON_IDLE;
}

void RTCBuzzerInit()
{
	RTC_BUZZER_DDR |= (1<<RTC_BUZZER_PIN);
	RTC_BUZZER_PORT &= ~(1<<RTC_BUZZER_PIN);
}

Buzzer BuzzerInit(volatile uint8_t *buzzer_ddr, volatile uint8_t *buzzer_port, uint8_t buzzer_pin, uint8_t beep_short_time)
{
	Buzzer new_buzzer;
	
	new_buzzer.ddr = buzzer_ddr;
	new_buzzer.port = buzzer_port;
	new_buzzer.pin = buzzer_pin;
	new_buzzer.beep_short_time = beep_short_time;
	new_buzzer.beep_long_time = 5*beep_short_time;
	
	*new_buzzer.ddr |= (1<<new_buzzer.pin);
	*new_buzzer.port &= ~(1<<new_buzzer.pin);
	
	return new_buzzer;
}

inline void BuzzerTurnON(volatile uint8_t *buzzer_port, uint8_t buzzer_pin) {*buzzer_port |= (1<<buzzer_pin);}
inline void BuzzerTurnOFF(volatile uint8_t *buzzer_port, uint8_t buzzer_pin) {*buzzer_port &= ~(1<<buzzer_pin);}

/* The function responsible for handling the buzzer - it is placed in timer0 overflow interrupt handler (it is called every time the overflow interrupt occurs) */
void BuzzerHandler(Buzzer *buzzer)
{
	static uint8_t buzzer_counter = 0;
	
	switch (buzzer->active_beep_type)
	{
	case BUZZER_NO_BEEP:
		return;
		break;
		
	case BUZZER_BEEP_SHORT:
		if(buzzer_counter == 0)
		{
			BuzzerTurnON(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);
		}
		else if(buzzer_counter >= buzzer->beep_short_time)
		{
			BuzzerTurnOFF(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);
			buzzer_counter = 0;
			buzzer->active_beep_type = BUZZER_NO_BEEP;
			
			return;
		}
		break;
		
	case BUZZER_BEEP_LONG:
		if(buzzer_counter == 0)
		{
			BuzzerTurnON(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);
		}
		else if(buzzer_counter >= buzzer->beep_long_time)
		{
			BuzzerTurnOFF(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);
			buzzer_counter = 0;
			buzzer->active_beep_type = BUZZER_NO_BEEP;
			
			return;
		}
		break;
		
	case BUZZER_BEEP_CONFIRM:	
		if(buzzer_counter == 0)
		BuzzerTurnON(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);

		else if(buzzer_counter == buzzer->beep_short_time)
		BuzzerTurnOFF(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);
		
		else if(buzzer_counter == 2*(buzzer->beep_short_time))
		BuzzerTurnON(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);
		
		else if(buzzer_counter == 3*(buzzer->beep_short_time))
		BuzzerTurnOFF(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);
		
		else if(buzzer_counter == 4*(buzzer->beep_short_time))
		BuzzerTurnON(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);
		
		else if(buzzer_counter >= 5*(buzzer->beep_short_time))
		{
			BuzzerTurnOFF(&RTC_BUZZER_PORT, RTC_BUZZER_PIN);
			buzzer_counter = 0;
			buzzer->active_beep_type = BUZZER_NO_BEEP;
			
			return;
		}
		break;
	}
	
	++buzzer_counter;
}

inline void BuzzerBeep(Buzzer *buzzer, uint8_t beep_type) {buzzer->active_beep_type = beep_type;}

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

void Timer1Init()
{
	DDRB |= (1<<DDRB2) | (1<<DDRB3);
	
	// Fast PWM, 8-bit, inverted mode
	TCCR1A = (1<<COM1A0) | (1<<COM1A1) | (1<<COM1B0) | (1<<COM1B1) | (1<<WGM10);
	// Prescaler fclk/1
	TCCR1B = (1<<WGM12) | (1<<CS10);
	// PWM duty cycle = 0 (always low)
	OCR1A = SHIFTR_MAX_BRIGHTNESS;
	OCR1B = SHIFTR_MAX_BRIGHTNESS;
}

void Timer2Init()
{
	DDRB |= (1<<DDRB1) | (1<<DDRB1);
	
	OCR2 = 0xFF;	
	// FastPWM, prescaler fclk/8, inverted pwm
	TCCR2 = (1<<WGM20) | (1<<WGM21) | (1<<COM20) | (1<<COM21) | (1<<CS21);
	
}