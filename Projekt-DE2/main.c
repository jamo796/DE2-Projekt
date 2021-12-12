/*
 * Hladina.c
 *
 * Created: 18.11.2021 23:43:01
 * Author : vojta,honza,filip
 */ 

#ifndef F_CPU
# define F_CPU 16000000
#endif
#include <util/delay.h>
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include "lcd.h"            // Peter Fleury's LCD library
#include <stdlib.h>

#define TRIG		PB5 // 
#define ECHO		PB4 // 
#define TEMPERATURE PC1 // switch - switching between display modes /A1
#define PINDISPLAY  PC2 // switch - switching between display modes /A2
#define PUMP        PC3 // switch - controls the automation of the pump /A3
#define VALVE       PC4 // switch - controls the automation of the valve /A4
#define LEDP        PD0 // LED - represents pump lights up when the pump is on /Rx
#define LEDV        PD1 // LED - represents pump that will turn on every 24 hours when temperature is ok /Tx
long depth = 0; //measured distance
double temp = 0;   // measured temperature
int state[] = {1,2,0,0};//{[1-normal;2-temperature below 5°C or over 35°C;3-off] , [1-the tank is empty;2-normal;3-the tank is full],[PUMP on/off],[VALVE open/closed]}
int display = 0; //switching between display modes {0-status mode;1-data mode}

uint8_t customChar[8] = { // defining the "°" symbol
	0b11100,
	0b10100,
	0b11100,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000
};
void displaymode0()       //voids that erase display and renders text corresponding to display mode
{						  //
	display = 0;          //
	lcd_clrscr();         //
	lcd_gotoxy(0, 0);     //
	lcd_puts("Tank:");    //
	lcd_gotoxy(0, 1);     //
	lcd_puts("Temp:");	  //
}                         //
void displaymode1()       //
{                         //
	display = 1;          //
	lcd_clrscr();         //
	lcd_gotoxy(0, 0);     //
	lcd_puts("Distance:");//
	lcd_gotoxy(0, 1);     //
	lcd_puts("Temp:");    //
	lcd_gotoxy(13, 0);    //
	lcd_puts("cm");       //
	lcd_gotoxy(13, 1);    //
	lcd_putc(0);          //
	lcd_puts("C");        //
}                         //*

int main(void)
{

    lcd_init(LCD_DISP_ON);  //turning on LCD 
	

	lcd_command(1 << LCD_CGRAM);   // setting the custom symbol
	for (uint8_t i = 0; i < 8; i++)//
	{							   //
		lcd_data(customChar[i]);   //
	}                              // 
	lcd_command(1 << LCD_DDRAM);   //*
	
    
    DDRB = DDRB | (1<<TRIG);	   //setting up registers of output pins
    PORTB = PORTB & ~(1<<TRIG);	   //
	                               //
	DDRD = DDRD | (1<<LEDP);       //
	PORTD = PORTD & ~(1<<LEDP);    //
	                               //
	DDRD = DDRD | (1<<LEDV);       //
	PORTD = PORTD & ~(1<<LEDV);    //*
	
	DDRB = DDRB & ~ (1<<ECHO);         //setting up registers of input pins
    PORTB = PORTB  & ~ (1<<ECHO);      //
									   //
	DDRC = DDRC & ~ (1<<TEMPERATURE);  //
	PORTC = PORTC | (1<<TEMPERATURE);  //
									   //
	DDRC = DDRC & ~ (1<<PINDISPLAY);   //
	PORTC = PORTC | (1<<PINDISPLAY);   //
	                                   //
	DDRC = DDRC & ~ (1<<PUMP);         //
	PORTC = PORTC | (1<<PUMP);         //
	                                   //
	DDRC = DDRC & ~ (1<<VALVE);        //
	PORTC = PORTC | (1<<VALVE);        //*

	if (((PINC >> PINDISPLAY) & 1) != 0)                  //setting up variables corresponding to switches states
	{                                                     //
		displaymode0();		                              //
	}                                                     //
	else                                                  //
	{                                                     //
		displaymode1();                                   //
	}                                                     //
	                                                      //
	if (((PINC >> TEMPERATURE) & 1) != 0){state[0]=3;}    //
	else{state[0]=1;}                                     //
		                                                  //
	if (((PINC >> PUMP) & 1) != 0){state[2]=0;}           //
	else{state[2]=1;}                                     //
		                                                  //
	if (((PINC >> VALVE) & 1) != 0){state[3]=0;}          //
	else{state[3]=1;}                                     //*
	                                                      
	
	// Configure ADC to convert PC0[A0] analog value
	// Set ADC reference to AVcc
	ADMUX |= (1<<REFS0);
	ADMUX  &= ~ (1<<REFS1);
	// Set input channel to ADC0
	ADMUX &= ~ (1<<MUX0);
	ADMUX &= ~ (1<<MUX1);
	ADMUX &= ~ (1<<MUX2);
	ADMUX &= ~ (1<<MUX3);
	
	// Enable ADC module
	ADCSRA |= (1<<ADEN);
	// Enable conversion complete interrupt
	ADCSRA |= (1<<ADIE);
	// Set clock prescaler to 128
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);
	//Interrupts
	TIM2_overflow_16ms();
	TIM1_overflow_1s();
	TIM2_overflow_interrupt_enable();
	TIM1_overflow_interrupt_enable();		
	sei();
    while (1) 
    {
    }
	return 0;
}
void LCD() //Void to draw out LCD
{
	char lcd_string[4] = "0000";
	if (display == 1)
	{
		if(state[0] != 2) // when temperature is normal or temperature measuring is off
		{
			//Distance sensor
			lcd_gotoxy(10, 0);
			lcd_puts("   cm");
			itoa(depth, lcd_string, 10);
			lcd_gotoxy(10, 0);
			lcd_puts(lcd_string);
		
			//Temperature sensor
			itoa(temp, lcd_string, 10);
			lcd_gotoxy(10, 1);
			lcd_puts("   ");
			lcd_gotoxy(10, 1);
			lcd_puts(lcd_string);
		}
		else                              // when temperature is bellow x and measuring is on
		{
			lcd_clrscr();
			lcd_gotoxy(0, 0);
			lcd_puts("Warning the temperature");
			lcd_gotoxy(0, 1);
			lcd_puts("is not OK!");
		}
	}
	else //when display mode is 0
	{
		if(state[0] != 2)                  // when temperature is normal or temperature measuring is off
		{
			//Distance sensor
			lcd_gotoxy(6, 0);
			switch (state[1])  //tank fill rate
			{
				case 1:
					lcd_gotoxy(6, 0);
					lcd_puts("filling");
					break;
				case 2:
				    lcd_puts("normal ");
					break;
				case 3:
					lcd_puts("full   ");
					break;
				case 4:
					lcd_puts("off    ");
					break;
			}
			//Temperature sensor
			lcd_gotoxy(6, 1);
			if (state[0] == 3)
			{
				lcd_puts("off");
			}
			else
			{
				lcd_puts("on ");
			}
		}
		else                             // when temperature is bellow x and measuring is on
		{
			lcd_clrscr();
			lcd_gotoxy(0, 0);
			lcd_puts("Warning the temperature");
			lcd_gotoxy(0, 1);
			lcd_puts("is not OK!");
		}
		
	}
}
ISR(ADC_vect) // ISR of analog pin conversion
{
    //změření napětí na termistoru
    temp = ADC;
	temp = ADC*5;
    temp/=1023;
    temp = -1.6*temp*temp*temp+13.1*temp*temp-51.8*temp+97.5;
	if(((temp > 35) || (temp < 5)) && state[0] != 3)
	{
		state[0] = 2;
	}
    else if(state[0] != 3){state[0] = 1;} 
}
ISR(TIMER1_OVF_vect) // ISR of measuring functions
{
	static int out = 0; //variable used to run pulse measuring method
	static int j = 0; //variable used for calculating time of pump on
    static int k = 0; 
	                                             //Distance sensor
	PORTB = PORTB ^ (1<<TRIG);                   //sending 10uS trigger pulse
	_delay_us(10);                               //
	PORTB = PORTB ^ (1<<TRIG);                   //
	while (out == 0)                             //waiting to echo of pulse
	{                                            //
		if (((PINB >> ECHO) & 1) != 0)           //when logic 1 on echo pin received
		{                                        //
			depth = 0;                           //reset depth variable
			while(((PINB >> ECHO) & 1) != 0)     //while logic 1 is still on pin ECHO
			{                                    //
				_delay_us(1);                    //
				depth++;                         // adding ++ to depth variable every 1uS when echo is still logic 1
			}                                    //
			depth = depth*42/1000;               // recalculating depth from echo pulse width 
			out = 1;                             //out = 1 to end while loop
                                                 //
		}                                        //
	}                                            //
	out = 0;                                     //*
	
	if (depth<20){state[1] = 3;}         //setting up tank status depending on the depth
	else if (depth>280){state[1] = 1;}   //
	else{state[1] = 2;}                  //*
		                 
	ADCSRA |= (1<<ADSC); //analog pin measuring

	if(j > 0)                            //turning off pump after some time 
	{                                    //
		j++;                             //
		if (j>9)                         //
		{                                //
			j=0;                         //
			PORTD = PORTD & ~(1<<LEDP);  //
		}                                //
	}                                    //*
	if(state[1] == 1 && state[2] == 1)    //turning on pump when tank is empty and pump is on
	{                                     //
		PORTD = PORTD | (1<<LEDP);        //
		j = 1;                            //
	}                                     //*
	if ((k >= 10) && (state[1] != 1) && (state[3] == 1))    // Wattering pump control
    {
     k = 0;
     PORTD = PORTD | (1<<LEDV);       
    }
    else
    {
        if (k<10){k++;}    
        PORTD = PORTD & ~(1<<LEDV);
    }            
	
                            	
    LCD(); //LCD rendering
}
ISR(TIMER2_OVF_vect) //ISR to detect switches changes
{
	if ((((PINC >> PINDISPLAY) & 1) != 0)) //Display switch
	{                                      //
		if (display == 1)                  //
		{                                  //
			displaymode0();                //
		}                                  //
	}                                      //
	else if (display == 0)                 //
	{                                      //
		displaymode1();                    //
	}                                      //*
	
	if ((((PINC >> TEMPERATURE) & 1) != 0)) //Temperature switch
	{                                       //
		if (state[0]!=3)                    //
		{                                   //
			state[0]=3;                     //
			if (display ==1)                //
			{                               //
				displaymode1();             //
			}                               //
			else                            //
			{                               //
				displaymode0();             //
			}                               //
		}                                   //
	}	                                    //
	else if (state[0]==3)                   //
	{                                       //
		state[0]=1;                         //
	}	                                    //*

	if (((PINC >> VALVE) & 1) == 0){state[3]=1;} //Wattering pump switch
	else{state[3]=0;}                            //*
		
	if (((PINC >> PUMP) & 1) != 0){state[2]=0;} //Pump switch
	else{state[2]=1;}                           //*
}


