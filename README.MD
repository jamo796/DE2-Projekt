# PROJEKT DE-2

### Téma projektu:

Měření výšky vodní hladiny v nádrži s využitím senzorů a Arduina Uno

#### Členové:


- Vojtěch Niederle
- Jan Pelka
- Filip Nimrichter



### Cíle Projektu:

- Kontrola Stavu Hladiny Vody a její doplňování.
- Kontrola Teploty.
- Zavlažování 1x za 24 Hod.



### Popis Funkcionality:

Náš projekt je dizajnován jako projekt který má obsluhovat zahrádku a má obsluhovat zavlažování záhonků zavlažovat je potřeba 1x za 24 hod a je potřeba hlídat jestli je ve vodní nádrži (tanku) dostatek vody, předpokládá se že nádrž je napojená na okap a sbírá tak dešťovou vodu.V případě nedostatku vody v nádrži se automaticy zapne čerpadlo. Dále je potřeba kontrolovat teplotu aby se nezalévalo vodou o teplotách které by mohly rostliny poškodit. Obě tyto funkce se dají vypnout pomcí přepínačů.



### Popis HW:

Celý systém je postaven na  [Arduinu verze UNO](https://store.arduino.cc/products/arduino-uno-rev3/) ke kterému jsou připojeny periferie jako [Display](https://www.sparkfun.com/datasheets/LCD/HD44780.pdf), [ultrazvukový senzor hladiny](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf) a [teplotní senzor](https://www.gme.cz/termistor-ntc-b57164k0103k). K arduinu jsou připojeny 2 relé kde jedno otevírá výpusť na zalévání zahrady a druhé spíná čerpadlo které doplňuje vodu do nádrže. Stavy jsou vybaveny Kontrolními LED - diodami. K adruinu jsou dále připojené spínače které přepínají mezi režimy zobrazování na display, zapnutí zpracování teploty, zapnutí čerpadla a zdali je zapnuté zavlažování.

![Obvod](https://github.com/jamo796/DE2-Projekt/blob/main/img/Obvod.jpg)

Termistorový senzor:
Jako senor teploty jsme použily Termistor který má negativní odporovou charakteristiku (s tostoucí teplotou klesá odpor)
tuto charakteristiku kterou poskytnul výrobce jsme vynesli do grafu a graf jsme proložil spojnicí trendu (polynom 3. stupně) 
která nám řekla jakou rovnicí máme popisovat teplotu v závislosti na  úbytku napětí na termistoru

v simulaci jsme tento termistor řešily napěťovým zdrojem který nám posloužil jako 

v zapojení jsme použily odpový dělič viz obrázek

![DELIC](img/DELIC2.jpg) ![graf](https://github.com/jamo796/DE2-Projekt/blob/main/img/GRAF2.jpg)




Režim Displeje 1:

Zobrazuje Přímo vzdálenost hladiny v cm a teplotu v °C

![display1](https://github.com/jamo796/DE2-Projekt/blob/main/img/display1.jpg)



Režim Displeje 2:

Zobrazuje stavy slovně (Filling, Normal, Full)

- Filling = Hladina vody je nízká sepne se čerpadlo

- Normal = Hladina je v normálních hodnodách

- Full = Hladina Vody je vysoká (Upozorňuje obsluhu na možnost vylítí)

Temp ON/Off vypníná či zapíná funkci kontroly teploty (ale teplota se měří a zobrazuje stále)
- ON = Kontrola je zapnutá pokud je teplota příliš nízká nebo vysoká vypne se automatické zavlažování
- OFF = Teplota se měři a zobrazuje na display ale neovlivňuje funkci zavlažování 

![Display2](https://github.com/jamo796/DE2-Projekt/blob/main/img/display2.jpg)



Hladina vody:

- Pokud je Hladina vody 280 cm od senzoru tak se považuje hladina vody za nízkou

- Pokud je Hladina vody 20 od senzoru tak se považuje za vysokou 





### Popis Knihoven:

Knihovna pro obsluhu LCD displeje:

http://www.peterfleury.epizy.com/avr-software.html?i=2

tato knihovna obsluhuje LCD display jako kam se má postavit kurzor jestli se má display zapnout nebo vypnout a nebo jaký symbol nebo textový řetězec má zobrazit

| Function Name | Function parameters | Description | Example |
| :-- | :-- | :-- | :-- |
| `lcd_init` | `LCD_DISP_OFF/on/on_cursor/on_cursor/blink` | Dysplay off | `lcd_init(LCD_DISP_ON_CURSOR_BLINK)` |
| `lcd_clrscr` | non | clear display and set cursor to home | `lcd_clrscr()` |
| `lcd_gotoxy` | uint8_t x , uint8_t y | set curson on position | `lcd_gotoxy(1,1)` |
| `lcd_putc` | char c | Display charakter and push cursor | `lcd_putc("A")` |
| `lcd_puts` | const char * s | display string and push cursor | `lcd_puts("ok")` |
| `lcd_command` | uint8_t cmd | send LCD controller instruction command | |
| `lcd_data` | uint8_t data | similar as lcd_putc(), without LF | |




Knihovna časovač:

https://github.com/tomas-fryza/Digital-electronics-2/blob/master/Labs/library/include/timer.h


???Veškeré knihovny byly vytvořeny na počítačových cvičeních předmětu BPC-DE2.Většina z těchto knihoven výchází z
knihoven pana
doc. Ing.Tomáše Frýzy,Ph.D odkza na github [zde](https://github.com/tomas-fryza/Digital-electronics-2/).
???

Knihovny byly použity ty které byly vytvořeny v Laboratorní Výuce dokumentaci k nim naleznete na [Repozitáři](https://github.com/tomas-fryza/Digital-electronics-2/) Pana Docenta Tomáše Frýzi 







### PROGRAM:

[Vývojový diagram](https://github.com/jamo796/DE2-Projekt/blob/main/DE2.pdf)


​	

```c
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
else if (depth>280){state[1] = 1;}   // 3 = EMPTY ; 2 = NORMAL ; 1 = FULL
else{state[1] = 2;}                  //
```

Ve výše vypsaném kódu si můžeme všimnout že hladinu měříme dobu trvání log. H na výstupu ultrazvukového senzoru (pin ECHO) a následně tento čas matematicky zpracováváme aby jsme dostaly výsledek v cm.

Toto měření Je prováděno jednou za čas (1 s) a je voláno metodou přerušení. Výsledky tohoto měření jsou přenášeny na Display 

tento senzor je připojen na piny arduina B4 a B5





```c
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
```

v této ukázce kódu si můžeme všimnout matematického Zpracování teploty které je popisováno polynomicky a to polynomem 3. stupně

Napětí na Termistoru měřime a analogově ho zpracováváme na A/D převodníku který integrován v mikro-kontroléru na desce Aruina  

měření je opět řešeno přerušením a výstup je opět interpretován na display

Výstupní jednotka je v °C 



### VIDEO: (staňme se YT-mrdky OUYEAH)

https://www.youtube.com/watch?v=rnSZW_Kkyh0



### REFERENCE:

knihovny z předešlých laboratorních cvičení

Datasheet: [ARDUINO ATMEGA328P](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)

senzor ultrazvuk: [ultrazvukový senzor hladiny](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)

datasheet termistoru: [Termistor](https://www.gme.cz/termistor-ntc-b57164k0103k)
