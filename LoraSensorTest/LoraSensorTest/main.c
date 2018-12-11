/*
 * LoraSensorTest.c
 *
 * Created: 11/27/2018 3:14:22 PM
 * Author : user
	* pin connection is as follows:
	*	PD2 = DHT22 data pin
	*	PB1 = RST on LoRa module
	*	PB2 = SPI CS on LoRa module
	*	PB3 = SPI MOSI
	*	PB4 = SPI MISO
	*	PB5 = SPI CLK
	*	PB6 = DI0 pin from LoRa
	*	PC0 = MQ135 analog data pin
	*	PC5 = Button input for TX interrupt
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h> //include libm
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "adc.h"
#include "STDIO_UART.h"
#include "DHT22int.h"
#include "mq.h"
#include "LoRa_example.h"

#define MQ_ADCCHANNEL 0 //mq data channel
#define MQ_PULLDOWNRES 22000 //MQ135 pulldown resistor
#define MQ_DEFAULTPPM 392 //MQ135 default ppm of CO2 for calibration
#define MQ_DEFAULTRO 41763 //MQ135 default sensor resistance Ro for CO2
#define MQ_SCALINGFACTOR 116.6020682 //MQ135 CO2 gas value / scaling factor based on sensitivity characteristics curve
#define MQ_EXPONENT -2.769034857 //MQ135 CO2 gas value / exponent based on sensitivity characteristics curve

//define max Rs/Ro interval
#define MQ_MAXRSRO 2.428 //MQ135 for CO2
//define min Rs/Ro interval
#define MQ_MINRSRO 0.358 //MQ135 for CO2

//define lookup number of points
#define MQ_LOOKUPTHSIZE 7
//define temperature lookup points
#define MQ_LOOKUPTHT {    -10,      0,     10,     20,     30,     40,     50 } //MQ135 dependance curve temperature points
//define temperature rs/ro lookup points on 1 humidity curve
#define MQ_LOOKUPTH1 { 1.7007, 1.3792, 1.1318, 0.9954, 0.9530, 0.9335, 0.9110 } //MQ135 dependance curve rs/ro over humidity 33% curve
//define temperature rs/ro lookup points on 2 humidity curve
#define MQ_LOOKUPTH2 { 1.5326, 1.2510, 1.0378, 0.9128, 0.8733, 0.8452, 0.8199 } //MQ135 dependance curve rs/ro over humidity 85% curve
//define temperature rs/ro lookup humidity 1 curve value
#define MQ_LOOKUPTH1HUMDVALUE 33 //MQ135 dependance curve humidity 33% curve
//define temperature rs/ro lookup humidity 2 curve value
#define MQ_LOOKUPTH2HUMDVALUE 85 //MQ135 dependance curve humidity 85% curve

//define reference temperature for ro as on sensitivity characteristics curve
#define MQ_SENREFTEMP 20 //MQ135 20C
//define reference humidity for ro as on sensitivity characteristics curve
#define MQ_SENREFHUMD 65 //MQ135 65%RH

ISR(PCINT0_vect);//rx done interrupt
ISR(PCINT1_vect);//tx button

int tx = 0;
int rx = 0;

int main(void)
{
	cli();
	
	//initialize uart
	ioinit();
	printf("uart works\n");

	//initialize adc
	adc_init();
	
	char printbuff[50];
	uint16_t adc = 0;
	long ro = 0;
	double ppm = 0;
	
	DHT22_STATE_t state;
	DHT22_DATA_t sensor_data;
	DHT22_Init();
	/*if (state == DHT_STOPPED)
	{
		printf("dht22 initialized\n");
	}*/
	
	int lookupthsize = MQ_LOOKUPTHSIZE;
	double lookuptht[] = MQ_LOOKUPTHT;
	double lookupth1[] = MQ_LOOKUPTH1;
	double lookupth2[] = MQ_LOOKUPTH2;
	double lookupth1humdvalue = MQ_LOOKUPTH1HUMDVALUE;
	double lookupth2humdvalue = MQ_LOOKUPTH2HUMDVALUE;
	int senreftemp = MQ_SENREFTEMP;
	int senrefhumd = MQ_SENREFHUMD;
	
	//Set MOSI, SCK, SS, and sx1278 RST as output, PINB6 (sx1278 DIO0 is the pin from LoRa which goes high when it has received something) as input
	DDRB |= (1<<PINB3) | (1<<PINB5) | (1<<PINB2) | (1<<PINB1) | (PINB6<<0);
	//PINC5(button for tx) as input
	DDRC |= (PINC5<<0);
	//Set SS and sx1278 RST to high, sx1278 SPI is enabled on SS LOW
	PORTB|= (1<<PINB2) | (1<<PINB1) | (PINB6<<0);
	//set pullup on button input
	PORTC|= (1<<PORTC5);
	
	//Enable master SPI at clock rate 8mHz/128 = 62.5KHz
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (CPOL<<0) | (CPHA<<0);
	// pin change interrupt enable pin on PCINT[0:7] and PCINT[8:14]
	PCICR |= (1<<PCIE0) | (1<<PCIE1);
	//pin change interrupt enable on PCINT6(PINB6)
	PCMSK0 |= (1<<PCINT6);
	//pin change interrupt enable on PCINT13(PINC5)
	PCMSK1 |= (1<<PCINT13);

	sei();
	
	_delay_ms(1000);		//delay to make sure the lora module is up and running before communication is started

	sx1278_LoRa_mode();		//setting the LoRa module to LoRa mode
	setSignalBandwidth(7);	//BW 7 = 125KHz
	setSpreadingFactor(9);	//Spreading factor of 9
	setTxPower(2);			//dBm power settings. go from 2-17
	enableCrc();
    
	//initDisplay();

	//RXCONT_mode();			//set LoRa radio to continuous receive mode
	
	
	//strFont5XY("LoRa TEST EXAMPLE",0,0);
    while (1) 
    {
		/***************************************/
		state = DHT22_StartReading();
		if (state != DHT_STARTED)
		{
			printf("DHT state machine busy\r");
			continue;
		}
		printf("reading...\r\n");

		//get temperature and humidity
		state = DHT22_CheckStatus(&sensor_data);

		if (state == DHT_DATA_READY){
			// Do something with the data.
			printf("temperature: %d.%d C\n", sensor_data.temperature_integral, sensor_data.temperature_decimal);
			printf("humidity: %d.%d %%\n\n", sensor_data.humidity_integral,sensor_data.humidity_decimal);
		}
		else if (state == DHT_ERROR_CHECKSUM){
			printf("DHT_ERROR_CHECKSUM \r");
			// Do something if there is a Checksum error
		}
		else if (state == DHT_ERROR_NOT_RESPOND){
			printf("DHT_ERROR_NOT_RESPOND \r");
			// Do something if the sensor did not respond
		}
		/***********************************/
		
		//get adc
		adc = adc_read(MQ_ADCCHANNEL);
		//calculated resistence depends on the sensor pulldown resistor
		long res = adc_getresistence(adc, MQ_PULLDOWNRES);
		double adcvol = adc_getvoltage(adc, 5.0);
		
		//get ro
		ro = mq_getro(res, MQ_DEFAULTPPM, MQ_SCALINGFACTOR, MQ_EXPONENT);
		//get ppm using DEFAULT_RO
		ppm = mq_getppm(res, MQ_DEFAULTRO, MQ_SCALINGFACTOR, MQ_EXPONENT, MQ_MAXRSRO, MQ_MINRSRO);
		
		//get ro T/H
		ro = mq_getrotemphumd(res, MQ_DEFAULTPPM, MQ_SCALINGFACTOR, MQ_EXPONENT, (int)sensor_data.temperature_integral, (int)sensor_data.humidity_integral, senreftemp, senrefhumd, lookupthsize, lookuptht, lookupth1, lookupth2, lookupth1humdvalue, lookupth2humdvalue);
		//get ppm using DEFAULT_ROTH
		ppm = mq_getppmtemphumd(res, MQ_DEFAULTRO, MQ_SCALINGFACTOR, MQ_EXPONENT, MQ_MAXRSRO, MQ_MINRSRO, (int)sensor_data.temperature_integral, (int)sensor_data.humidity_integral, senreftemp, senrefhumd, lookupthsize, lookuptht, lookupth1, lookupth2, lookupth1humdvalue, lookupth2humdvalue);
		
		/******************************/
		//print out values
		itoa(adc, printbuff, 10);
		printf("ADC "); printf(printbuff); printf("\r\n");
		ltoa(res, printbuff, 10);
		printf("RES "); printf(printbuff); printf("\r\n");
		dtostrf(adcvol, 3, 3, printbuff);
		printf("VOL "); printf(printbuff); printf("\r\n");
		
		//print out values
		ltoa(ro, printbuff, 10);
		printf("ro        "); printf(printbuff); printf("\r\n");
		dtostrf(ppm, 3, 5, printbuff);
		printf("ppm_roDef "); printf(printbuff); printf("\r\n");

		//print out values
		ltoa(ro, printbuff, 10);
		printf("roTH        "); printf(printbuff); printf("\r\n");
		dtostrf(ppm, 3, 5, printbuff);
		printf("ppm_roTHDef "); printf(printbuff); printf("\r\n");

		/***********************************/
		RXCONT_mode();			//set LoRa radio to continuous receive mode
		while(1) 
		{
			if(rx == 1){
				cli();										//disable interrupts to prevent data and communication corruption
				TRX_buffer *buffer;							//buffer for transfering data to and from the LoRa module
				buffer = init_struct_trx_buffer();			//initialize the struct as it is a pointer.
				r_FIFO(buffer);								//put data recived from the LoRa FIFO int to the buffer struct
			
				printf("lora data:");
				for (uint8_t count = 0; count <= sizeof(buffer->array)+1; count++) {
					printf("%d", buffer[0].array[count]); 
				}
				printf("\n");
			
			
				//do something with the data. this should be replaced unless you have a OLED display connected like my setup
				/*clsDisplay();
				byteFont5XY(buffer[0].array[0],0,0);
				byteFont5XY(buffer[0].array[1],0,1);
				byteFont5XY(buffer[0].array[2],0,2);
				byteFont5XY(buffer[0].array[3],0,3);
				byteFont5XY(buffer[0].array[4],0,4);
				byteFont5XY(buffer[0].array[5],0,5);									
				*/
				//after you have handled you data you should free the struct so it free's the memory.

				free_array(buffer);							//free the struct 
				RXCONT_mode();								//put radio in continuous receive mode
						
				sei();										//enable interrupts
				rx = 0;										//clear the rx var so we dont run the code again
				break;
			}
			if(tx == 1){
				cli();										//disable interrupts
				printf("tx mode on \n");
				TRX_buffer *buffer;							//struct for data
				buffer = init_struct_trx_buffer();			//initialize struct
				//inserting number 0 to 6 in buffer and transmitting
				/*for(unsigned char i = 0; i <= 6; i++){
					insert_array(buffer, i);
				}*/
				insert_array(buffer, sensor_data.temperature_integral);
				insert_array(buffer, sensor_data.temperature_decimal);
				insert_array(buffer, sensor_data.humidity_integral);
				insert_array(buffer, sensor_data.humidity_decimal);
				//remove this if you dont have OLED
				/*
				clsDisplay();
			
				strFont5XY("WRITE FIFO BEGIN",0,0);
				*/
				//********************************//
			
				w_FIFO(buffer);

				//remove this if you dont have OLED
				/*strFont5XY("WRITE FIFO END",0,1);
				*/
				//********************************//

				free_array(buffer);							//free array after it has been written to the LoRa module FIFO

				//remove this if you dont have OLED
				//strFont5XY("TX BEGIN",0,2);
				//*************************//

				TX_mode();									//put LoRa in TX mode after FIFO have been filled to transmit the data.

				//remove this if you dont have OLED
				/*strFont5XY("TX END",0,3);
				*/
				//***********************//

				RXCONT_mode();								//put LoRa back in continuous receive mode after TX is 
				tx = 0;										//reset tx interrupt var
				sei();										//enable interrupts
				break;
			}
			_delay_us(1);
		}
		printf("now reading sensor data...\n");
		//_delay_ms(2000);
    }
}

ISR(PCINT0_vect){	
	cli();
	rx = 1;												//set rx interrupt var
	
}
ISR(PCINT1_vect){
	cli();
	tx = 1;												//set tx interrupt var
	
}
