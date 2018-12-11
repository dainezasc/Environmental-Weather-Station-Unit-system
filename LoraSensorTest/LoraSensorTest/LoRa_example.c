/*
 * LoRa_example.c
 *
 * Created: 12-Apr-18 11:57:37
 *  Author: MBJ
 */ 
 /*
 * SX1278.c
 *
 * Created: 07-Feb-18 11:31:03
 *  Author: MBJ
 */ 
 #define F_CPU 8000000UL
 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <stdlib.h>
 #include <util/delay.h>
 
 #include "LoRa_example.h"
 
 
 #define DEBUG_MESSAGES 0 //turn on debug messages = 1, turn them off = 0

 //initialize buffer. first step before inserting anything.
TRX_buffer* init_struct_trx_buffer(void){
	TRX_buffer *trx_buffer = malloc(sizeof(TRX_buffer));
	trx_buffer->array = malloc(4 * sizeof(unsigned char));
	trx_buffer->size = 4;
	trx_buffer->used = 0;
	return trx_buffer;
}
//inserts a byte of data in the buffer and dynamically allocates the buffer
void insert_array(TRX_buffer *trxbuffer, unsigned char data){
	if(trxbuffer->used == trxbuffer->size){
		trxbuffer->size *= 2;
		trxbuffer[0].array = realloc(trxbuffer[0].array, trxbuffer->size * sizeof(unsigned char));
	}
	trxbuffer[0].array[trxbuffer->used] = data;
	trxbuffer->used++;
}
//use this to free the memory after w_fifo has been called
void free_array(TRX_buffer *trx_buffer){
	free(trx_buffer->array);
	trx_buffer->array = NULL;
	trx_buffer->used = trx_buffer->size = 0;
}
 unsigned char SPI_LoRa_Read(unsigned char addr){
		
	PORTB &= ~(1<<PINB2);
	asm ("nop");
	SPDR = addr;
	asm ("nop");	
	while (!(SPSR &(1<<SPIF)));
	SPDR = 0x00;
	asm ("nop");	
	while (!(SPSR &(1<<SPIF)));	
	PORTB |=(1<<PINB2);
	return SPDR;
 }
 void SPI_LoRa_Write(unsigned char addr, unsigned char data){
	PORTB &= ~(1<<PINB2);
	addr |= 0x80;
	SPDR = addr;
	asm ("nop");
	while (!(SPSR &(1<<SPIF)));	
	SPDR = data;
	asm ("nop");
	while (!(SPSR &(1<<SPIF)));
	PORTB |=(1<<PINB2);
 }
 
 int status_check_bit(unsigned char addr, int bitPos){
	 int bit = 0;
	 unsigned char status = 0x00;
	 //int i = 3;
	 //while(!bit && i != 0){
	 status = SPI_LoRa_Read(addr);
	 bit = status &(1<<bitPos);
	 //i--;
	 //_delay_ms(200);
	 //}
	 return bit == 0 ? 0 : 1;
 }
void clear_status(void){
	unsigned char status;
	status = SPI_LoRa_Read(RegIrqFlags);
	if(status != 0){
		SPI_LoRa_Write(RegIrqFlags, 0xFF);
	}	
 }
 int TX_mode(void){
	unsigned char FifoPtr;

	/*if(DEBUG == 1){
		clLnDisplay(2);
		strFont5XY("INIT TX MODE",1,2);
	}*/		

	FifoPtr = SPI_LoRa_Read(RegFifoRxBaseAddr);
	SPI_LoRa_Write(RegFifoAddrPtr, FifoPtr);
	
	SPI_LoRa_Write(RegIrqFlags, 0xFF);//clear irq flags
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp | LoRa_fsrx);
	_delay_ms(1);
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp | LoRa_cad);

	if(SPI_LoRa_Read(RegOpMode) != 0x8F){
		/*if(DEBUG == 1){
			clLnDisplay(3);
			strFont5XY("CAD MODE FAILED",1,3);
		}*/
		
	}
	/*if(DEBUG == 1){
		clLnDisplay(3);
		strFont5XY("CAD MODE",1,3);
	}*/		

	while(status_check_bit(RegIrqFlags,2) != 1);
	
	if(status_check_bit(RegIrqFlags,0) != 0){
		/*if(DEBUG == 1){
			clLnDisplay(4);
			strFont5XY("CAD DETECTED EXIT TX",1,4);
		}*/		
		SPI_LoRa_Write(RegIrqFlags, 0xFF);
		return 2;
	}	

	/*if(DEBUG == 1){
		clLnDisplay(4);
		strFont5XY("CAD DONE",1,4);
		
		clLnDisplay(5);
		strFont5XY("NO CAD",1,5);
	}*/	
	
	SPI_LoRa_Write(RegIrqFlags, 0xFF);

	FifoPtr = SPI_LoRa_Read(RegFifoTxBaseAddr);
	SPI_LoRa_Write(RegFifoAddrPtr, FifoPtr);	
	
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp | LoRa_tx);

	/*if(DEBUG == 1){
		clLnDisplay(2);
		strFont5XY("TX MODE",1,2);
	}*/	
			 
	while(status_check_bit(RegIrqFlags,3) != 1);

	SPI_LoRa_Write(RegIrqFlags, 0xFF);
	/*if(DEBUG == 1){
		clLnDisplay(6);
		strFont5XY("TX DONE",1,6);
	}*/	
	return 1;		 
 }
 void SLEEP_mode(void){
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp);
 }
 void STANDBY_mode(void){
	SPI_LoRa_Write(RegOpMode, NTC_standby_regOp);
	while(SPI_LoRa_Read(RegOpMode) != 0x89);
 }
 void RXCONT_mode(void){
	unsigned char FifoPtr;
	
	SPI_LoRa_Write(RegOpMode, NTC_standby_regOp);

	FifoPtr = SPI_LoRa_Read(RegFifoRxBaseAddr);
	SPI_LoRa_Write(RegFifoAddrPtr, FifoPtr);
	
	SPI_LoRa_Write(RegIrqFlags, 0xFF);//clear irq flags	
	
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp | LoRa_rxcont);

	while(SPI_LoRa_Read(RegOpMode) != 0x8D);
 }
 void r_FIFO(TRX_buffer *rx_buffer){
	cli();	

	unsigned char FifoPtr;
	unsigned char addr;
	unsigned char bytesRecieved;
	unsigned char data_byte;		

	SPI_LoRa_Write(RegOpMode, NTC_standby_regOp);

	while(SPI_LoRa_Read(RegOpMode) != 0x89);	

	FifoPtr = SPI_LoRa_Read(RegFifoRxCurrentAddr);
	SPI_LoRa_Write(RegFifoAddrPtr, FifoPtr);

	bytesRecieved = SPI_LoRa_Read(RegRxNbBytes);	
	
	if(status_check_bit(RegIrqFlags,4) == 1){
		
		if(status_check_bit(RegIrqFlags,5) != 1){			

			PORTB &= ~(1<<PINB2);
			addr = FIFO;
			SPDR = addr;
			asm ("nop");
			while (!(SPSR &(1<<SPIF)));			
			
			for(int i = 0 ; i < bytesRecieved; i++){
				SPDR = 0x00;
				asm ("nop");
				while (!(SPSR &(1<<SPIF)));
				data_byte = SPDR;
				insert_array(rx_buffer, data_byte);				
			}		
			PORTB |=(1<<PINB2);
			SPI_LoRa_Write(RegIrqFlags, 0xFF);
			return;
		}
		else{
			return;
		}
	}
	else{
		return;
	}				
 }
 void w_FIFO(TRX_buffer *payload){
	cli();
	unsigned char FifoPtr;
	unsigned char addr;
	unsigned char lenght;		
		
	//strFont5XY("W FIFO FUNC ",1,3);
	
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp); // clear FIFO
	

	SPI_LoRa_Write(RegOpMode, NTC_standby_regOp); // standby mode
	while(SPI_LoRa_Read(RegOpMode) != 0x89);
			
	FifoPtr = SPI_LoRa_Read(RegFifoTxBaseAddr);
	SPI_LoRa_Write(RegFifoAddrPtr, FifoPtr);		
	
	for(int i = 0; i < payload->used; i++){
		PORTB &= ~(1<<PINB2);
		addr = FIFO | 0x80;
		SPDR = addr;
		asm ("nop");
		while (!(SPSR &(1<<SPIF)));

		SPDR = payload[0].array[i];
		asm ("nop");
		while (!(SPSR &(1<<SPIF)));
		PORTB |=(1<<PINB2);		
	}
	
	//strFont5XY("W FIFO FINISH",1,4);
	lenght = payload->used;
	SPI_LoRa_Write(RegPayloadLength, lenght);		
 }

 unsigned char bytes_recieved(void){
 return SPI_LoRa_Read(RegRxNbBytes);
 }

 void sx1278_LoRa_mode(void)
 {	
	unsigned char status= 0x00;

	while(status_check_bit(RegOpMode,0) != 1);

	SPI_LoRa_Write(RegOpMode, LoRa_sleep | Low_Frq_Mode_on);
	status = SPI_LoRa_Read(RegOpMode);
	if(status == 0x08){
		SPI_LoRa_Write(RegOpMode, NTC_default_regOp);
		while(SPI_LoRa_Read(RegOpMode) != 0x88);

		SPI_LoRa_Write(RegOpMode, NTC_standby_regOp);
		while(SPI_LoRa_Read(RegOpMode) != 0x89);

		SPI_LoRa_Write(RegLna, 0x20);
		SPI_LoRa_Write(RegPaConfig, SPI_LoRa_Read(RegPaConfig) | 0x80);//set PA_BOOST pin, RFO pin probably not wired internally on sx1278
		SPI_LoRa_Write(RegModemConfig3, 0x04);//auto AGC		
		SPI_LoRa_Write(RegPreambleLsb, 0xFF);//preamble length
		SPI_LoRa_Write(RegIrqFlags, 0xFF);
	}
	else{
		//strFont5XY("LoRa MODE FAILED",1,7);
	}	
 }
 void setTxPower(int level)
 {//level between 0 and 17 dBm
	//RFO is not connected to antenna output
	/*if (status_check_bit(RegPaConfig, 7) == 0) {
		// RFO
		if (level < 0) {
			level = 0;
		} 
		else if (level > 14) {
			level = 14;
		}
		SPI_LoRa_Write(RegPaConfig, 0x70 | level);
	} */
	
	// PA BOOST
	if (level < 2) {
		level = 2;
	} 
	else if (level > 17) {
		level = 17;
	}
	SPI_LoRa_Write(RegPaConfig, 0x80 | (level - 2));
	
 }
 int packetRssi(void)
 {
	 return (SPI_LoRa_Read(RegPktRssiValue) - 164);
 }

 float packetSnr(void)
 {
	 return ((int8_t)SPI_LoRa_Read(RegPktSnrValue)) * 0.25;
 }
 void setSpreadingFactor(int sf)
 {
	if (sf < 6) {
		sf = 6;
	} 
	else if (sf > 12) {
	sf = 12;
	}

	if (sf == 6) {
		SPI_LoRa_Write(RegDetectOptimize, 0xc5);
		SPI_LoRa_Write(RegDetectionThreshold, 0x0c);
		} 
		else {
		SPI_LoRa_Write(RegDetectOptimize, 0xc3);
		SPI_LoRa_Write(RegDetectionThreshold, 0x0a);
	}
	 SPI_LoRa_Write(RegModemConfig2, (SPI_LoRa_Read(RegModemConfig2) & 0x0f) | ((sf << 4) & 0xf0));
 }
 void setSignalBandwidth(int bw){
	//set bandwidth values are 0=7.8KHz,1=10.4KHz,2=15.6KHz,3=20.8KHz,4=31.25KHz,5=41.7KHz,6=62.5KHz,7=125KHz,8=250KHz,9=500KHz
	if(bw>=0 && bw<=9){
		SPI_LoRa_Write(RegModemConfig1, (SPI_LoRa_Read(RegModemConfig1) & 0x0f) | (bw << 4));
	}	
 }
 void enableCrc(void)
 {
	 SPI_LoRa_Write(RegModemConfig2, SPI_LoRa_Read(RegModemConfig2) | 0x04);
 }

 void disableCrc(void)
 {
	 SPI_LoRa_Write(RegModemConfig2, SPI_LoRa_Read(RegModemConfig2) & 0xfb);
 }
 void sx1278_CAD_mode(void){
	//SPI_LoRa_Write(RegOpMode, LoRa_standby);
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp | LoRa_cad);
	
 }
 void sx1278_TX_mode(void){
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp | LoRa_tx);	
	
 }
 void sx1278_RXsingle_mode(void){
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp | LoRa_rxsingle);
 }
 void sx1278_RXcont_mode(void){
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp | LoRa_rxcont);
 }
 void sx1278_Set_node_address(unsigned char nodeAddress){
	SPI_LoRa_Write(RegOpMode, NTC_default_regOp | LoRa_shareddreg_on);
	while(SPI_LoRa_Read(RegOpMode) != 0xC8);
	
	SPI_LoRa_Write(RegNodeAddress, nodeAddress);
	SPI_LoRa_Write(RegOpMode, NTC_standby_regOp);
	while(SPI_LoRa_Read(RegOpMode) != 0x89);

	//node_address = nodeAddress;
 }
 unsigned char sx1278_Read_node_address(void){
	unsigned char nodeAddress;
	SPI_LoRa_Write(RegOpMode, LoRa_shareddreg_on);
	while(SPI_LoRa_Read(RegOpMode) != 0xC8);

	nodeAddress = SPI_LoRa_Read(RegNodeAddress);
	SPI_LoRa_Write(RegOpMode, NTC_standby_regOp);
	while(SPI_LoRa_Read(RegOpMode) != 0x89);

	return nodeAddress;
 }