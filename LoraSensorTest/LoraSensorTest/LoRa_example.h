/*
 * LoRa_example.h
 *
 * Created: 12-Apr-18 11:57:52
 *  Author: MBJ
 */ 


#ifndef LORA_EXAMPLE_H_
#define LORA_EXAMPLE_H_


#define FIFO 0x00 //fifo register 8 bits

#define RegOpMode 0x01 //mode selection register bit 7 set to 1 is LoRa mode

#define RegFrMsb 0x06 // MSB of RF frequency
#define RegFrMid 0x07 // MSB of RF frequency
#define RegFrLsb 0x08 //LSB of RF frequency determines the carrier frequence default is 0x6C8000 for the three registers 
#define RegPaConfig 0x09 // bit 0-3 = Output Power
#define RegPaRamp 0x0A // rise/fall of ramp

#define RegOcp 0x0B // overload current protection
#define RegLna 0x0C // LNA gain

#define RegFifoAddrPtr 0x0D // SPI interface address pointer in FIFO buffer
#define RegFifoTxBaseAddr 0x0E //Write base address in FIFO
#define RegFifoRxBaseAddr 0x0F //read base address in FIFO
#define RegFifoRxCurrentAddr 0x10 //start address in buffer or last packet recieved

#define RegIrqFlagsMask 0x11 //various interrupt settings
#define RegIrqFlags 0x12 //interrupt flags logic '1' clears flag

#define RegRxNbBytes 0x13 //number of payload bytes of latest packet.
#define RegRxHeaderCntValueMsb 0x14 //number of valid headers recieved since last RX mode
#define RegRxHeaderCntValueLsb 0x15 //number of valid headers recieved since last RX mode
#define RegRxPacketCntValueMsb 0x16 //number of valid packets recieved since last RX mode
#define RegRxPacketCntValueLsb 0x17 // number of valid packets recieved since last RX mode
  
#define RegModemStat 0x18 //modemstatus register
#define RegPktSnrValue 0x19 // snr ratio estimation of last packet
#define RegPktRssiValue 0x1A //RSSI value of latest packet
#define RegRssiValue 0x1B //Current RSSI value
#define RegHopChannel 0x1C //value of hopping channel, PLL lock and CRC on/off

#define RegModemConfig1 0x1D //bit 0= explicit/implicit header mode bit 1-3 coding rate bit 4-7 bandwidth
#define RegModemConfig2 0x1E //bit 0-1 RX timeout MSB bit. 2=CRC on/off. bit 3=continuousmode/single packet. bit 4-7= spreadingfactor

#define RegSymbTimeoutLsb 0x1F //rx timout expressed number of symbols as timeout= Symbtimeout * Ts
#define RegPreambleMsb 0x20 //preamble lenght MSB = length + 4.25 symbols
#define RegPreambleLsb 0x21 // preamble LSB
#define RegPayloadLength 0x22 //payload lenght in bytes. only needed in implicit header mode
#define RegMaxPayloadLength 0x23 //max payload length. if header payload exceeds CRC error occur
#define RegHopPeriod 0x24 // periods between hops

#define RegFifoRxByteAddr 0x25 // current value of RX buffer pointer
#define RegModemConfig3 0x26 // bit 2= agc on/off. bit 3= low datarate enable.
#define PpmCorrection 0x27 // datarate offset with AFC

#define RegFeiMsb 0x28 //estimated frequency error from modem
#define RegFeiMid 0x29
#define RegFeiLsb 0x2A

#define RegRssiWideband 0x2C //RSSI wideband measurment to generate random number

#define RegDetectOptimize 0x31 //bit 0-2= LoRa detection optimize for spreading factors
#define RegInvertIQ 0x33 // bit 6 = invert Lora I and Q signals '0'=normal
#define RegDetectionThreshold 0x37 // LoRa detection threshold
#define RegSyncWord 0x39 //LoRa sync word 0x34 is reserved for LoRaWAN

#define RegDioMapping1 0x40 //mapping of pins DIO0 to DIO3
#define RegDioMapping2 0x41 //mapping of pins DIO4 and DIO5

#define RegVersion 0x42 // semtech ID silicon rev

#define RegTcxo 0x4B //TCXO or XTAL settings
#define RegPaDac 0x4D //Higher power settings of PA
#define RegFormerTemp 0x5B //stored temp during IQ calibration

#define RegAgcRef 0x61 // adjustment of agc
#define RegAgcThresh1 0x62
#define RegAgcThresh2 0x63
#define RegAgcThresh3 0x64
#define RegPll 0x70 //control of pll bandwidth

#define RegPacketConfig1 0x30
#define RegNodeAddress 0x33
#define RegBroadcastAddress 0x34

#define LoRa_sleep 0x00
#define LoRa_standby 0x01
#define LoRa_fstx 0x02
#define LoRa_tx 0x03
#define LoRa_fsrx 0x04
#define LoRa_rxcont 0x05
#define LoRa_rxsingle 0x06
#define LoRa_cad 0x07

#define Low_Frq_Mode_on 0x08
#define Low_Frq_Mode_off 0x00

#define LoRa_shareddreg_on 0x40
#define LoRa_sharedreg_off 0x0

#define LoRa_mode 0x80
#define FSK_mode 0x0

#define NTC_default_regOp 0x88
#define NTC_standby_regOp 0x89

#define DioMapRX 0x40
#define DioMapTX 0x01


//transmit and recieve buffer from LoRa Radio
typedef struct TRX_buffer{
	unsigned char *array;
	size_t used;
	size_t size;
}TRX_buffer;

TRX_buffer* init_struct_trx_buffer(void);
void insert_array(TRX_buffer *trxbuffer, unsigned char data);
void free_array(TRX_buffer *trx_buffer);

void SPI_sx1278_setup(void);
unsigned char SPI_LoRa_Read(unsigned char addr);
void SPI_LoRa_Write(unsigned char addr, unsigned char data);
int status_check_bit(unsigned char addr, int bitPos);
void clear_status(void);
void SLEEP_mode(void);
int TX_mode(void);
void STANDBY_mode(void);
void RXCONT_mode(void);
void r_FIFO(TRX_buffer *rx_buffer);
void w_FIFO(TRX_buffer *payload);
void sx1278_LoRa_mode(void);
void setTxPower(int level);
int packetRssi(void);
float packetSnr(void);
void setSpreadingFactor(int sf);
void setSignalBandwidth(int bw);
void enableCrc(void);
void disableCrc(void);
void sx1278_CAD_mode(void);
void sx1278_TX_mode(void);
void sx1278_RXsingle_mode(void);
void sx1278_RXcont_mode(void);
void sx1278_Set_node_address(unsigned char nodeAddress);
unsigned char sx1278_Read_node_address(void);


#endif /* LORA_EXAMPLE_H_ */