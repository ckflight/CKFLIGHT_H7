
#include <COMMON/maths.h>
#include "DRIVERS/CK_UART.h"
#include "DRIVERS/CK_CIRCULARBUFFER.h"

#include "FLIGHT/CK_SBUS.h"


SBUS_Method sbus_method;

circularBuffer_t sbus_cb;

uint8_t sbus_buffer[SBUS_PACKET_SIZE];
uint8_t buffer_index = 0;

uint8_t sbus_step;
uint16_t sbus_receive_error;
uint8_t sbus_receive_counter;

bool sbus_dataReceived;

int _channels[19];

USART_TypeDef* CK_SBUS_UART;

void CK_SBUS_Init(SBUS_Method method){

	sbus_method = method;

	int sbus_baud = 100000;

	USART_CONFIGURATION_ config;
	config.interrupt 			= RX_INTERRUPT;
	config.mode 				= RX_ONLY;
	config.parity 				= PARITY_EVEN;
	config.stop_bit 			= STOP_BIT2;
	config.baudrate 			= sbus_baud;
#if SBUS_
	config.rx_gpio_type			= SBUS_UART_RX_GPIO;
	config.rx_gpio_pin			= SBUS_UART_RX_PIN;
	config.rx_af				= SBUS_UART_RX_AF;
	config.usart 				= SBUS_UART;
	CK_SBUS_UART 				= SBUS_UART;
#endif
	config.use_circular_buffer 	= true;

	CK_UART_Init(&config, &sbus_cb);

	CK_UART_RXInterruptEnable(CK_SBUS_UART);
	CK_UART_RXEnable(CK_SBUS_UART);
}

bool CK_SBUS_Update(uint32_t current_time){

	bool isNewData = false;
    /*
     * Now sbus is updated with interrupt.
     * No need to check periodically as soon as new arrival of data
     * State machine places it in buffer and if packet is arrived raises a ready flag.
     */
    if(sbus_method == SBUS_INTERRUPT){

    	if(sbus_dataReceived){

    		CK_SBUS_Decode();

    		isNewData = true;
        }
    }

    /*
     * This method is first classic method.
     * At every SBUS_CHECK_INTERVAL it uses a while loop to find sbus packet and decodes
     * It takes around 35usec which is not so fast.
     */
    else if(sbus_method == SBUS_POLLING){

    	static uint32_t previousTime = 0;

        if(current_time >= previousTime + SBUS_CHECK_INTERVAL){

            previousTime = current_time;
            CK_SBUS_Process();

            isNewData = true;

        }
    }

    return isNewData;

}

void CK_SBUS_Process(void){

    while(!CK_CIRCULARBUFFER_IsBufferEmpty(&sbus_cb)){
        uint8_t rx;
        CK_CIRCULARBUFFER_BufferRead(&sbus_cb, &rx);
        if (buffer_index == 0 && rx != SBUS_STARTBYTE) {
            continue; // incorrect start byte, out of sync go to while with next iteration
        }

        sbus_buffer[buffer_index++] = rx;

        if (buffer_index == SBUS_PACKET_SIZE){
            buffer_index = 0;
            if (sbus_buffer[24] != SBUS_ENDBYTE) {
                continue; // incorrect start byte, out of sync go to while with next iteration
            }
            CK_SBUS_Decode();
        }
    }
}

void CK_SBUS_Decode(void){

    _channels[0]  = ((sbus_buffer[1]      | sbus_buffer[2]  << 8)                        & 0x07FF);
    _channels[1]  = ((sbus_buffer[2]  >>3 | sbus_buffer[3]  << 5)                        & 0x07FF);
    _channels[2]  = ((sbus_buffer[3]  >>6 | sbus_buffer[4]  << 2  | sbus_buffer[5]<<10)  & 0x07FF);
    _channels[3]  = ((sbus_buffer[5]  >>1 | sbus_buffer[6]  << 7)                        & 0x07FF);
    _channels[4]  = ((sbus_buffer[6]  >>4 | sbus_buffer[7]  << 4)                        & 0x07FF);
    _channels[5]  = ((sbus_buffer[7]  >>7 | sbus_buffer[8]  << 1  | sbus_buffer[9]<<9)   & 0x07FF);
    _channels[6]  = ((sbus_buffer[9]  >>2 | sbus_buffer[10] << 6)                        & 0x07FF);
    _channels[7]  = ((sbus_buffer[10] >>5 | sbus_buffer[11] << 3)                        & 0x07FF);
    _channels[8]  = ((sbus_buffer[12]     | sbus_buffer[13] << 8)                        & 0x07FF);
    _channels[9]  = ((sbus_buffer[13] >>3 | sbus_buffer[14] << 5)                        & 0x07FF);
    _channels[10] = ((sbus_buffer[14] >>6 | sbus_buffer[15] << 2  | sbus_buffer[16]<<10) & 0x07FF);
    _channels[11] = ((sbus_buffer[16] >>1 | sbus_buffer[17] << 7)                        & 0x07FF);
    _channels[12] = ((sbus_buffer[17] >>4 | sbus_buffer[18] << 4)                        & 0x07FF);
    _channels[13] = ((sbus_buffer[18] >>7 | sbus_buffer[19] << 1  | sbus_buffer[20]<<9)  & 0x07FF);
    _channels[14] = ((sbus_buffer[20] >>2 | sbus_buffer[21] << 6)                        & 0x07FF);
    _channels[15] = ((sbus_buffer[21] >>5 | sbus_buffer[22] << 3)                        & 0x07FF);

    if(sbus_buffer[23] & 0x0001) _channels[16] = 2047;
    else _channels[16] = 0;

    if((sbus_buffer[23] >> 1) & 0x0001) _channels[17] = 2047;
    else _channels[17] = 0;

    //Channel 19 is FAILSAFE FLAG 18 means Channel 19
    if ((sbus_buffer[23] >> 3) & 0x0001) _channels[18] = SBUS_FAILSAFE_ACTIVE;
    else _channels[18] = SBUS_FAILSAFE_INACTIVE;

    sbus_dataReceived = false;

}

void CK_SBUS_NewByte(uint8_t data){

    switch(sbus_step){
    case 0:
        sbus_receive_counter = 0;
        if(data == SBUS_STARTBYTE){
            sbus_buffer[sbus_receive_counter++] = data;
            sbus_step++;
        }
        break;
    case 1:
        if(sbus_receive_counter < SBUS_PACKET_SIZE){
            sbus_buffer[sbus_receive_counter++] = data;
        }
        if(sbus_receive_counter >= SBUS_PACKET_SIZE){
            if(sbus_buffer[SBUS_PACKET_SIZE-1] == SBUS_ENDBYTE){
                sbus_dataReceived = true;
            }
            sbus_step = 0;
        }
        break;
    }
}

bool CK_SBUS_IsReady(void){
    return sbus_dataReceived;
}

int CK_SBUS_GetChannelRaw(int channel){

	if(channel < 1 || channel > 19){
		return 0;
	}
	else{
		return _channels[channel - 1];
	}
}

#if SBUS_

#if SBUS_INTERRUPT_ == 1
void USART1_IRQHandler(void){
#endif

#if SBUS_INTERRUPT_ == 2
void USART2_IRQHandler(void){
#endif

#if SBUS_INTERRUPT_ == 3
void USART3_IRQHandler(void){
#endif

#if SBUS_INTERRUPT_ == 4
void UART4_IRQHandler(void){
#endif

#if SBUS_INTERRUPT_ == 5
void UART5_IRQHandler(void){
#endif

#if SBUS_INTERRUPT_ == 6
void USART6_IRQHandler(void){
#endif

	#if USE_H7 == 1
	if(CK_SBUS_UART->ISR & CK_USART_SR_RXNE){

		CK_SBUS_UART->ICR = 0xFFFFFFFF;

        uint8_t rxData = CK_SBUS_UART->RDR;

	#endif

	#if USE_F4 == 1
    if(CK_SBUS_UART->SR & CK_USART_SR_RXNE){
    	uint8_t rxData = CK_SBUS_UART->DR;
	#endif

        if(sbus_method == SBUS_INTERRUPT){
            CK_SBUS_NewByte(rxData);
        }
        else if(sbus_method == SBUS_POLLING){

            if(!CK_CIRCULARBUFFER_IsBufferFull(&sbus_cb)){
                CK_CIRCULARBUFFER_BufferWrite(&sbus_cb, rxData);
            }
        }
    }
}

#endif
