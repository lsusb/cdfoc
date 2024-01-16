#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "std_config.h"

void usart_decode(u8* buffer);
void usart_userCMD_decode(u8* buffer);
void can_rx_decode(u8* buffer,u8 can_id);
void can_tx_encode(u8* buffer);



#endif
