#include <nrf.h>

#define RADIO_MODE 2
#define BLOCKER_FREQUENCY 8

#define START_DELAY  80  
#define END_DELAY  6500 
#define PAYLOAD_SIZE 125  

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif

#define POWER RADIO_TXPOWER_TXPOWER_0dBm
