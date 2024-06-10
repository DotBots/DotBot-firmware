#include <nrf.h>

#define RADIO_MODE 3
#define BLOCKER_FREQUENCY 8

#define START_DELAY  80 
#define END_DELAY  5000   

#define PAYLOAD_SIZE 1  // Not use

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif

#define POWER RADIO_TXPOWER_TXPOWER_0dBm
