#include <nrf.h>

#define RADIO_MODE 1
#define FREQUENCY 8

#define START_DELAY  10  // 10 to block BLE / 5 to block 802.15.4
#define END_DELAY  4000 
#define PAYLOAD_SIZE 60  // 60 to block BLE / 240 to block 802.15.4 

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif

#define POWER RADIO_TXPOWER_TXPOWER_0dBm
