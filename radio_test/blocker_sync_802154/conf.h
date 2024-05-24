#include <nrf.h>

#define RADIO_MODE 2
#define FREQUENCY 8

#define START_DELAY  85  // 85 to block BLE / 90 to block 802.15.4
#define END_DELAY  8000 
#define PAYLOAD_SIZE 16  // 16 to block BLE / 80 to block 802.15.4 

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif

#define POWER RADIO_TXPOWER_TXPOWER_0dBm
