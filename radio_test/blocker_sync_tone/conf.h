#include <nrf.h>

#define RADIO_MODE 3
#define FREQUENCY 8

#define START_DELAY  150  // 150 to block BLE / 200 to block 802.15.4
#define END_DELAY  1800   // 1800 to block BLE / 4000 to block 802.15.4

#define PAYLOAD_SIZE 1  // Not use

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif

#define POWER RADIO_TXPOWER_TXPOWER_0dBm
