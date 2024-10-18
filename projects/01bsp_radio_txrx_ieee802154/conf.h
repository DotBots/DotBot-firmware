#include <nrf.h>
#include "radio.h"

#define DOTBOT_GW_RADIO_MODE DB_RADIO_IEEE802154_250Kbit
#define DELAY_MS             (100)  // Wait 100ms between each send
#define CHANNEL              (15)   // 2425 MHz for IEEE 802.15.4
