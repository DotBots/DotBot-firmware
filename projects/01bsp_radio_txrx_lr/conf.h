#include <nrf.h>
#include "radio.h"

#define DOTBOT_GW_RADIO_MODE DB_RADIO_BLE_LR125Kbit
#define DELAY_MS             (100)  // Wait 100ms between each send
#define CHANNEL              (22)   // 2450 MHz for BLE
