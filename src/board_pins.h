#pragma once

// RejsaCAN-ESP32-S3 v3.x pin map. Numbers come from the vendor's
// getall_s3.ino example.
#if defined(BOARD_REJSACAN)

#define CAN_TX_PIN       GPIO_NUM_14
#define CAN_RX_PIN       GPIO_NUM_13

// CAN_RS sets the transceiver mode: LOW = high-speed normal, HIGH =
// listen-only/low-power. Leaving it floating selects slope-control mode,
// which silently mangles 500 kbit/s frames. Always drive it.
#define CAN_RS_PIN       GPIO_NUM_38

// FORCE_ON holds the auto-shutdown circuit off so firmware decides when to
// power down rather than the hardware cutting us mid-capture.
#define FORCE_ON_PIN     GPIO_NUM_17

#define WARN_LED_PIN     GPIO_NUM_11   // yellow
#define ACTIVITY_LED_PIN GPIO_NUM_10   // blue

// The board's microSD slot. NO FIRMWARE USES THESE — SD logging was dropped
// once the in-car bus test showed the Hyundai/Kia gateway does not bridge
// broadcast traffic to the OBD port, so there is nothing to log. They are kept
// because this header is the board's pin map, not a list of pins in use:
// deleting them would mean re-deriving the numbers from the vendor example the
// next time anything touches the slot.
#define SD_SCK_PIN       GPIO_NUM_39
#define SD_MOSI_PIN      GPIO_NUM_40
#define SD_MISO_PIN      GPIO_NUM_41
#define SD_CS_PIN        GPIO_NUM_45

#else
#error "No board selected. Build the rejsacan environment."
#endif
