#pragma once

#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"

// HT20 currently fits well below this bound. Never silently truncate a frame.
#define CSI_RAW_MAX_LEN 1024
#define CSI_MOTION_PRIORITY 4
#define CSI_USB_PRIORITY 3
#define CSI_SEND_PRIORITY 5
#define CSI_LED_PRIORITY 2

typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    uint16_t len;
    uint8_t peer_mac[6];
    int8_t rssi;
    uint8_t channel;
    uint8_t first_word_invalid;
    int8_t data[CSI_RAW_MAX_LEN];
} csi_frame_t;

// Call before enabling CSI; creates both queues and the sole USB writer.
void csi_stream_init(const uint8_t local_mac[6]);
// Diagnostic state is shared under a short critical section; only the stream
// task formats/writes it. Stage strings must have static lifetime.
void csi_stream_set_stage(const char *stage, esp_err_t error);
typedef enum {
    CSI_DIAG_TX_OK, CSI_DIAG_TX_FAIL, CSI_DIAG_TX_ERROR,
    CSI_DIAG_RX_PEER, CSI_DIAG_SEEN, CSI_DIAG_FILTERED, CSI_DIAG_COUNT
} csi_diag_event_t;
void csi_stream_count(csi_diag_event_t event);
// Wi-Fi task only: copy the SDK buffer, then enqueue with zero wait.
void csi_stream_publish(const wifi_csi_info_t *info);
bool csi_stream_receive_motion(csi_frame_t *frame, TickType_t timeout);
