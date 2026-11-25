#include "csi_stream.h"

#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "soc/uart_pins.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#if !CONFIG_ESP_CONSOLE_NONE || !CONFIG_ESP_CONSOLE_SECONDARY_NONE
#error "Reserve the CSI transport for its sole writer: disable console output."
#endif
#if CONFIG_FREERTOS_HZ != 1000
#error "CSI scheduling requires a 1 ms RTOS tick."
#endif

#define USB_QUEUE_DEPTH 16
#define MOTION_QUEUE_DEPTH 8
#define HEADER_LEN 48
#define CRC_LEN 4
#define CSI_UART UART_NUM_0
#define CSI_UART_BAUD 921600

// COM devices with VID:PID 303A:1001 are native USB Serial/JTAG, not UART0.
// Set -DCSI_STREAM_USE_USB=0 only for a real CH340/CP210x UART0 bridge.
#ifndef CSI_STREAM_USE_USB
#define CSI_STREAM_USE_USB 1
#endif

#if CSI_STREAM_USE_USB
#define TRANSPORT_NAME "USB_SERIAL_JTAG"
#else
#define TRANSPORT_NAME "UART0"
#endif

static QueueHandle_t s_usb_queue;
static QueueHandle_t s_motion_queue;
static uint8_t s_local_mac[6];
static uint32_t s_sequence; // Only the Wi-Fi callback writes this.
static uint32_t s_usb_queue_drops;
static uint32_t s_motion_queue_drops;
static uint32_t s_oversize_drops;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_events[CSI_DIAG_COUNT];
static const char *s_stage = "stream_ready";
static esp_err_t s_error;

void csi_stream_set_stage(const char *stage, esp_err_t error)
{
    portENTER_CRITICAL(&s_stats_lock);
    // Latch the first failure: another task completing startup must not hide it.
    if (s_error == ESP_OK) {
        s_stage = stage;
        s_error = error;
    }
    portEXIT_CRITICAL(&s_stats_lock);
}

void csi_stream_count(csi_diag_event_t event)
{
    if ((unsigned)event >= CSI_DIAG_COUNT) return;
    portENTER_CRITICAL(&s_stats_lock);
    ++s_events[event];
    portEXIT_CRITICAL(&s_stats_lock);
}

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = v;
    p[1] = v >> 8;
}

static void put_le32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; ++i) p[i] = v >> (8 * i);
}

// CRC-32/ISO-HDLC, same as Python zlib.crc32().
static uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

static bool serial_write_frame(const uint8_t *packet, size_t len)
{
#if CSI_STREAM_USE_USB
    // Whole item enqueue or timeout in IDF 5.5. No host can stall Wi-Fi/motion.
    return usb_serial_jtag_write_bytes(packet, len, pdMS_TO_TICKS(5)) == (int)len;
#else
    size_t offset = 0;
    int64_t deadline = esp_timer_get_time() + 30000;
    while (offset < len && esp_timer_get_time() < deadline) {
        // No TX ring buffer: this API only fills available hardware FIFO space.
        int written = uart_tx_chars(CSI_UART, (const char *)packet + offset, len - offset);
        if (written < 0) return false;
        offset += written;
        if (offset < len) vTaskDelay(1);
    }
    return offset == len;
#endif
}

static bool stream_status(bool text_mode, uint32_t write_drops)
{
    uint32_t events[CSI_DIAG_COUNT], usb_drops, motion_drops, oversize;
    const char *stage;
    esp_err_t error;
    portENTER_CRITICAL(&s_stats_lock);
    memcpy(events, s_events, sizeof(events));
    usb_drops = s_usb_queue_drops + write_drops;
    motion_drops = s_motion_queue_drops;
    oversize = s_oversize_drops;
    stage = s_stage;
    error = s_error;
    portEXIT_CRITICAL(&s_stats_lock);
    char line[640];
    int len = snprintf(line, sizeof(line),
        "\r\n#CSI_STATUS transport=%s mode=%s local=%02x:%02x:%02x:%02x:%02x:%02x "
        "uptime_ms=%llu stage=%.96s error=0x%x tx_ok=%lu tx_fail=%lu tx_error=%lu "
        "rx_peer=%lu csi_seen=%lu csi_filtered=%lu usb_drops=%lu motion_drops=%lu "
        "oversize_drops=%lu commands=TEXT/BINARY/STATUS\r\n",
        TRANSPORT_NAME, text_mode ? "TEXT" : "BINARY",
        s_local_mac[0], s_local_mac[1], s_local_mac[2], s_local_mac[3], s_local_mac[4], s_local_mac[5],
        (unsigned long long)(esp_timer_get_time() / 1000), stage, (unsigned)error,
        (unsigned long)events[CSI_DIAG_TX_OK], (unsigned long)events[CSI_DIAG_TX_FAIL],
        (unsigned long)events[CSI_DIAG_TX_ERROR], (unsigned long)events[CSI_DIAG_RX_PEER],
        (unsigned long)events[CSI_DIAG_SEEN], (unsigned long)events[CSI_DIAG_FILTERED],
        (unsigned long)usb_drops, (unsigned long)motion_drops, (unsigned long)oversize);
    return len > 0 && (size_t)len < sizeof(line) && serial_write_frame((uint8_t *)line, len);
}

static bool read_commands(bool *text_mode)
{
    // Bounded input processing; CR or LF terminates a command. Invalid/overlong
    // lines are ignored until their terminator. There is no allocation here.
    static char command[16];
    static size_t used;
    static bool overflow;
    char input[64];
#if CSI_STREAM_USE_USB
    int count = usb_serial_jtag_read_bytes(input, sizeof(input), 0);
#else
    int count = uart_read_bytes(CSI_UART, input, sizeof(input), 0);
#endif
    bool report = false;
    for (int i = 0; i < count; ++i) {
        char c = input[i];
        if (c == '\r' || c == '\n') {
            command[used] = '\0';
            if (!overflow) {
                if (strcmp(command, "TEXT") == 0) { *text_mode = true; report = true; }
                if (strcmp(command, "BINARY") == 0) { *text_mode = false; report = true; }
                if (strcmp(command, "STATUS") == 0) report = true;
            }
            used = 0;
            overflow = false;
        } else if (!overflow && used < sizeof(command) - 1 && c >= ' ' && c <= '~') {
            command[used++] = c;
        } else {
            overflow = true;
        }
    }
    return report;
}

static bool write_text_frame(const csi_frame_t *frame)
{
    // int8 minimum needs 4 chars plus a delimiter; only this task owns buffer.
    static char line[256 + 5 * CSI_RAW_MAX_LEN];
    int len = snprintf(line, sizeof(line),
        "CSI_DATA,seq=%lu,t_us=%llu,peer=%02x:%02x:%02x:%02x:%02x:%02x,rssi=%d,channel=%u,len=%u,first_word_invalid=%u,data=[",
        (unsigned long)frame->sequence, (unsigned long long)frame->timestamp_us,
        frame->peer_mac[0], frame->peer_mac[1], frame->peer_mac[2],
        frame->peer_mac[3], frame->peer_mac[4], frame->peer_mac[5],
        frame->rssi, frame->channel, frame->len, frame->first_word_invalid);
    if (len < 0 || (size_t)len >= sizeof(line)) return false;
    size_t used = len;
    for (int i = 0; i < frame->len; ++i) {
        len = snprintf(line + used, sizeof(line) - used, "%s%d", i ? "," : "", frame->data[i]);
        if (len < 0 || (size_t)len >= sizeof(line) - used) return false;
        used += len;
    }
    if (sizeof(line) - used < 3) return false;
    memcpy(line + used, "]\r\n", 3);
    return serial_write_frame((uint8_t *)line, used + 3);
}

static void usb_csi_task(void *arg)
{
    csi_frame_t frame;
    uint8_t packet[HEADER_LEN + CSI_RAW_MAX_LEN + CRC_LEN];
    uint32_t write_drops = 0;
    bool text_mode = false;
    int64_t next_status_us = 0;
    while (true) {
        bool report = read_commands(&text_mode);
        int64_t now = esp_timer_get_time();
        if (report || now >= next_status_us) {
            // Complete diagnostic lines are emitted only BETWEEN CSI frames.
            // The existing binary decoder skips these just like ROM boot logs.
            stream_status(text_mode, write_drops);
            next_status_us = now + 1000000;
        }
        if (xQueueReceive(s_usb_queue, &frame, pdMS_TO_TICKS(20)) != pdTRUE) continue;
        if (text_mode) {
            if (!write_text_frame(&frame)) ++write_drops;
            vTaskDelay(1);
            continue;
        }
        memcpy(packet, "CSI1", 4);
        packet[4] = 1;
        packet[5] = frame.first_word_invalid ? 1 : 0;
        put_le16(packet + 6, HEADER_LEN);
        put_le16(packet + 8, frame.len);
        packet[10] = (uint8_t)frame.rssi;
        packet[11] = frame.channel;
        put_le32(packet + 12, frame.sequence);
        put_le32(packet + 16, (uint32_t)frame.timestamp_us);
        put_le32(packet + 20, (uint32_t)(frame.timestamp_us >> 32));
        memcpy(packet + 24, s_local_mac, 6);
        memcpy(packet + 30, frame.peer_mac, 6);
        portENTER_CRITICAL(&s_stats_lock);
        put_le32(packet + 36, s_usb_queue_drops + write_drops);
        put_le32(packet + 40, s_motion_queue_drops);
        put_le32(packet + 44, s_oversize_drops);
        portEXIT_CRITICAL(&s_stats_lock);
        // This byte copy is the only operation on the exported CSI samples.
        memcpy(packet + HEADER_LEN, frame.data, frame.len);
        size_t len = HEADER_LEN + frame.len;
        put_le32(packet + len, crc32(packet, len));
        len += CRC_LEN;
        // Partial/time-limited writes are rejected by the host CRC parser.
        // The UART bridge has no host-delivery acknowledgement or flow control.
        if (!serial_write_frame(packet, len)) ++write_drops;
        // Bound CPU consumption even when an accumulated queue is drained.
        vTaskDelay(1);
    }
}

void csi_stream_init(const uint8_t local_mac[6])
{
    memcpy(s_local_mac, local_mac, 6);
    s_usb_queue = xQueueCreate(USB_QUEUE_DEPTH, sizeof(csi_frame_t));
    s_motion_queue = xQueueCreate(MOTION_QUEUE_DEPTH, sizeof(csi_frame_t));
    ESP_ERROR_CHECK(s_usb_queue && s_motion_queue ? ESP_OK : ESP_ERR_NO_MEM);
#if CSI_STREAM_USE_USB
    usb_serial_jtag_driver_config_t config = {
        // Must hold the largest TEXT item (5376 bytes), too.
        .tx_buffer_size = 8192,
        .rx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
#else
    const uart_config_t config = {
        .baud_rate = CSI_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(CSI_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(CSI_UART, U0TXD_GPIO_NUM, U0RXD_GPIO_NUM,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(CSI_UART, 256, 0, 0, NULL, 0));
#endif
    ESP_ERROR_CHECK(xTaskCreate(usb_csi_task, "usb_csi", 6144, NULL,
                               CSI_USB_PRIORITY, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

void csi_stream_publish(const wifi_csi_info_t *info)
{
    // Static storage avoids using >1 KB of the driver's callback stack.
    // CSI callbacks are serialized by the Wi-Fi task; queues own full copies.
    static csi_frame_t frame;
    if (!info || !info->buf || info->len == 0) return;
    uint32_t sequence = s_sequence++;
    if (info->len > CSI_RAW_MAX_LEN) {
        portENTER_CRITICAL(&s_stats_lock);
        ++s_oversize_drops;
        portEXIT_CRITICAL(&s_stats_lock);
        return;
    }
    frame.sequence = sequence;
    frame.timestamp_us = esp_timer_get_time();
    frame.len = info->len;
    frame.rssi = info->rx_ctrl.rssi;
    frame.channel = info->rx_ctrl.channel;
    frame.first_word_invalid = info->first_word_invalid;
    memcpy(frame.peer_mac, info->mac, 6);
    memcpy(frame.data, info->buf, info->len);
    BaseType_t motion_ok = xQueueSend(s_motion_queue, &frame, 0);
    BaseType_t usb_ok = xQueueSend(s_usb_queue, &frame, 0);
    portENTER_CRITICAL(&s_stats_lock);
    if (motion_ok != pdTRUE) ++s_motion_queue_drops;
    if (usb_ok != pdTRUE) ++s_usb_queue_drops;
    portEXIT_CRITICAL(&s_stats_lock);
}

bool csi_stream_receive_motion(csi_frame_t *frame, TickType_t timeout)
{
    return xQueueReceive(s_motion_queue, frame, timeout) == pdTRUE;
}
