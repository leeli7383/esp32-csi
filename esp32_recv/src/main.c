/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ESP32-C6 CSI 接收端。
 *
 * 1. 接收端工作流程：固定 MAC 和信道，初始化 ESP-NOW 以产生可观测报文，
 * 2. 开启 Wi-Fi CSI 回调后筛选指定发送端 MAC，将原始 CSI 复制到独立的 USB 和运动检测队列。
 * 3. 固件端使用稳定子载波、基线差、帧间差和短窗方差的 motion score 判定 PASS/CLEAR。
 * 4. recv 端只采集 802.11n HT20 CSI
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motion_core.h"
#include "csi_stream.h"
#include "freertos/semphr.h"

#if !CONFIG_ESP_WIFI_CSI_ENABLED
#error "Enable CONFIG_ESP_WIFI_CSI_ENABLED in sdkconfig or sdkconfig.defaults for CSI receive."
#endif

// 接收端和发送端必须保持同一信道，否则 ESP-NOW 报文和 CSI 都无法稳定收到。
#define CONFIG_LESS_INTERFERENCE_CHANNEL   11

// 只保留指定发送端虚拟 MAC 的 CSI，减少环境中其他 Wi-Fi 包带来的干扰。
#define CSI_FILTER_SEND_MAC                 1

// ESP-IDF 5.4+ 在 ESP32-C6 上使用新的 band/protocol/bandwidth API，旧版本保留兼容路径。
#if CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
#define CONFIG_WIFI_BAND_MODE               WIFI_BAND_MODE_2G_ONLY
#define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW_HT20
#define CONFIG_WIFI_2G_PROTOCOL             WIFI_PROTOCOL_11N
#else
#define CONFIG_WIFI_BANDWIDTH               WIFI_BW_HT20
#endif

#define CONFIG_ESP_NOW_PHYMODE              WIFI_PHY_MODE_HT20
#define CONFIG_ESP_NOW_RATE                 WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_SEND_FREQUENCY               100
#define PASS_LED_HOLD_US                     (1500LL * 1000LL)

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

// ---------------- LED 配置 ----------------
#define RUN_LED_GPIO                        GPIO_NUM_10
#define PASS_LED_GPIO                       GPIO_NUM_11
// -----------------------------------------

// 使用本地管理 MAC，便于发送端和接收端在实验中固定配对。
static const uint8_t CONFIG_RECV_STA_MAC[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
static const char *TAG = "csi_recv";

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);
static const uint8_t *s_peer_mac = CONFIG_CSI_SEND_MAC;
static SemaphoreHandle_t s_send_ready;

// Keep the independent stream task alive so startup errors remain observable.
static void app_check(esp_err_t error, const char *expression)
{
    if (error == ESP_OK) return;
    csi_stream_set_stage(expression, error);
    vTaskDelete(NULL);
}
#define APP_CHECK(call) app_check((call), #call)


// GPIO11 is active high: PASS turns the LED on, CLEAR turns it off.
static void pass_led_set(bool pass)
{
    esp_err_t ret = gpio_set_level(PASS_LED_GPIO, pass ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PASS LED: %s", esp_err_to_name(ret));
    }
}

// GPIO11 由 motion_task 独占；GPIO10 由 breathing_led_task 独占。
static void led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PASS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    APP_CHECK(gpio_config(&io_conf));

    // 上电后熄灭检测灯。
    pass_led_set(false);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t channel = {
        .gpio_num = RUN_LED_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    APP_CHECK(ledc_timer_config(&timer));
    APP_CHECK(ledc_channel_config(&channel));

    ESP_LOGI(TAG, "breathing LED GPIO%d and detection LED GPIO%d initialized (active high)",
             RUN_LED_GPIO, PASS_LED_GPIO);
}

static void breathing_led_task(void *arg)
{
    int duty = 0;
    int step = 16;
    while (true) {
        APP_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
        APP_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        duty += step;
        if (duty >= 1023) { duty = 1023; step = -16; }
        if (duty <= 0) { duty = 0; step = 16; }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// 初始化 Wi-Fi：固定 MAC、固定 2.4GHz HT20 信道，并关闭省电保证 CSI 连续性。
static void wifi_init(void)
{
    APP_CHECK(esp_event_loop_create_default());
    APP_CHECK(esp_netif_init());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    APP_CHECK(esp_wifi_init(&cfg));
    APP_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    APP_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    APP_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_RECV_STA_MAC));

#if (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0))
    APP_CHECK(esp_wifi_start());
    APP_CHECK(esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE));

    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
    };
    APP_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));

    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
    };
    APP_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));
#else
    APP_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, CONFIG_WIFI_BANDWIDTH));
    APP_CHECK(esp_wifi_start());
#endif

    // 关闭 Wi-Fi 省电，避免接收间歇影响 CSI 时间序列。
    APP_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    APP_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // 接收端不要设置成发送端 MAC
    uint8_t primary_channel = 0;
    wifi_second_chan_t second_channel = WIFI_SECOND_CHAN_NONE;
    APP_CHECK(esp_wifi_get_channel(&primary_channel, &second_channel));
    ESP_LOGI(TAG, "Current WiFi channel: primary=%u second=%d", primary_channel, second_channel);

    // APP_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
}

// 初始化 ESP-NOW 接收路径；广播 peer 用于接收发送端周期报文并触发 CSI 采集。
static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
    APP_CHECK(esp_now_init());
    // Unencrypted experiment peer: no application PMK is configured.
    APP_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    APP_CHECK(esp_now_register_send_cb(espnow_send_cb));

    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate = CONFIG_ESP_NOW_RATE,
        .ersu = false,
        .dcm = false
    };

    APP_CHECK(esp_now_add_peer(&peer));
    APP_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));
}

// ESP-NOW 回调运行在 Wi-Fi 任务中，只释放一次发送许可。
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    csi_stream_count(status == ESP_NOW_SEND_SUCCESS ? CSI_DIAG_TX_OK : CSI_DIAG_TX_FAIL);
    xSemaphoreGive(s_send_ready);
}

static void espnow_send_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5));
    TickType_t last_wake = xTaskGetTickCount();
    for (uint32_t seq = 0; ; ++seq) {
        // 上一帧尚未完成则跳过本周期，禁止重叠提交和积压发送。
        if (xSemaphoreTake(s_send_ready, 0) == pdTRUE) {
            if (esp_now_send(s_peer_mac, (const uint8_t *)&seq, sizeof(seq)) != ESP_OK) {
                csi_stream_count(CSI_DIAG_TX_ERROR);
                xSemaphoreGive(s_send_ready);
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / CONFIG_SEND_FREQUENCY));
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info && memcmp(recv_info->src_addr, s_peer_mac, 6) == 0) {
        csi_stream_count(CSI_DIAG_RX_PEER);
    }
    (void)data;
    (void)len;
}

// 仅筛选对端并复制原始字节；禁止在 Wi-Fi 回调里计算或等待 USB。
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (!info || !info->buf) return;
    csi_stream_count(CSI_DIAG_SEEN);
#if CSI_FILTER_SEND_MAC
    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6) != 0) {
        csi_stream_count(CSI_DIAG_FILTERED);
        return;
    }
#endif
    csi_stream_publish(info);
}

static void motion_task(void *arg)
{
    csi_frame_t frame;
    int64_t hold_until_us = 0;
    motion_core_init();
    while (true) {
        bool received = csi_stream_receive_motion(&frame, pdMS_TO_TICKS(20));
        int64_t now = esp_timer_get_time();
        // 即使对端断电、没有 CSI，也会到期熄灯并清除算法 PASS。
        if (hold_until_us && now >= hold_until_us) {
            hold_until_us = 0;
            pass_led_set(false);
            motion_core_force_clear();
        }
        if (!received) continue;
        motion_result_t motion;
        if (motion_core_update(frame.data, frame.len, &motion)) {
            bool pass = !motion.in_warmup && motion.state == MOTION_STATE_PASS &&
                        motion.motion_score >= motion.trigger_threshold;
            int64_t deadline = (int64_t)frame.timestamp_us + PASS_LED_HOLD_US;
            if (pass && deadline > esp_timer_get_time()) {
                hold_until_us = deadline;
                pass_led_set(true);
            }
        }
        // 高负载时也为 USB、呼吸灯和 idle 留出调度机会。
        vTaskDelay(1);
    }
}

// 开启 promiscuous + CSI，仅采集 HT20 CSI，匹配当前 ESP32-C6 实验配置。
static void wifi_csi_init(void)
{
    APP_CHECK(esp_wifi_set_promiscuous(true));

    // 只打开实际需要的 HT20 CSI，其他制式保持关闭以减少兼容性问题。
    wifi_csi_config_t csi_config = {
        .enable                 = true,
        .acquire_csi_legacy     = false,
        .acquire_csi_ht20       = true,
        .acquire_csi_ht40       = false,
        .acquire_csi_su         = false,
        .acquire_csi_mu         = false,
        .acquire_csi_dcm        = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc    = 0,
        .val_scale_cfg          = false,
        .dump_ack_en            = false,
        .reserved               = false
    };

    APP_CHECK(esp_wifi_set_csi_config(&csi_config));
    APP_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    APP_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "CSI enabled: HT20 only, ESP32-C6 compatible config");
}

// 应用入口：NVS -> LED -> Wi-Fi -> ESP-NOW -> CSI，顺序不能随意调整。
void app_main(void)
{
    // 暂停使用串口日志，避免 CSI 高频处理受到输出阻塞。
    esp_log_level_set("*", ESP_LOG_NONE);
    csi_stream_init(CONFIG_RECV_STA_MAC);
    csi_stream_set_stage("nvs", ESP_OK);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        APP_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    APP_CHECK(ret);

    // 先初始化运行指示灯，表示程序已经启动。
    csi_stream_set_stage("led_tasks", ESP_OK);
    led_init();
    APP_CHECK(xTaskCreate(motion_task, "motion", 4096, NULL,
                               CSI_MOTION_PRIORITY, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    APP_CHECK(xTaskCreate(breathing_led_task, "breathing_led", 2048, NULL,
                               CSI_LED_PRIORITY, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    s_send_ready = xSemaphoreCreateBinary();
    APP_CHECK(s_send_ready ? ESP_OK : ESP_ERR_NO_MEM);
    xSemaphoreGive(s_send_ready);

    csi_stream_set_stage("wifi", ESP_OK);
    wifi_init();
    ESP_LOGI(TAG, "expect sender virtual MAC: " MACSTR, MAC2STR(CONFIG_CSI_SEND_MAC));

    // 固定对端同时用于发送和接收；信道必须一致。
    esp_now_peer_info_t peer = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
        .peer_addr = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01},
    };

    csi_stream_set_stage("espnow", ESP_OK);
    wifi_esp_now_init(peer);
    csi_stream_set_stage("csi", ESP_OK);
    wifi_csi_init();
    APP_CHECK(xTaskCreate(espnow_send_task, "espnow_send", 3072, NULL,
                               CSI_SEND_PRIORITY, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    csi_stream_set_stage("running", ESP_OK);
    ESP_LOGI(TAG, "bidirectional ESP-NOW + CSI enabled");

    ESP_LOGI(TAG, "csi_recv app started");
}
