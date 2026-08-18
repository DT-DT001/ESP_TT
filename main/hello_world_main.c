/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"

static const char *TAG = "AT_BRIDGE";

// 缓冲区大小定义
#define BUF_SIZE (1024)
#define USB_BUF_SIZE (512)

// UART1配置 - 用于连接外部AT模块
#define UART_NUM UART_NUM_1
#define TXD_PIN (GPIO_NUM_5)  // ESP32S3 UART1 TX
#define RXD_PIN (GPIO_NUM_4)  // ESP32S3 UART1 RX
#define UART_BAUD_RATE (115200)

// USB控制命令：--c <baudrate>
#define COMMAND_BUFFER_SIZE (32)
#define COMMAND_IDLE_TIMEOUT_MS (50)
#define UART_BAUD_RATE_MIN (300)
#define UART_BAUD_RATE_MAX (5000000)
#define UART_TX_DRAIN_TIMEOUT_MS (2000)

// 任务句柄
static TaskHandle_t usb_to_uart_task_handle = NULL;
static TaskHandle_t uart_to_usb_task_handle = NULL;
static SemaphoreHandle_t usb_tx_mutex = NULL;

// 数据缓冲区
static uint8_t usb_data[USB_BUF_SIZE];
static uint8_t uart_data[BUF_SIZE];

// 命令解析状态。USB虚拟串口是字节流，命令可能被拆成多次读取。
static char command_buffer[COMMAND_BUFFER_SIZE];
static size_t command_length = 0;
static bool command_recognized = false;
static bool command_overflow = false;
static bool at_command_boundary = true;
static bool skip_lf_after_command = false;
static uint32_t current_baud_rate = UART_BAUD_RATE;

static bool is_command_space(char ch)
{
    return ch == ' ' || ch == '\t';
}

static bool is_command_terminator(uint8_t ch)
{
    return ch == '\r' || ch == '\n';
}

/**
 * @brief 向USB写数据，防止UART回传和命令响应互相穿插
 */
static int write_usb_bytes(const uint8_t *data, size_t length)
{
    if (length == 0) {
        return 0;
    }

    if (usb_tx_mutex != NULL && xSemaphoreTake(usb_tx_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "USB TX mutex timeout");
        return 0;
    }

    size_t total_written = 0;
    while (total_written < length) {
        int written = usb_serial_jtag_write_bytes(data + total_written,
                                                   length - total_written,
                                                   pdMS_TO_TICKS(100));
        if (written <= 0) {
            break;
        }
        total_written += (size_t)written;
    }

    if (total_written > 0) {
        usb_serial_jtag_ll_txfifo_flush();
    }

    if (usb_tx_mutex != NULL) {
        xSemaphoreGive(usb_tx_mutex);
    }

    return (int)total_written;
}

static void send_usb_text(const char *text)
{
    write_usb_bytes((const uint8_t *)text, strlen(text));
}

static void reset_command_state(void)
{
    command_length = 0;
    command_recognized = false;
    command_overflow = false;
}

/**
 * @brief 执行已经完整接收的波特率命令
 */
static void execute_baud_command(void)
{
    if (command_overflow || command_length >= sizeof(command_buffer)) {
        send_usb_text("\r\nERROR: usage: --c <baudrate>\r\n");
        return;
    }

    command_buffer[command_length] = '\0';

    const char *value = command_buffer + 3; // 跳过"--c"
    if (!is_command_space(*value)) {
        send_usb_text("\r\nERROR: usage: --c <baudrate>\r\n");
        return;
    }

    while (is_command_space(*value)) {
        value++;
    }
    if (*value < '0' || *value > '9') {
        send_usb_text("\r\nERROR: usage: --c <baudrate>\r\n");
        return;
    }

    char *end = NULL;
    unsigned long requested_baud = strtoul(value, &end, 10);
    while (is_command_space(*end)) {
        end++;
    }

    if (*end != '\0') {
        send_usb_text("\r\nERROR: usage: --c <baudrate>\r\n");
        return;
    }

    if (requested_baud < UART_BAUD_RATE_MIN || requested_baud > UART_BAUD_RATE_MAX) {
        char response[96];
        snprintf(response, sizeof(response),
                 "\r\nERROR: baudrate must be %d..%d\r\n",
                 UART_BAUD_RATE_MIN, UART_BAUD_RATE_MAX);
        send_usb_text(response);
        return;
    }

    if ((uint32_t)requested_baud != current_baud_rate) {
        // 先让已经排队的透传数据按旧波特率发送完，避免中途改变造成乱码。
        esp_err_t ret = uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(UART_TX_DRAIN_TIMEOUT_MS));
        if (ret != ESP_OK) {
            send_usb_text("\r\nERROR: UART TX is busy, please retry\r\n");
            return;
        }

        ret = uart_set_baudrate(UART_NUM, (uint32_t)requested_baud);
        if (ret != ESP_OK) {
            char response[96];
            snprintf(response, sizeof(response),
                     "\r\nERROR: failed to set baudrate: %s\r\n",
                     esp_err_to_name(ret));
            send_usb_text(response);
            return;
        }
        current_baud_rate = (uint32_t)requested_baud;
    }

    char response[64];
    snprintf(response, sizeof(response),
             "\r\nOK: UART baudrate=%" PRIu32 "\r\n", current_baud_rate);
    send_usb_text(response);
}

static void write_uart_bytes(const uint8_t *data, size_t length)
{
    if (length == 0) {
        return;
    }

    int written = uart_write_bytes(UART_NUM, data, length);
    if (written < 0 || (size_t)written != length) {
        ESP_LOGW(TAG, "UART TX incomplete: %d/%u", written, (unsigned int)length);
    }
}

static void append_forward_data(uint8_t *forward_data, size_t capacity,
                                size_t *forward_length, const uint8_t *data, size_t length)
{
    if (*forward_length + length > capacity) {
        write_uart_bytes(forward_data, *forward_length);
        *forward_length = 0;
    }

    if (length > capacity) {
        write_uart_bytes(data, length);
        return;
    }

    memcpy(forward_data + *forward_length, data, length);
    *forward_length += length;
}

/**
 * @brief 解析USB输入；控制命令不转发，其他字节保持原样转发
 */
static void process_usb_data(const uint8_t *data, size_t length)
{
    // 未确认的命令前缀最多为"--c"，所以额外预留4字节即可。
    uint8_t forward_data[USB_BUF_SIZE + 4];
    size_t forward_length = 0;

    for (size_t i = 0; i < length; i++) {
        uint8_t ch = data[i];

        if (skip_lf_after_command) {
            skip_lf_after_command = false;
            if (ch == '\n') {
                at_command_boundary = true;
                continue;
            }
        }

        if (command_length > 0) {
            if (!command_recognized) {
                bool prefix_matches = (command_length == 1 && ch == '-') ||
                                      (command_length == 2 && ch == 'c');
                if (prefix_matches) {
                    command_buffer[command_length++] = (char)ch;
                    continue;
                }

                if (command_length == 3 && is_command_space((char)ch)) {
                    command_buffer[command_length++] = (char)ch;
                    command_recognized = true;
                    continue;
                }

                if (command_length == 3 && is_command_terminator(ch)) {
                    // 完整的"--c"也属于控制命令，但缺少参数，会返回用法提示。
                    write_uart_bytes(forward_data, forward_length);
                    forward_length = 0;
                    command_recognized = true;
                    execute_baud_command();
                    reset_command_state();
                    at_command_boundary = true;
                    skip_lf_after_command = (ch == '\r');
                    continue;
                }

                // 不是精确的"--c"命令（例如"--config"），全部按普通数据透传。
                append_forward_data(forward_data, sizeof(forward_data), &forward_length,
                                    (const uint8_t *)command_buffer, command_length);
                append_forward_data(forward_data, sizeof(forward_data), &forward_length, &ch, 1);
                reset_command_state();
                at_command_boundary = is_command_terminator(ch);
                continue;
            }

            if (is_command_terminator(ch)) {
                // 保证命令前面的普通数据先按旧波特率进入UART发送队列。
                write_uart_bytes(forward_data, forward_length);
                forward_length = 0;
                execute_baud_command();
                reset_command_state();
                at_command_boundary = true;
                skip_lf_after_command = (ch == '\r');
                continue;
            }

            if (command_length < sizeof(command_buffer) - 1) {
                command_buffer[command_length++] = (char)ch;
            } else {
                command_overflow = true;
            }
            continue;
        }

        if (at_command_boundary && ch == '-') {
            command_buffer[0] = '-';
            command_length = 1;
            at_command_boundary = false;
            continue;
        }

        append_forward_data(forward_data, sizeof(forward_data), &forward_length, &ch, 1);
        at_command_boundary = is_command_terminator(ch);
    }

    write_uart_bytes(forward_data, forward_length);
}

/**
 * @brief USB输入空闲时结束无换行的命令，或释放未匹配完整的普通数据前缀
 */
static void finish_usb_input_on_idle(void)
{
    if (command_length > 0) {
        if (command_recognized ||
            (command_length == 3 && memcmp(command_buffer, "--c", 3) == 0)) {
            execute_baud_command();
        } else {
            write_uart_bytes((const uint8_t *)command_buffer, command_length);
        }
        reset_command_state();
    }

    // 一段新的USB输入可以从控制命令开始。
    at_command_boundary = true;
    skip_lf_after_command = false;
}

/**
 * @brief 初始化USB Serial JTAG
 */
static void init_usb_serial_jtag(void)
{
    // 配置USB Serial JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = USB_BUF_SIZE,
        .tx_buffer_size = USB_BUF_SIZE,
    };
    
    // 安装USB Serial JTAG驱动
    esp_err_t ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB Serial JTAG init failed: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 初始化UART1
 */
static void init_uart1(void)
{
    // UART配置
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 安装UART驱动
    esp_err_t ret = uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 配置UART参数
    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 设置UART引脚
    ret = uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART pin config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 清空UART缓冲区
    uart_flush(UART_NUM);
}

/**
 * @brief USB到UART转发任务
 * 从USB Serial JTAG读取数据并转发到UART1
 */
static void usb_to_uart_task(void *arg)
{
    while (1) {
        // 从USB Serial JTAG读取数据
        int len = usb_serial_jtag_read_bytes(usb_data, USB_BUF_SIZE - 1,
                                             pdMS_TO_TICKS(COMMAND_IDLE_TIMEOUT_MS));
        
        if (len > 0) {
            process_usb_data(usb_data, (size_t)len);
        } else {
            finish_usb_input_on_idle();
        }
        
        // 短暂延时，避免占用过多CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief UART到USB转发任务
 * 从UART1读取数据并回传到USB Serial JTAG
 */
static void uart_to_usb_task(void *arg)
{
    while (1) {
        // 从UART1读取数据，等待最多100ms
        int len = uart_read_bytes(UART_NUM, uart_data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // 直接转发数据到USB，不添加字符串结束符
            int written = write_usb_bytes(uart_data, (size_t)len);
            if (written != len) {
                ESP_LOGW(TAG, "USB TX incomplete: %d/%d", written, len);
            }
        }
        
        // 短暂延时，避免占用过多CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief 主函数
 */
void app_main(void)
{
    // 初始化USB Serial JTAG
    init_usb_serial_jtag();
    
    // 初始化UART1
    init_uart1();

    usb_tx_mutex = xSemaphoreCreateMutex();
    if (usb_tx_mutex == NULL) {
        ESP_LOGE(TAG, "USB TX mutex creation failed");
        return;
    }
    
    // 创建USB到UART转发任务
    BaseType_t ret = xTaskCreate(usb_to_uart_task, "usb_to_uart", 4096, NULL, 5, &usb_to_uart_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "USB to UART task creation failed");
        return;
    }
    
    // 创建UART到USB转发任务
    ret = xTaskCreate(uart_to_usb_task, "uart_to_usb", 4096, NULL, 5, &uart_to_usb_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "UART to USB task creation failed");
        return;
    }
    
    ESP_LOGI(TAG, "USB-UART Bridge Ready (GPIO%d/GPIO%d, %" PRIu32 " baud)",
             TXD_PIN, RXD_PIN, current_baud_rate);
    ESP_LOGI(TAG, "Change UART baudrate with: --c <baudrate>");
    
    // 主任务保持运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
