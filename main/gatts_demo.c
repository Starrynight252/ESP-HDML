/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
 *
 * This demo showcases BLE GATT server. It can send adv data, be connected by client.
 * Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
 * Client demo will enable gatt_server's notify after connection. The two devices will then exchange
 * data.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#include "driver/gpio.h"

// hdml 信号通道切换 高为HDMI-A 低为HDMI-B
#define Eandover_GIO 8
// 检查按键是否按下按下为低电平
#define Switch_GIO 9

// 保存当前信号通道状态
bool Eandover_state = true;
// 保存当前按键状态
bool Switch_state = true;

#define GATTS_TAG "GATTS_DEMO"

/// Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A 0x00FF
#define GATTS_CHAR_UUID_TEST_A 0xFF01
#define GATTS_DESCR_UUID_TEST_A 0x3333
#define GATTS_NUM_HANDLE_TEST_A 4

static char test_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "HDML-ESP";

#define TEST_MANUFACTURER_DATA_LEN 17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
    {
        .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
        .attr_len = sizeof(char1_str),
        .attr_value = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

// 定义一个标志来跟踪BLE连接状态
static bool is_ble_connected = false;

// #CONFIG_EXAMPLE_SET_RAW_ADV_DATAservice_uuid  eab57c2d-5f27-4a2f-89d4-97576f492e1f
static uint8_t adv_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0x1f, 0x2e, 0x49, 0x6f, 0x57, 0x97, 0xd4, 0x89, 0x2f, 0x4a, 0x27, 0x5f, 0x2d, 0x7c, 0xb5, 0xea};

// 将UUID数组转换为esp_bt_uuid_t类型 660536ae-cdf3-49c2-9254-8c3fecad547d
static esp_bt_uuid_t service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = {0x7d, 0x54, 0xad, 0xec, 0x3f, 0x8c, 0x54, 0x92, 0xc2, 0x49, 0xf3, 0xcd, 0xae, 0x36, 0x05, 0x66}};

// The length of adv data must be less than 31 bytes
// static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
// adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} preparedata;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                 param->update_conn_params.status,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                 param->pkt_data_length_cmpl.status,
                 param->pkt_data_length_cmpl.params.rx_len,
                 param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp)
    {
        if (param->write.is_prep)
        {
            if (param->write.offset > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_OFFSET;
            }
            else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
            {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL)
                {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp)
            {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK)
                {
                    ESP_LOGE(GATTS_TAG, "Send response error\n");
                }
                free(gatt_rsp);
            }
            else
            {
                ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK)
            {
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;
        }
        else
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
    {
        ESP_LOGI(GATTS_TAG, "Execute write, value len %d", prepare_write_env->prepare_len);
        ESP_LOG_BUFFER_HEX(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TAG, "Prepare write cancel");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    // 记录事件的日志
    esp_err_t add_char_ret;

    // 判断事件类型
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
        // 设置服务为主服务
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;

        /// 设置服务实例ID
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;

        // // 设置服务UUID的长度为16位
        // gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;

        // // 设置服务UUID
        // gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        // 设置服务句柄为创建服务事件返回的句柄
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;

        // 设置特征UUID的长度为128位
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_128;

        // 将UUID数组复制到特征UUID中
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid128, adv_service_uuid128, sizeof(adv_service_uuid128));

        // 启动服务，使其开始运行
        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);

        // 设置特征属性为可读、可写和可通知
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

        // 添加特征到服务中，并设置特征的权限和初始值
        add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                              a_property,
                                              &gatts_demo_char1_val, NULL);

        // 设置设备名称
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(test_device_name);
        // 检查设置设备名称是否成功
        if (set_dev_name_ret)
        {
            // 如果失败，记录错误日志
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }

        // 配置广播数据
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            // 如果配置失败，记录错误日志
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        // 设置广播配置完成标志
        adv_config_done |= adv_config_flag;

        // 配置扫描响应数据
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            // 如果配置失败，记录错误日志
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        // 设置扫描响应配置完成标志
        adv_config_done |= scan_rsp_config_flag;

        // 创建GATT服务
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT:
    {
        // 记录特征读取事件的日志
        ESP_LOGI(GATTS_TAG, "Characteristic read, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);

        // 初始化响应结构体
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

        // 设置响应的属性值
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;

        // 发送响应
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT:
    {

        // 记录特征写入事件的日志
        ESP_LOGI(GATTS_TAG, "Characteristic write, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);

        // 表示当前的写操作是否是准备写入操作。
        if (!param->write.is_prep)
        {
            // 记录写入的值的长度和内容
            ESP_LOGI(GATTS_TAG, "value len %d, value ", param->write.len);
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);

            // 释放之前的内存
            if (preparedata.prepare_buf != NULL)
            {
                free(preparedata.prepare_buf);
                preparedata.prepare_buf = NULL;
            }

            // 避免无效申请内存
            if (param->write.len > 0)
            {
                // 申请内存
                preparedata.prepare_buf = (uint8_t *)malloc(param->write.len * sizeof(uint8_t));
                for (uint16_t i = 0; i < param->write.len; i++)
                {
                    preparedata.prepare_buf[i] = param->write.value[i];
                }
            }
            preparedata.prepare_len = param->write.len;

            // 检查写入的句柄是否为描述符句柄，并且写入的长度是否为2
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
            {
                // 将写入的值转换为描述符值
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];

                if (descr_value == 0x0001)
                {
                    // 如果描述符值为0x0001，启用通知
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                    {
                        ESP_LOGI(GATTS_TAG, "Notification enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        // 通知数据的大小需要小于MTU大小
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(notify_data), notify_data, false);
                    }
                }
                else if (descr_value == 0x0002)
                {
                    // 如果描述符值为0x0002，启用指示
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                    {
                        ESP_LOGI(GATTS_TAG, "Indication enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        // 指示数据的大小需要小于MTU大小
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000)
                {
                    // 如果描述符值为0x0000，禁用通知和指示
                    ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
                }
                else
                {
                    // 如果描述符值未知，记录错误日志
                    ESP_LOGE(GATTS_TAG, "Unknown descriptor value");
                    ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
                }
            }
        }

        // 处理准备写入事件
        example_write_event_env(gatts_if, &a_prepare_write_env, param);

        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        // 记录执行写入事件的日志
        ESP_LOGI(GATTS_TAG, "Execute write");

        // 发送响应
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);

        // 处理执行写入事件
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;

    case ESP_GATTS_MTU_EVT:
        // 记录MTU交换事件的日志
        ESP_LOGI(GATTS_TAG, "MTU exchange, MTU %d", param->mtu.mtu);
        break;

    case ESP_GATTS_UNREG_EVT:
        break;

    case ESP_GATTS_CREATE_EVT:
        // 记录服务创建事件的日志
        ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);

        // 设置服务句柄
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;

        // 设置特征UUID的长度为16位
        // gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;

        // 设置特征UUID
        // gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        // 设置特征UUID的长度为128位
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_128;
        // 将UUID数组复制到特征UUID中
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid128, adv_service_uuid128, sizeof(adv_service_uuid128));

        // 启动服务
        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);

        // 设置特征属性为可读、可写和可通知
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

        // 添加特征到服务中，并设置特征的权限和初始值
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret)
        {
            // 如果添加特征失败，记录错误日志
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
        }
        break;

    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
    {
        uint16_t length = 0;
        const uint8_t *prf_char;

        // 记录特征添加事件的日志
        ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        // 设置特征句柄
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;

        // 设置描述符UUID的长度为16位
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;

        // 设置描述符UUID
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

        // 获取特征值
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
        if (get_attr_ret == ESP_FAIL)
        {
            // 如果获取特征值失败，记录错误日志
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        // 记录特征值的长度
        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
        for (int i = 0; i < length; i++)
        {
            // 记录特征值
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x", i, prf_char[i]);
        }

        // 添加描述符到特征中，并设置描述符的权限
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret)
        {
            // 如果添加描述符失败，记录错误日志
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        // 设置描述符句柄
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;

        // 记录描述符添加事件的日志
        ESP_LOGI(GATTS_TAG, "Descriptor add, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;

    case ESP_GATTS_DELETE_EVT:
        break;

    case ESP_GATTS_START_EVT:
        // 记录服务启动事件的日志
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;

    case ESP_GATTS_STOP_EVT:
        break;

    case ESP_GATTS_CONNECT_EVT:
    {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms

        // 记录连接事件的日志
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote " ESP_BD_ADDR_STR "",
                 param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));

        // 设置连接ID
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;

        is_ble_connected = true; // 设置连接标志
        // 向对等设备发送更新连接参数请求
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT:
        // 记录断开连接事件的日志
        ESP_LOGI(GATTS_TAG, "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);

        // 重新开始广播
        esp_ble_gap_start_advertising(&adv_params);
        is_ble_connected = false; // 清除连接标志
        break;

    case ESP_GATTS_CONF_EVT:
        // 记录确认接收事件的日志
        ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK)
        {
            // 如果确认接收失败，记录错误日志
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;

    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile_tab[idx].gatts_if)
            {
                if (gl_profile_tab[idx].gatts_cb)
                {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void init_gpio(void)
{
    gpio_config_t eaio_conf;
    // 配置GPIO输出模式
    eaio_conf.intr_type = GPIO_INTR_DISABLE;
    eaio_conf.mode = GPIO_MODE_OUTPUT;
    eaio_conf.pin_bit_mask = (1ULL << Eandover_GIO);
    eaio_conf.pull_down_en = 0;
    eaio_conf.pull_up_en = 0;
    gpio_config(&eaio_conf);

    gpio_config_t switch_io_conf;
    // 配置GPIO输入模式
    switch_io_conf.intr_type = GPIO_INTR_DISABLE;
    switch_io_conf.mode = GPIO_MODE_INPUT;
    switch_io_conf.pin_bit_mask = (1ULL << Switch_GIO);
    switch_io_conf.pull_down_en = 0;
    switch_io_conf.pull_up_en = 0;
    gpio_config(&switch_io_conf);

    // 获取当前开关状态
    Switch_state = gpio_get_level(Switch_GIO);

    // 设置Eandover_GIO为当前开关状态 hdml a
    gpio_set_level(Eandover_GIO, 1);
}

void app_main(void)
{
    // 初始化gio
    init_gpio();

    // 初始化prepare write data
    preparedata.prepare_buf = NULL;
    preparedata.prepare_len = 0;

    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if CONFIG_EXAMPLE_CI_PIPELINE_ID
    memcpy(test_device_name, esp_bluedroid_get_example_name(), ESP_BLE_ADV_NAME_LEN_MAX);
#endif

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    // 获取系统运行时间 -判断按键长时间按下
    uint64_t switch_press_time = 0;
    // 如果按键长时间按下则屏蔽蓝牙对通道配置的控制 - 除非再次长按按键
    bool switch_press_flag = false;

    // 进入主循环
    while (true)
    {

        if (Switch_state && !gpio_get_level(Switch_GIO)) // 检查按键是否在松开在按下
        {
            //超时时间
            uint64_t timeout = 0;
            do
            {
                if(timeout == 0)
                {
                    timeout = esp_timer_get_time();
                }

                // 重复执行的代码
                vTaskDelay(20 / portTICK_PERIOD_MS); // 延迟20ms

                if( (timeout && esp_timer_get_time() - timeout > 500000) ) // 如果按键长时间按下时间大于0.5s
                {
                    timeout = 0; // 重置超时时间
                    break; // 退出循环
                }
            } while (!gpio_get_level(Switch_GIO));// 检查按键是否在松开在按下

            if(timeout) //不是超时退出的
            {
                Eandover_state = !Eandover_state;
                // 设置Eandover_GIO为当前开关状态 hdml a/b
                gpio_set_level(Eandover_GIO, Eandover_state);

                if (switch_press_time) // 如果按键长时间按下时间不为0 则清除
                {
                    switch_press_time = 0;
                }
            }
        }
        else if (!Switch_state && !gpio_get_level(Switch_GIO)) // 长时间保持按下
        {
            if (!switch_press_time) // 如果按键长时间按下时间为0 则获取当前时间
            {
                switch_press_time = esp_timer_get_time(); // 获取系统运行时间
            }
        }

        if (switch_press_time && (esp_timer_get_time() - switch_press_time) > 3000000) // 如果按键长时间按下时间大于3s
        {
            // 重置按键长时间按下时间
            switch_press_time = 0;
            // 设置按键长时间按下/清除标志
            switch_press_flag = !switch_press_flag;
            // 重启设备
            // esp_restart();

            do
            {
                // 重复执行的代码
                vTaskDelay(20 / portTICK_PERIOD_MS); // 延迟20ms

            } while (!gpio_get_level(Switch_GIO));// 检查按键是否在松开在按下
        }

        if (is_ble_connected) // 如果蓝牙连接了
        {

            if (!switch_press_flag) // 检查是否想屏蔽/允许蓝牙对通道配置的控制
            {
                // 记录写入的值的长度和内容
                ESP_LOGI(GATTS_TAG, "min len %d, value ", preparedata.prepare_len);

                if (preparedata.prepare_len > 1 && preparedata.prepare_buf[2] == 0x00) // 检查写入数据是否为 包头0xff  包尾0X00
                {
                    ESP_LOG_BUFFER_HEX(GATTS_TAG, preparedata.prepare_buf, preparedata.prepare_len);
                    if (preparedata.prepare_buf[1] == 0x01) // 检查写入数据是否为 0x01
                    {
                        Eandover_state = 0; // 切换通道
                        // 设置Eandover_GIO为当前开关状态 hdml a/b
                        gpio_set_level(Eandover_GIO, Eandover_state);
                    }
                    else if (preparedata.prepare_buf[1] == 0x02) // 检查写入数据是否为 0x02
                    {
                        Eandover_state = 1; // 切换通道
                        // 设置Eandover_GIO为当前开关状态 hdml a/b
                        gpio_set_level(Eandover_GIO, Eandover_state);
                    }

                    // 清空
                    preparedata.prepare_len = 0;
                    free(preparedata.prepare_buf);
                    preparedata.prepare_buf = NULL;
                }
            }
        }

        // 获取当前开关状态
        Switch_state = gpio_get_level(Switch_GIO);

        // 重复执行的代码
        vTaskDelay(10 / portTICK_PERIOD_MS); // 延迟20ms
    }

    return;
}
