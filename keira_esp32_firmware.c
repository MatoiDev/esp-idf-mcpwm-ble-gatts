#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_bt.h>

#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gatt_common_api.h>

#include "attributes.h"
#include "motor.h"

#define GATTS_TABLE_TAG                     "KEIRA_GATTS_TABLE"
#define MOTORS_TAG                          "KEIRA_MOTORS"

#define PROFILES_QUANTITY                   1
#define ESP_APP_ID                          0x55
#define PROFILE_APP_IDX                     0

#define DEVICE_NAME                         "Sentinel"
#define SERVICE_IDENTIFIER                  0

#define GATTS_MESSAGE_CHAR_VAL_LEN_MAX      512
#define GATTS_JOYSTICK_CHAR_VAL_LEN_MAX     16
#define PREPARE_BUF_MAX_SIZE                1024
#define CHAR_DECLARATION_SIZE               (sizeof(uint8_t))

#define ADV_CONFIG_FLAG                     (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                (1 << 1)

/* Motors */
bdc_motor_handle_t RIGHT_FRONT_MOTOR = nil;
bdc_motor_handle_t RIGHT_BACK_MOTOR = nil;
bdc_motor_handle_t LEFT_FRONT_MOTOR = nil;
bdc_motor_handle_t LEFT_BACK_MOTOR = nil;

static uint8_t adv_config_done = 0;

uint16_t keira_handle_table[KEIRA_IDX_NB];

static uint8_t service_uuid[16] = {
        0x2D, 0xC1, 0x7D, 0x3D, 0x7A, 0xC3, 0xEE, 0x9F, 0x57, 0x41, 0x3F, 0xE4, 0xEF, 0xA9, 0x8C, 0x9A
};

typedef struct {
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;


/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
        .set_scan_rsp        = false,
        .include_name        = true,
        .include_txpower     = true,
        .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
        .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
        .appearance          = 0x00,
        .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
        .p_manufacturer_data = NULL, //test_manufacturer,
        .service_data_len    = 0,
        .p_service_data      = NULL,
        .service_uuid_len    = sizeof(service_uuid),
        .p_service_uuid      = service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
        .set_scan_rsp        = true,
        .include_name        = true,
        .include_txpower     = true,
        .min_interval        = 0x0006,
        .max_interval        = 0x0010,
        .appearance          = 0x00,
        .manufacturer_len    = 0,
        .p_manufacturer_data = NULL,
        .service_data_len    = 0,
        .p_service_data      = NULL,
        .service_uuid_len    = sizeof(service_uuid),
        .p_service_uuid      = service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
        .adv_int_min         = 0x20,
        .adv_int_max         = 0x40,
        .adv_type            = ADV_TYPE_IND,
        .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
        .channel_map         = ADV_CHNL_ALL,
        .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


struct gatts_profile_inst {
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

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


static struct gatts_profile_inst keira_profile_tab[PROFILES_QUANTITY] = {
        [PROFILE_APP_IDX] = {
                .gatts_cb = gatts_profile_event_handler,
                .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        },
};

/* UUIDs */
static const uint8_t GATTS_MSG_CHARACTERISTIC_UUID[ESP_UUID_LEN_128] = {
        0xA0, 0x19, 0x8F, 0x6B,
        0x24, 0xC4, 0x36, 0xB8,
        0xB6, 0x42, 0x3E, 0x00,
        0x44, 0xB9, 0x46, 0xCC
};

static const uint8_t GATTS_LEFT_JOYSTICK_CHARACTERISTIC_UUID[ESP_UUID_LEN_128] = {
        0x8A, 0x0A, 0x8C, 0x6F,
        0x3B, 0x7A, 0x3C, 0xBB,
        0x71, 0x4E, 0x9E, 0x4F,
        0xE1, 0xC6, 0xB9, 0xF2
};

static const uint8_t GATTS_RIGHT_JOYSTICK_CHARACTERISTIC_UUID[ESP_UUID_LEN_128] = {
        0x2D, 0x0D, 0x9A, 0x8B,
        0x6C, 0x1E, 0x9F, 0x8E,
        0x6E, 0x4B, 0x3B, 0x9E,
        0xC4, 0xF7, 0xD6, 0xA8
};

/* Setup properties constants */
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
__attribute__((__unused__)) static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
__attribute__((__unused__)) static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

/* Cancellables */
static uint8_t msg_char_cancellable[GATTS_MESSAGE_CHAR_VAL_LEN_MAX] = {0x00};
static uint8_t right_joystick_char_cancellable[GATTS_JOYSTICK_CHAR_VAL_LEN_MAX] = {0x00};
static uint8_t left_joystick_char_cancellable[GATTS_JOYSTICK_CHAR_VAL_LEN_MAX] = {0x00};

/* Values */
uint8_t msg_value[GATTS_MESSAGE_CHAR_VAL_LEN_MAX] = {0x00};
uint8_t lj_value[GATTS_JOYSTICK_CHAR_VAL_LEN_MAX] = {0x31, 0x32, 0x37, 0x20, 0x31, 0x32, 0x37};
uint8_t rj_value[GATTS_JOYSTICK_CHAR_VAL_LEN_MAX] = {0x31, 0x32, 0x37, 0x20, 0x31, 0x32, 0x37};

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[KEIRA_IDX_NB] = {
        // Service Declaration
        [KEIRA_IDX_SVC] = {
                .attr_control = {.auto_rsp=ESP_GATT_AUTO_RSP},
                .att_desc=  {
                        .uuid_length = ESP_UUID_LEN_16,
                        .uuid_p = (uint8_t *) &primary_service_uuid,
                        .perm = ESP_GATT_PERM_READ,
                        .max_length = sizeof(uint16_t),
                        .length = sizeof(service_uuid),
                        .value = (uint8_t *) &service_uuid
                }
        },

        /* Characteristic Declaration */
        [KEIRA_IDX_MSG_CHARACTERISTIC] = {
                .attr_control = {.auto_rsp=ESP_GATT_AUTO_RSP},
                .att_desc = {
                        .uuid_length = ESP_UUID_LEN_16,
                        .uuid_p = (uint8_t *) &character_declaration_uuid,
                        .perm = ESP_GATT_PERM_READ,
                        .max_length = CHAR_DECLARATION_SIZE,
                        .length = CHAR_DECLARATION_SIZE,
                        .value = (uint8_t *) &char_prop_write_nr
                }
        },

        /* Characteristic Value for Message*/
        [KEIRA_IDX_MSG_CHARACTERISTIC_VAL] = {
                .attr_control = {.auto_rsp=ESP_GATT_AUTO_RSP},
                .att_desc = {
                        .uuid_length = ESP_UUID_LEN_128,
                        .uuid_p = (uint8_t *) &GATTS_MSG_CHARACTERISTIC_UUID,
                        .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                        .max_length = GATTS_MESSAGE_CHAR_VAL_LEN_MAX,
                        .length = sizeof(msg_char_cancellable),
                        .value = msg_char_cancellable
                }
        },


        /* Characteristic Declaration */
        [KEIRA_IDX_LEFT_JOYSTICK_CHARACTERISTIC] = {
                .attr_control = {.auto_rsp=ESP_GATT_AUTO_RSP},
                .att_desc = {
                        .uuid_length = ESP_UUID_LEN_16,
                        .uuid_p = (uint8_t *) &character_declaration_uuid,
                        .perm = ESP_GATT_PERM_READ,
                        .max_length = CHAR_DECLARATION_SIZE,
                        .length = CHAR_DECLARATION_SIZE,
                        .value = (uint8_t *) &char_prop_write_nr
                }
        },

        /* Characteristic Value for Right Joystick */
        [KEIRA_IDX_RIGHT_JOYSTICK_CHARACTERISTIC_VAL] = {
                .attr_control = {.auto_rsp=ESP_GATT_AUTO_RSP},
                .att_desc = {
                        .uuid_length = ESP_UUID_LEN_128,
                        .uuid_p = (uint8_t *) &GATTS_RIGHT_JOYSTICK_CHARACTERISTIC_UUID,
                        .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                        .max_length = GATTS_MESSAGE_CHAR_VAL_LEN_MAX,
                        .length = sizeof(right_joystick_char_cancellable),
                        .value = right_joystick_char_cancellable
                }
        },

        /* Characteristic Declaration */
        [KEIRA_IDX_RIGHT_JOYSTICK_CHARACTERISTIC] = {
                .attr_control = {.auto_rsp=ESP_GATT_AUTO_RSP},
                .att_desc = {
                        .uuid_length = ESP_UUID_LEN_16,
                        .uuid_p = (uint8_t *) &character_declaration_uuid,
                        .perm = ESP_GATT_PERM_READ,
                        .max_length = CHAR_DECLARATION_SIZE,
                        .length = CHAR_DECLARATION_SIZE,
                        .value = (uint8_t *) &char_prop_write_nr
                }
        },

        /* Characteristic Value for Left Joystick */
        [KEIRA_IDX_LEFT_JOYSTICK_CHARACTERISTIC_VAL] = {
                .attr_control = {.auto_rsp=ESP_GATT_AUTO_RSP},
                .att_desc = {
                        .uuid_length = ESP_UUID_LEN_128,
                        .uuid_p = (uint8_t *) &GATTS_LEFT_JOYSTICK_CHARACTERISTIC_UUID,
                        .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                        .max_length = GATTS_MESSAGE_CHAR_VAL_LEN_MAX,
                        .length = sizeof(left_joystick_char_cancellable),
                        .value = left_joystick_char_cancellable
                }
        },
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            } else {
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            } else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG,
                     "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}


void keira_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf) {
                esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    } else {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

// Обработчик событий
static void
gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            esp_err_t set_dev_name_ret = esp_bt_dev_set_device_name(DEVICE_NAME);
            if (set_dev_name_ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }

            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, KEIRA_IDX_NB,
                                                                      SERVICE_IDENTIFIER);
            if (create_attr_ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
            break;
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %lu, handle %d", param->write.conn_id,
                     param->write.trans_id, param->write.handle);
            if (!param->write.is_prep) {
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
//                ESP_LOGW(GATTS_TABLE_TAG, "\nMSG: %d MSG_VAL: %d\nRJ: %d RJ_VAL: %d\nLJ: %d LJ_VAL: %d",
//                         keira_handle_table[KEIRA_IDX_MSG_CHARACTERISTIC],
//                         keira_handle_table[KEIRA_IDX_MSG_CHARACTERISTIC_VAL],
//                         keira_handle_table[KEIRA_IDX_RIGHT_JOYSTICK_CHARACTERISTIC],
//                         keira_handle_table[KEIRA_IDX_RIGHT_JOYSTICK_CHARACTERISTIC_VAL],
//                         keira_handle_table[KEIRA_IDX_LEFT_JOYSTICK_CHARACTERISTIC],
//                         keira_handle_table[KEIRA_IDX_LEFT_JOYSTICK_CHARACTERISTIC_VAL]);
//                ESP_LOGE(GATTS_TABLE_TAG, "Handle: %d  ", param->write.handle);
                if (keira_handle_table[KEIRA_IDX_MSG_CHARACTERISTIC_VAL] == param->write.handle) {
                    ESP_LOGE(GATTS_TABLE_TAG, "Recieved MESSAGE: %s", param->write.value);
                    memcpy(msg_value, param->write.value, sizeof(msg_value));
                } else if (keira_handle_table[KEIRA_IDX_LEFT_JOYSTICK_CHARACTERISTIC_VAL] == param->write.handle) {
                    ESP_LOGE(GATTS_TABLE_TAG, "Recieved LEFT Joystick position: %s", param->write.value);
                    memcpy(lj_value, param->write.value, sizeof(lj_value));
                } else if (keira_handle_table[KEIRA_IDX_RIGHT_JOYSTICK_CHARACTERISTIC_VAL] == param->write.handle) {
                    ESP_LOGE(GATTS_TABLE_TAG, "Recieved RIGHT Joystick position: %s", param->write.value);
                    memcpy(rj_value, param->write.value, sizeof(rj_value));
                }
            }
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_MESSAGE_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            keira_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status,
                     param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status,
                     param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT: {

            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
                    esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            for (int i = 0; i < 100; i++)
                ESP_LOGE(GATTS_TABLE_TAG, "_________________ ESP has been successfully CONNECTED!!!_________________");
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            device_connected = true;
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params); // To reload service
            device_connected = false;
            break;
        }
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            } else if (param->add_attr_tab.num_handle != KEIRA_IDX_NB) {
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to KEIRA_IDX_NB(%d)", param->add_attr_tab.num_handle, KEIRA_IDX_NB);
            } else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d",
                         param->add_attr_tab.num_handle);
                memcpy(keira_handle_table, param->add_attr_tab.handles, sizeof(keira_handle_table));
                esp_ble_gatts_start_service(keira_handle_table[KEIRA_IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_CREATE_EVT: {
            ESP_LOGI(GATTS_TABLE_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status,
                     param->create.service_handle);
            keira_profile_tab[PROFILE_APP_IDX].char_uuid.len = ESP_UUID_LEN_128;
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

// Тут обработчик регистрации (создания) GATT, выполняется при запуске контроллера
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            keira_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILES_QUANTITY; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == keira_profile_tab[idx].gatts_if) {
                if (keira_profile_tab[idx].gatts_cb) {
                    keira_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void run_motors() {
    while (true) {
        if (device_connected) {

            __INT_FAST32_TYPE__ valLX;
            __INT_FAST32_TYPE__ valLY;
            __INT_FAST32_TYPE__ valRX;
            __INT_FAST32_TYPE__ valRY;

            sscanf((char *) rj_value, "%d %d", &valRX, &valRY);
            sscanf((char *) lj_value, "%d %d", &valLX, &valLY);

            ESP_LOGI(MOTORS_TAG, "LX %d LY %d RX %d RY %d", valLX, valLY, valRX, valRY);

            valLX = map(valLX, 0, 254, -MAX_SPEED, MAX_SPEED);
            valLY = map(valLY, 254, 0, -MAX_SPEED, MAX_SPEED);
            valRX = map(valRX, 0, 254, -MAX_SPEED, MAX_SPEED);
            valRY = map(valRY, 254, 0, -MAX_SPEED, MAX_SPEED);

            ESP_LOGI(MOTORS_TAG, "New Values: LX %d LY %d RX %d RY %d", valLX, valLY, valRX, valRY);

            __INT_FAST32_TYPE__ dutyFR = valLY - valLX;
            __INT_FAST32_TYPE__ dutyFL = valLY + valLX;
            __INT_FAST32_TYPE__ dutyBR = valLY + valLX;
            __INT_FAST32_TYPE__ dutyBL = valLY - valLX;

            dutyFR += valRY - valRX;
            dutyFL += valRY + valRX;
            dutyBR += valRY - valRX;
            dutyBL += valRY + valRX;

            dutyFR = constrain(dutyFR, -100, 100);
            dutyFL = constrain(dutyFL, -100, 100);
            dutyBR = constrain(dutyBR, -100, 100);
            dutyBL = constrain(dutyBL, -100, 100);

            run_motor_smooth1(RIGHT_FRONT_MOTOR, dutyFR);
            run_motor_smooth2(LEFT_FRONT_MOTOR, dutyFL);
            run_motor_smooth3(RIGHT_BACK_MOTOR, dutyBR);
            run_motor_smooth4(LEFT_BACK_MOTOR, dutyBL);

            ESP_LOGI(MOTORS_TAG, "\n\n\nFRONT RIGHT = %d\nFRONT LEFT = %d\nBACK RIGHT = %d\nBACK LEFT = %d\n\n\n",
                     dutyFR, dutyFL, dutyBR, dutyBL);

        } else {
            stop_motor(LEFT_FRONT_MOTOR);
            stop_motor(RIGHT_FRONT_MOTOR);
            stop_motor(LEFT_BACK_MOTOR);
            stop_motor(RIGHT_BACK_MOTOR);
        }

//        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {

    init_motor(&RIGHT_FRONT_MOTOR, GPIO_NUM_19, GPIO_NUM_23, BDC_MCPWM_FREQ_HZ, BDC_MCPWM_TIMER_RESOLUTION_HZ, 0);
    init_motor(&RIGHT_BACK_MOTOR, GPIO_NUM_32, GPIO_NUM_33, BDC_MCPWM_FREQ_HZ, BDC_MCPWM_TIMER_RESOLUTION_HZ, 1);
    init_motor(&LEFT_FRONT_MOTOR, GPIO_NUM_4, GPIO_NUM_18, BDC_MCPWM_FREQ_HZ, BDC_MCPWM_TIMER_RESOLUTION_HZ, 0);
    init_motor(&LEFT_BACK_MOTOR, GPIO_NUM_26, GPIO_NUM_25, BDC_MCPWM_FREQ_HZ, BDC_MCPWM_TIMER_RESOLUTION_HZ, 1);

    bdc_motor_handle_t motors[] = {
            RIGHT_FRONT_MOTOR,
            RIGHT_BACK_MOTOR,
            LEFT_FRONT_MOTOR,
            LEFT_BACK_MOTOR
    };

    enable_all_motors(motors);

    /* BLE */

    esp_err_t ret;

    /* Инициализация библиотеки энергонезависимого хранилища (non-volatile storage, NVS), чтобы иметь возможность сохранять параметры в память flash */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    /* Очистка свободной памяти в контроллере под использование BLE, т.к. мы используем только BLE, а не BLE + BT Classic */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    /*
    Вот, что пишут в официальной доке espressif:
         If the app calls esp_bt_controller_enable(ESP_BT_MODE_BLE)
         to use BLE only then it is safe to call esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT)
         at initialization time to free unused BT Classic memory.
     */

    // Теперь иинициализируем контроллер BT, сначала создавая структуру конфигурации
    // esp_bt_controller_config_t с настройками по умолчанию
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // Включаем контроллер с режимом BLE ONLY
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    // После инициализации контроллера BT выполняется инициализация и разрешение стека Bluedroid,
    // которая включает общие определения и API-функции как для BT Classic, так и для BLE
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    /*
            В этом месте стек Bluetooth запускается, однако функционал приложения пока еще не определен.
        Функционал определяется реакцией на такие события, как установка соединения с клиентом,
        попытки чтения или записи параметров устройством клиента.
        Существуют 2 главных менеджера событий - обработчики GAP и GATT.
        Приложению надо зарегистрировать callback-функцию для каждого обработчика,
        чтобы приложение знало, какие функции предназначены для обработки событий GAP и GATT:

            Функции gatts_event_handler() и gap_event_handler() обрабатывают все события,
        которые проталкиваются в приложение из стека BLE.
     */
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    // Устанавливаем MTU (Memoty Transmision Unit), для Core Bluetooth это 512 байт, поэтому ставит 512
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
    if (local_mtu_ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    xTaskCreatePinnedToCore(&run_motors, "run_motors", 2048, NULL, 5, NULL, 1);
}

