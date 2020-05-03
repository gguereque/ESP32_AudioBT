#include <Arduino.h>
#include <esp_a2dp_api.h>
#include <analogWrite.h>
#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include "esp_log.h"

#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_err.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_core.c"


#include "esp_avrc_api.h"

#define LED 2
int connected;

#define BT_AV_TAG               "BT_AV"
#define BT_RC_CT_TAG            "RCCT"

#define BT_APP_HEART_BEAT_EVT                (0xff00)

/* A2DP global state */
enum {
  APP_AV_STATE_IDLE,
  APP_AV_STATE_DISCOVERING,
  APP_AV_STATE_DISCOVERED,
  APP_AV_STATE_UNCONNECTED,
  APP_AV_STATE_CONNECTING,
  APP_AV_STATE_CONNECTED,
  APP_AV_STATE_DISCONNECTING,
};

/* event for handler "bt_av_hdl_stack_up */
enum {
  BT_APP_EVT_STACK_UP = 0,
};

/* sub states of APP_AV_STATE_CONNECTED */
enum {
    APP_AV_MEDIA_STATE_IDLE,
    APP_AV_MEDIA_STATE_STARTING,
    APP_AV_MEDIA_STATE_STARTED,
    APP_AV_MEDIA_STATE_STOPPING,
};

static int s_a2d_state = APP_AV_STATE_IDLE;
static TimerHandle_t s_tmr;
static int s_media_state = APP_AV_MEDIA_STATE_IDLE;

/* A2DP application state machine handler for each state */
static void bt_app_av_state_unconnected(uint16_t event, void *param);
static void bt_app_av_state_connecting(uint16_t event, void *param);
static void bt_app_av_state_connected(uint16_t event, void *param);
static void bt_app_av_state_disconnecting(uint16_t event, void *param);



#define DAC1 25
#define BUFFER_SIZE 25
int16_t abufPos = 0;
int16_t buf[BUFFER_SIZE];

uint8_t mac[] = {0x50, 0x8C, 0x44, 0x52, 0xA0, 0x18};

static void a2d_app_heart_beat(void *arg);

/// callback function for A2DP source audio data stream
static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len);

// /// A2DP application state machine
// static void bt_app_av_sm_hdlr(uint16_t event, void *param);

// / A2DP application state machine
static void bt_app_av_sm_hdlr(uint16_t event, void *param);

static void bt_app_av_sm_hdlr(uint16_t event, void *param)
{
  ESP_LOGI(BT_AV_TAG, "%s state %d, evt 0x%x", __func__, s_a2d_state, event);
  switch (s_a2d_state) {
    case APP_AV_STATE_DISCOVERING:
    case APP_AV_STATE_DISCOVERED:
    break;
    case APP_AV_STATE_UNCONNECTED:
    //      bt_app_av_state_unconnected(event, param);
    break;
    case APP_AV_STATE_CONNECTING:
    //      bt_app_av_state_connecting(event, param);
    break;
    case APP_AV_STATE_CONNECTED:
      Serial.println("bt_app_av_sm_hdlr, APP_AV_STATE_CONNECTED ");
      // bt_app_av_state_connected(event, param);
      // s_media_state = APP_AV_MEDIA_STATE_IDLE;
      esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
      Serial.println("bt_app_av_sm_hdlr, APP_AV_STATE_CONNECTED ");
      // bt_app_av_media_proc(event, param);
    break;
    case APP_AV_STATE_DISCONNECTING:
    //      bt_app_av_state_disconnecting(event, param);
    break;
    default:
    ESP_LOGE(BT_AV_TAG, "%s invalid state %d", __func__, s_a2d_state);
    break;
  }
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
  // bt_app_work_dispatch(bt_app_av_sm_hdlr, event, param, sizeof(esp_a2d_cb_param_t), NULL);
  Serial.println("In a2d_cb callback! with event: "+event);
  esp_a2d_cb_param_t *a2d = NULL;
   switch (event) {
   case ESP_A2D_CONNECTION_STATE_EVT: {
       a2d = (esp_a2d_cb_param_t *)(param);

       if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
           ESP_LOGI(BT_AV_TAG, "a2dp connected");
           Serial.println("Connected");
           s_a2d_state =  APP_AV_STATE_CONNECTED;
           s_media_state = APP_AV_MEDIA_STATE_IDLE;
       }
       // else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
       //      Serial.println("Disconnected");
       //
       // }
       break;
   }
   default:
       ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
       break;
   }
}

static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len)
{
  Serial.println("estamos dentro");
  if (len < 0 || data == NULL) {

    Serial.println("En return 0, Len= "+len);
    return 0;
  }
Serial.println("estamos primera prueba");
  // generate random sequence
  //int val = rand() % (1 << 16);
  int val = rand();
  Serial.println("estamos en segunda prueba ");
  Serial.println(val);
  for (int i = 0; i < (len >> 1); i++) {
    Serial.println("estamos en tercera prueba ");
  //   Serial.print("Data["+i+"]: ");
  //   Serial.println(data[i]);
  //   Serial.print("Data["+i+"<< 1]antes: ");
  //   Serial.println(data[i << 1]);
  //   Serial.print("Data[()"+i+"<< 1) + 1]antes: ");
  //   Serial.println(data[(i << 1) + 1]);
    data[(i << 1)] = val & 0xff;
  //   Serial.print("Data["+i+"<< 1]despues: ");
  //   Serial.println(data[i << 1]);
    data[(i << 1) + 1] = (val >> 8) & 0xff;
  //   Serial.print("Data[()"+i+"<< 1) + 1]despues: ");
  //   Serial.println(data[(i << 1) + 1]);

  }

  return len;
}

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
  ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
  switch (event) {
    case BT_APP_EVT_STACK_UP: {
      /* set up device name */
      const char *dev_name = "SM-TF";
      esp_bt_dev_set_device_name(dev_name);

      /* initialize A2DP source */
      if (esp_a2d_register_callback(&bt_app_a2d_cb) == ESP_OK){
        Serial.println("esp_a2d registered succesfully");
      }
      else{
        Serial.println("Failed to enable esp_a2d callback");
      }

      if(esp_a2d_source_register_data_callback(bt_app_a2d_data_cb) == ESP_OK){
        Serial.println("Source data callback enabled succesfully!");
      }
      uint8_t vale[] = {12,13,255,146,125,13,46,246,255,255,255,13,48,95,78,255};
      int32_t length = 16;
      uint8_t * sieselvalor = (uint8_t*)vale;
      bt_app_a2d_data_cb(sieselvalor,length);

      if (esp_a2d_source_init() != ESP_OK) {
        Serial.println("Failed to enable A2DP");
      } else {
        Serial.println("A2DP Enable correctly");
      }

      // /* start device discovery */
      // ESP_LOGI(BT_AV_TAG, "Starting device discovery...");
      // s_a2d_state = APP_AV_STATE_DISCOVERING;

      if (esp_a2d_source_connect(mac) != ESP_OK) {
        Serial.println("Failed to conect to SM-TF");
      } else {
        Serial.println((String)"Connection to: "+mac[0]+":"+mac[1]+":"+mac[2]+":"+mac[3]+":"+mac[4]+":"+mac[5]+", named "+dev_name+" correctly");
      }

      s_a2d_state = APP_AV_STATE_CONNECTED;

      bt_app_av_sm_hdlr(NULL,NULL);

      // /* create and start heart beat timer */
      // do {
      //   int tmr_id = 0;
      //   s_tmr = xTimerCreate("connTmr", (10000 / portTICK_RATE_MS),
      //   pdTRUE, (void *)tmr_id, a2d_app_heart_beat);
      //   xTimerStart(s_tmr, portMAX_DELAY);
      // } while (0);
      break;
    }
    default:
    ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
    break;
  }
}



// bool initBluetooth(const char *devicename){
bool initBluetooth() {
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  } else {
    Serial.println("Controller initialize correctly");
  }

  // Initialize NVS.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  } else {
    Serial.println("Bluedroid initialize succesfully");
  }

  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  } else {
    Serial.println("Bluedroid Enable succesfully");
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  Serial.println("0");

  // bool connected;
  if (!initBluetooth()) {
    Serial.println("Bluetooth init failed");
  } else {
    Serial.println("Bluetooth Init correctly");
  }
  Serial.println("1");
  bt_av_hdl_stack_evt(BT_APP_EVT_STACK_UP, NULL);


}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t x = 0;
  while ( x < BUFFER_SIZE) {
    unsigned long timeBegin = micros();
    //adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
    //abufPos =  adc1_get_raw(ADC1_CHANNEL_0);
    abufPos = analogRead(36);
    buf[x] = abufPos;
    unsigned long timeEnd = micros() - timeBegin;
    x++;
  }

  for (int i = 0; i < BUFFER_SIZE; i++) {
    buf[i] = buf[i] / 16;
    dacWrite(DAC1, buf[i]);
  }
  // Serial.println("El valor de buf = ");
  // Serial.println(abufPos);
}
