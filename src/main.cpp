
#include <Arduino.h>
#include <lvgl.h>
#include "ui/ui.h"
#include "lovyanGfxSetup.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <SD.h>
#include <PubSubClient.h>


#define SD_CS_PIN 10
#define WIFI_CREDENTIALS_FILE "/wifi_credentials.txt"

typedef struct sensor {
  int id;
  float temp;
  float hum;
  int readingId;
} sensor;
sensor DHTsensor;

HardwareSerial mySerial(0);
TaskHandle_t WiFiTaskHandle = NULL;
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t MQTTTaskHandle = NULL;

const char* mqtt_server = "116.108.82.46";
const char* mqtt_topic = "window";

WiFiClient espClient;
PubSubClient client(espClient);

#define TFT_HOR_RES SCREEN_WIDTH
#define TFT_VER_RES SCREEN_HEIGHT

/* LVGL draws into this buffer, 1/10 screen size usually works well. The size is in bytes. */
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))

uint32_t draw_buf[DRAW_BUF_SIZE / 4];

LGFX tft;

#if LV_USE_LOG != 0
void my_print(lv_log_level_t level, const char *buf)
{
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}
#endif

/* LVGL calls it when a rendered image needs to copied to the display. */
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((lgfx::rgb565_t *)px_map, w * h);
  tft.endWrite();

  /* Call it to tell LVGL you are ready. */
  lv_disp_flush_ready(disp);
}

/* Read the touchpad. */
void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
  uint16_t touchX, touchY;
  bool touched = tft.getTouch(&touchX, &touchY);

  if (!touched)
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
  else
  {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = touchX;
    data->point.y = touchY;
#if 0
    Serial.printf("x: %03d, y: %03d\n", data->point.x, data->point.y);
#endif
  }
}

/** Set tick routine needed for LVGL internal timings **/
static uint32_t my_tick_get_cb (void) { return millis(); }

void ReceiveSensorDataFromSlave(void *parameter) {
  while (true) {
    while (mySerial.available()) { 
        char input[128];
        int len = mySerial.readBytesUntil('}', input, sizeof(input) - 2);
        input[len] = '}'; 
        input[len + 1] = '\0';

        StaticJsonDocument<128> jsonRecvData;
        DeserializationError error = deserializeJson(jsonRecvData, input);

        if (!error) {
            String type = jsonRecvData["type"].as<String>();
            if (type == "sensor") {    
                DHTsensor.id = jsonRecvData["id"].as<String>().toInt();
                DHTsensor.temp = jsonRecvData["temp"];
                DHTsensor.hum = jsonRecvData["hum"];
                DHTsensor.readingId = jsonRecvData["readingId"].as<String>().toInt();

                Serial.print("Received sensor data:\n");
                Serial.print("ID: "); Serial.println(DHTsensor.id);
                Serial.print("Temp: "); Serial.println(DHTsensor.temp);
                Serial.print("Hum: "); Serial.println(DHTsensor.hum);
                Serial.print("Reading ID: "); Serial.println(DHTsensor.readingId);

                String tempString = String("PV: ") + String(DHTsensor.temp) + "°C";
                String humString = String("PV: ") + String(DHTsensor.hum) + "%";
                lv_label_set_text(ui_pvtemp, tempString.c_str());
                lv_label_set_text(ui_pvhumi, humString.c_str());
            }
        } else {
            Serial.println("Error parsing JSON");
        }
    }
    vTaskDelay(100); // Delay to prevent blocking other tasks
  }
}


// Hàm gửi dữ liệu sensor đến MQTT
void sendSensorDataToMQTT() {
  if (client.connected()) {
    String payload = String("{\"temp\":") + String(DHTsensor.temp) + ", \"hum\":" + String(DHTsensor.hum) + "}";
    client.publish(mqtt_topic, payload.c_str());
    Serial.println("Sensor data sent to MQTT broker: " + payload);
  }
}

void mqttConnectTask(void *parameter) {
  // Kết nối với WiFi nếu chưa kết nối
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }
    Serial.println("WiFi connected.");
  }

  // Kết nối với MQTT broker
  client.setServer(mqtt_server, 1883);  // 1883 là port mặc định của MQTT
 

  while (true) {
    if (!client.connected()) {
      Serial.println("Connecting to MQTT...");
      // Tạo ID kết nối MQTT
      String clientId = "ESP32Client-";
      clientId += String(random(0xffff), HEX);

      if (client.connect(clientId.c_str())) {
        Serial.println("MQTT connected");
        client.subscribe(mqtt_topic);  // Đăng ký subscribe cho topic
      } else {
        Serial.print("Failed to connect to MQTT. Retrying...");
        delay(5000);
      }
    }

    // Gửi dữ liệu sensor lên MQTT mỗi 10 giây
    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime > 10000) {
      sendSensorDataToMQTT();
      lastSendTime = millis();
    }

    client.loop();  // Để xử lý các tin nhắn đến từ MQTT
    vTaskDelay(100);  // Delay để tránh task chiếm hết CPU
  }
}

char prev_wifi_ssid[32] = "";
char prev_wifi_password[32] = "";

void saveWiFiCredentials(const char* ssid, const char* password) {
  if (strcmp(wifi_ssid, prev_wifi_ssid) != 0 || strcmp(wifi_password, prev_wifi_password) != 0) {
    Serial.println("New WiFi credentials entered!");
    strncpy(prev_wifi_ssid, wifi_ssid, sizeof(prev_wifi_ssid) - 1);
    strncpy(prev_wifi_password, wifi_password, sizeof(prev_wifi_password) - 1);
  } else {
    // Dữ liệu không thay đổi
    Serial.println("WiFi credentials have not changed.");
  }

  File file = SD.open("/wifi_credentials.txt", FILE_WRITE);

  if (file) {
    file.print(ssid);
    file.print("\n");
    file.print(password);
    file.close();
    Serial.println("WiFi credentials saved to SD card.");
  } else {
    Serial.println("Failed to open file for writing.");
  }
  
}

void readWiFiCredentials() {
  
  File file = SD.open("/wifi_credentials.txt", FILE_READ);

  if (file) {
    String ssid = file.readStringUntil('\n');
    String password = file.readStringUntil('\n');

    ssid.trim(); // Xóa khoảng trắng đầu và cuối
    password.trim(); // Xóa khoảng trắng đầu và cuối

    if (ssid.length() > 0 && password.length() > 0) {
      ssid.toCharArray(wifi_ssid, sizeof(wifi_ssid));
      password.toCharArray(wifi_password, sizeof(wifi_password));

    } else {
      Serial.println("Invalid WiFi credentials in file.");
    }

    file.close();

    } else {
      Serial.println("Failed to open file for reading.");
    }

  Serial.print("SSID: ");
  Serial.println(wifi_ssid);
  Serial.print("Password: ");
  Serial.println(wifi_password);
}

/** WiFi erase function stored in EEPROM **/
void clearWiFiCredentials() {
  // Xóa file chứa thông tin WiFi trên SD card
  if (SD.exists("/wifi_credentials.txt")) {
    SD.remove("/wifi_credentials.txt");
    Serial.println("WiFi credentials cleared!");
  }
}

/** Wifi config with LVGL */
void wifiTask(void *parameter) {
  const char* ssid = wifi_ssid;
  const char* password = wifi_password;

  // Ngắt kết nối WiFi cũ, thiết lập lại chế độ WiFi Station và bắt đầu kết nối mới
  WiFi.disconnect(true);
  vTaskDelay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int timeout = 20; // Thời gian tối đa là 10 giây để kết nối WiFi
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    vTaskDelay(500);  // Chờ 500ms trước khi thử lại
    Serial.print(".__.");
    timeout--;
  }

  if (WiFi.status() == WL_CONNECTED) {
    // Nếu kết nối WiFi thành công
    Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    lv_label_set_text(ui_statuswf, "Connected");
    lv_label_set_text(ui_wifiname, WiFi.SSID().c_str());

    // Lưu SSID và mật khẩu vào EEPROM để tự động kết nối lần sau
    saveWiFiCredentials(ssid, password);
  } else {
    // Nếu kết nối WiFi không thành công
    Serial.println("\nConnect fail! Please check SSID & Password.");
    lv_label_set_text(ui_statuswf, "Wrong WiFi");
    lv_label_set_text(ui_wifiname, "No WiFi");

    // Ngắt kết nối WiFi và tắt WiFi
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  // Sau khi kết nối hoặc thất bại, xóa task WiFi để giải phóng bộ nhớ
  //WiFiTaskHandle = NULL;  // Đặt lại handle task WiFi
  vTaskDelete(NULL);  // Xóa task hiện tại
}


/** Function to input SSID and password from the keyboard */
void connectWifi(lv_timer_t * timer) {

  if (WiFiTaskHandle != NULL) return;
  
  
  const char* ssid = get_wifi_ssid();
  if (*ssid) {
      xTaskCreate(
          wifiTask,      // WiFi Task
          "WiFiTask",    
          4048,          // RAM For task (4KB)
          (void*)ssid,   // Input parameter (SSID)
          1,             // Priority (1 = low)
          &WiFiTaskHandle     
      );
  }
}

void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 44, 43); // RX_PIN, TX_PIN là các chân UART0

  lv_init();

  tft.begin();
  tft.setRotation(0);
  tft.setBrightness(255);

  /* Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb((lv_tick_get_cb_t)millis);

  /* Register print function for debugging. */
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  /* Create a display. */
  lv_display_t *disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

  /* Initialize the (dummy) input device driver. */
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
  lv_indev_set_read_cb(indev, my_touchpad_read);

#if 1
  /* Create a simple label. */
  lv_obj_t *label = lv_label_create( lv_scr_act() );
  lv_label_set_text( label, "Hello Arduino, I'm LVGL!" );
  lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
#endif

lv_tick_set_cb( my_tick_get_cb );

ui_init();

if (!SD.begin(SD_CS_PIN)) {
  Serial.println("Failed to initialize SD card.");
  return;
}
Serial.println("SD card initialized.");

  //clearWiFiCredentials();
  readWiFiCredentials();

  if (strlen(wifi_ssid) > 0 && strlen(wifi_password) > 0) {
    Serial.println("Connecting to WiFi...");
    // Tạo task WiFi, nếu WiFiTaskHandle đã có giá trị thì không tạo lại
    if (WiFiTaskHandle == NULL) {
      xTaskCreate(wifiTask, "WiFiTask", 4096, (void*)wifi_ssid, 0, &WiFiTaskHandle);
    }
  } else {
    Serial.println("No valid WiFi credentials found. Please input new WiFi credentials.");
    // Yêu cầu người dùng nhập thông tin WiFi nếu không có dữ liệu lưu trữ
  }

/** lv timer for run task */
lv_timer_t* WifiTask = lv_timer_create(connectWifi, 5000, NULL);

Serial.println("Setup done");

xTaskCreate(ReceiveSensorDataFromSlave, "SensorDataTask", 4096, NULL, 2, &SensorTaskHandle);
if (MQTTTaskHandle == NULL) {
  xTaskCreate(mqttConnectTask, "MQTTConnectTask", 4096, NULL, 1, &MQTTTaskHandle);
}

Serial.println("Setup done");
}

void loop()
{
  lv_task_handler(); /* Let LVGL do its work. */
  delay(5);

  
}