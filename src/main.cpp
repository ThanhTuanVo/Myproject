
#include <Arduino.h>
#include <lvgl.h>
#include "ui/ui.h"
#include "lovyanGfxSetup.h"
#include <WiFi.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "DHT.h"


#define EEPROM_SIZE 100

typedef struct sensor {
  int id;
  float temp;
  float hum;
  int readingId;
} sensor;
sensor DHTsensor;

HardwareSerial mySerial(0);
TaskHandle_t WiFiTaskHandle = NULL;

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


void ReceiveSensorDataFromSlave() {
  while (mySerial.available()) { 
      char input[128];  // Buffer lớn đủ để chứa một JSON
      int len = mySerial.readBytesUntil('}', input, sizeof(input) - 2);
      input[len] = '}'; // Đóng JSON
      input[len + 1] = '\0'; // Kết thúc chuỗi JSON

      // Giải mã dữ liệu JSON
      StaticJsonDocument<128> jsonRecvData;
      DeserializationError error = deserializeJson(jsonRecvData, input);

      if (!error) {
          String type = jsonRecvData["type"].as<String>();
          if (type == "sensor") {    
              // Dùng .as<String>() để lấy giá trị dạng String
              DHTsensor.id = jsonRecvData["id"].as<String>().toInt();
              DHTsensor.temp = jsonRecvData["temp"];
              DHTsensor.hum = jsonRecvData["hum"];
              DHTsensor.readingId = jsonRecvData["readingId"].as<String>().toInt();

              // In ra dữ liệu nhận được
              Serial.print("Received sensor data:\n");
              Serial.print("ID: "); Serial.println(DHTsensor.id);
              Serial.print("Temp: "); Serial.println(DHTsensor.temp);
              Serial.print("Hum: "); Serial.println(DHTsensor.hum);
              Serial.print("Reading ID: "); Serial.println(DHTsensor.readingId);

              // Cập nhật giao diện người dùng với dữ liệu mới
              String tempString = String("PV: ") + String(DHTsensor.temp) + "°C";
              String humString = String("PV: ") + String(DHTsensor.hum) + "%";
              lv_label_set_text(ui_pvtemp, tempString.c_str());
              lv_label_set_text(ui_pvhumi, humString.c_str());
          }
      } else {
          // Nếu có lỗi khi giải mã JSON
          Serial.println("Error parsing JSON");
      }
  }
}



void saveWiFiCredentials(const char* ssid, const char* password) {

  char stored_ssid[32];
  EEPROM.readBytes(0, stored_ssid, 32);

  // Kiểm tra xem SSID hoặc mật khẩu có thay đổi không
  if (strcmp(stored_ssid, ssid) != 0 || strcmp(wifi_password, password) != 0) {
      EEPROM.writeBytes(0, ssid, 32);
      EEPROM.writeBytes(32, password, 32);
      EEPROM.commit();
      Serial.println("WiFi credentials saved.");
  } else {
      Serial.println("The SSID and password are the same, no need to write it.");
  }
  
  EEPROM.end();
}

void readWiFiCredentials() {
  Serial.println("Reading WiFi credentials...");

  String savedSSID = EEPROM.readString(0);
  String savedPassword = EEPROM.readString(32);

  // savedSSID.trim();   //Remove spaces or strange characters
  // savedPassword.trim();

  //Copy it into a global variable, ensuring there are no errors
  memset(wifi_ssid, 0, sizeof(wifi_ssid));
  memset(wifi_password, 0, sizeof(wifi_password));

  savedSSID.toCharArray(wifi_ssid, sizeof(wifi_ssid) - 1);
  savedPassword.toCharArray(wifi_password, sizeof(wifi_password) - 1);

  Serial.print("SSID: ");
  Serial.println(wifi_ssid);
  Serial.print("Password: ");
  Serial.println(wifi_password);
}

/** WiFi erase function stored in EEPROM **/
void clearWiFiCredentials() {
  EEPROM.begin(EEPROM_SIZE);  // Khởi tạo EEPROM
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);  // Ghi giá trị 0 vào tất cả các byte trong EEPROM
  }
  EEPROM.commit();  // Đảm bảo ghi các thay đổi vào bộ nhớ
  EEPROM.end();     // Kết thúc làm việc với EEPROM
  Serial.println("WiFi credentials cleared!");
}

/** Wifi config with LVGL */
void wifiTask(void *parameter) {
  const char* ssid = (const char*)parameter;
  const char* password = get_wifi_password();

  // Ngắt kết nối WiFi cũ, thiết lập lại chế độ WiFi Station và bắt đầu kết nối mới
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int timeout = 10; // Thời gian tối đa là 10 giây để kết nối WiFi
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
  WiFiTaskHandle = NULL;  // Đặt lại handle task WiFi
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
  EEPROM.begin(EEPROM_SIZE); 
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

  //clearWiFiCredentials();
  readWiFiCredentials();

  if (strlen(wifi_ssid) > 0 && strlen(wifi_password) > 0) {
    Serial.println("Connecting to WiFi...");
    // Tạo task WiFi, nếu WiFiTaskHandle đã có giá trị thì không tạo lại
    if (WiFiTaskHandle == NULL) {
      xTaskCreate(wifiTask, "WiFiTask", 4096, (void*)wifi_ssid, 1, &WiFiTaskHandle);
    }
  } else {
    Serial.println("No valid WiFi credentials found. Please input new WiFi credentials.");
    // Yêu cầu người dùng nhập thông tin WiFi nếu không có dữ liệu lưu trữ
  }

/** lv timer for run task */
lv_timer_t* WifiTask = lv_timer_create(connectWifi, 5000, NULL);

Serial.println("Setup done");
}

void loop()
{
  lv_task_handler(); /* Let LVGL do its work. */
  delay(5);

  ReceiveSensorDataFromSlave();
}