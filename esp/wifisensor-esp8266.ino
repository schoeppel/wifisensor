#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

#define AVR_ADDR 0x50

#ifndef WLAN_SSID
#define WLAN_SSID "<SSID>"
#define WLAN_KEY "<PASSWORD>"
#define WLAN_MQTT "<MQTT_ADDR>"
#endif

_Static_assert(MQTT_MAX_PACKET_SIZE >= 256, "mqtt max packet too small");

struct avr_sensor_data {
  int32_t temperature;
  uint32_t pressure;
  uint32_t humidity;
  uint32_t uptime;
  uint32_t wakeup_count;
  uint16_t battery_voltage;
  uint16_t brightness;
} __attribute__((packed));

struct avr_wifi_data {
  uint8_t valid;
  uint8_t channel;
  uint8_t bssid[6];
} __attribute__((packed));

avr_sensor_data sensors;
avr_wifi_data wifi_data;

WiFiClient espClient;
PubSubClient client(espClient);

void trace(const char* msg) {
  uint32_t ms = ESP.getCycleCount() / 80000;
  Serial.print(ms);
  Serial.print("ms: ");
  Serial.println(msg);
}

void poweroff() {
  uint8_t cmd = 1;
  i2c_write(255, &cmd, 1);
  trace("deep sleep");
  delay(100);
}

void setup() {
  uint32_t cc_start = ESP.getCycleCount();
  
  Serial.begin(230400);
  Serial.println();
  trace("setup");
  
  Wire.begin(5, 4); // SDA, SCL
  Wire.setClock(50000);
 
  i2c_read(sizeof(sensors), &wifi_data, sizeof(wifi_data));
  trace("wifi read");

  avr_wifi_data wifi_data_copy;

  if (wifi_data.valid == 0) {
    Serial.println("WiFi: channel not saved");
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WLAN_SSID, WLAN_KEY);
    wifi_data_copy.valid = 1;
  } else if (wifi_data.valid == 1) {
    Serial.println("WiFi: save channel to FLASH!");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WLAN_SSID, WLAN_KEY, wifi_data.channel, wifi_data.bssid, true);
    wifi_data_copy.valid = 2;
  } else {
    Serial.println("WiFi: nop");
    WiFi.persistent(false);
  }

  trace("wifi started");
  i2c_read(0, &sensors, sizeof(sensors));
  div_t temperature = div(sensors.temperature, 1000);
  div_t pressure = div(sensors.pressure, 1000);
  div_t humidity = div(sensors.humidity, 1000);

  int status;
  int retries = 0;
  do {
    delay(10);
    status = WiFi.status();
  } while(status != WL_CONNECTED && retries++ < 800);

  if (status != WL_CONNECTED) {
    Serial.println("WiFi: not connected, giving up");
    wifi_data.valid = 0;
    i2c_write(sizeof(sensors), &wifi_data, sizeof(wifi_data));
    poweroff();
  }

  trace("wifi connected");

  char json[256];
  snprintf(json, sizeof(json), "{\"temperature\":%i.%03u,\"pressure\":%u.%03u,\"humidity\":%u.%03u,\"voltage\":%u,\"count\":%u,\"uptime\":%u,\"brightness\":%u,\"duration\":%u,\"rssi\":%i,\"channel\":%i}",
    temperature.quot, abs(temperature.rem),
    pressure.quot, pressure.rem,
    humidity.quot, humidity.rem,
    sensors.battery_voltage,
    sensors.wakeup_count,
    sensors.uptime,
    sensors.brightness,
    (ESP.getCycleCount() - cc_start)/ 80000,
    WiFi.RSSI(),
    WiFi.channel()
  );

  trace("json created");
  Serial.println(json);

  
  wifi_data_copy.channel = WiFi.channel();
  memcpy(wifi_data_copy.bssid, WiFi.BSSID(), 6);
  
  if (memcmp(&wifi_data_copy, &wifi_data, sizeof(wifi_data_copy))) {
    i2c_write(sizeof(sensors), &wifi_data_copy, sizeof(wifi_data_copy));
  }
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
  trace("mqtt_start");

  client.setServer(WLAN_MQTT, 1883); 

  uint8_t mac[6];
  WiFi.macAddress(mac);
  char mac_str[16];
  snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  client.connect(mac_str);

  char topic[64];
  snprintf(topic, sizeof(topic), "wifisensor%s/data", mac_str);
  client.publish(topic, json, true);

  espClient.stop();
  poweroff();
}



void loop() {
  delay(1000);
}

void i2c_read(int addr, void* bufv, int len) {
  byte* buf = (byte*)bufv;

  Wire.beginTransmission(AVR_ADDR);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(AVR_ADDR, len);

  byte data;
  while (Wire.available()) {
    *buf = Wire.read();
    buf++;
  }
}

void i2c_write(int addr, void* bufv, int len) {
  byte* buf = (byte*)bufv;

  Wire.beginTransmission(AVR_ADDR);
  Wire.write(addr);

  while (len > 0) {
    Wire.write(*buf);
    buf++;
    len--;
  }
  Wire.endTransmission(true);
}
