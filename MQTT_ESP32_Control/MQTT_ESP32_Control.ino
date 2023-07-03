#include <Wire.h>
#include <ClosedCube_HDC1080.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "your_SSID";                  // Wi-Fi 네트워크 이름
const char* password = "your_PASSWORD";        // Wi-Fi 암호
const char* mqttServer = "your_MQTT_broker_address"; // MQTT 브로커 주소
const int mqttPort = 1883;                    // MQTT 브로커 포트
const char* mqttUser = "your_mqttUser";        // MQTT 사용자 이름 (선택 사항)
const char* mqttPassword = "your_mqttPassword";       // MQTT 비밀번호 (선택 사항)

WiFiClient espClient;
PubSubClient client(espClient);

ClosedCube_HDC1080 hdc1080;

const char* temperatureTopic = "car/temperature"; // 온도 토픽
const char* humidityTopic = "car/humidity";       // 습도 토픽
const char* voltageTopic = "car/voltage";         // 전압 토픽
const char* mqtt_topic_lock = "car/lock";         // 릴레이(잠금)토픽
const char* mqtt_topic_unlock = "car/unlock";     // 릴레이(열림)토픽
const char* mqtt_topic_trunk = "car/trunk";       // 릴레이(트렁크)토픽
const char* mqtt_topic_start = "car/start";       // 릴레이(시동)토픽
const char* mqtt_topic_stop = "car/stop";         // 릴레이(시동정지)토픽

const uint8_t analogInputPin = 34;
const uint32_t maxADCValue = 4095;
const float adcCorrectionFactor = 5.5;
const int powerPin = 13;
const int lockPin = 12;
const int unlockPin = 14;
const int trunkPin = 27;

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Wi-Fi에 연결 중...");
  }
  Serial.println("Wi-Fi에 연결되었습니다!");
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("MQTT에 연결 중...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("MQTT에 연결되었습니다!");
      client.subscribe(mqtt_topic_lock);
      client.subscribe(mqtt_topic_unlock);
      client.subscribe(mqtt_topic_trunk);
      client.subscribe(mqtt_topic_start);
      client.subscribe(mqtt_topic_stop);
    } else {
      Serial.print("연결 실패, rc=");
      Serial.print(client.state());
      Serial.println(" 5초 후 재시도...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  Wire.begin(26, 25); // SDA: D26, SCL: D25
  hdc1080.begin(0x40);
  pinMode(powerPin, OUTPUT);
  pinMode(lockPin, OUTPUT);
  pinMode(unlockPin, OUTPUT);
  pinMode(trunkPin, OUTPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  float temperature = hdc1080.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("C");
  sendTemperature(temperature);

  float humidity = hdc1080.readHumidity();
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  sendHumidity(humidity);

  uint16_t rawValue = analogRead(analogInputPin);
  float voltage = (rawValue * 3.3 * adcCorrectionFactor) / maxADCValue;
  Serial.println(voltage);
  sendVoltage(voltage);

  delay(2000);
}

void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, temperatureTopic) == 0) {
  } else if (strcmp(topic, humidityTopic) == 0) {
  } else if (strcmp(topic, mqtt_topic_lock) == 0) {
    onLockChange();
  } else if (strcmp(topic, mqtt_topic_unlock) == 0) {
    onUnlockChange();
  } else if (strcmp(topic, mqtt_topic_trunk) == 0) {
    onTrunkChange();
  } else if (strcmp(topic, mqtt_topic_start) == 0) {
    onStartChange();
  } else if (strcmp(topic, mqtt_topic_stop) == 0) {
    onStopChange();
  }
}

void sendTemperature(float temperature) {
  char temperatureStr[10];
  dtostrf(temperature, 4, 2, temperatureStr);
  client.publish(temperatureTopic, temperatureStr);
}

void sendHumidity(float humidity) {
  char humidityStr[10];
  dtostrf(humidity, 4, 2, humidityStr);
  client.publish(humidityTopic, humidityStr);
}

void sendVoltage(float voltage) {
  char voltageStr[10];
  dtostrf(voltage, 4, 2, voltageStr);
  client.publish(voltageTopic, voltageStr);
}

void onLockChange() {
  digitalWrite(powerPin, HIGH);
  delay(300);
  digitalWrite(lockPin, HIGH);
  delay(200);
  digitalWrite(lockPin, LOW);
  delay(300);
  digitalWrite(powerPin, LOW);
}

void onUnlockChange() {
  digitalWrite(powerPin, HIGH);
  delay(300);
  digitalWrite(unlockPin, HIGH);
  delay(200);
  digitalWrite(unlockPin, LOW);
  delay(300);
  digitalWrite(powerPin, LOW);
}

void onTrunkChange() {
  digitalWrite(powerPin, HIGH);
  delay(300);
  digitalWrite(trunkPin, HIGH);
  delay(2100);
  digitalWrite(trunkPin, LOW);
  delay(300);
  digitalWrite(powerPin, LOW);
}

void onStartChange() {
  digitalWrite(powerPin, HIGH);
  delay(300);
  for (int i = 0; i < 3; i++) {
    digitalWrite(trunkPin, HIGH);
    delay(200);
    digitalWrite(trunkPin, LOW);
    delay(200);
  }
  delay(300);
  digitalWrite(powerPin, LOW);
}

void onStopChange() {
  digitalWrite(powerPin, HIGH);
  delay(300);
  for (int i = 0; i < 3; i++) {
    digitalWrite(lockPin, HIGH);
    delay(200);
    digitalWrite(lockPin, LOW);
    delay(200);
  }
  delay(300);
  digitalWrite(powerPin, LOW);
}
