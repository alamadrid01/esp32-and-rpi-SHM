#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "PubSubClient.h"
#include "WiFi.h"

#define DHTPIN 23
#define DHTTYPE DHT11

#define PIR_PIN 22
#define LIGHT_PIN 5

#define RELAY_PIN_1 26
#define RELAY_PIN_2 27
#define RELAY_PIN_3 14
#define RELAY_PIN_4 25

DHT dht(DHTPIN, DHTTYPE);

bool motionDetected = false;
bool printMotionStatus = false;

unsigned long motionTimestamp = 0;
unsigned long temperatureTimestamp = 0;
const unsigned long TEMPERATURE_INTERVAL = 2000;
const unsigned long MOTION_TIMEOUT = 35000;
const unsigned long ENERGY_INTERVAL = 1000 * 3600;

int motionValue = 0;

float sumPower = 0.0;
float accumulatedPower = 0;

const int analogInPin = 34;
const int analogInPin2 = 35;
const int analogInPin3 = 32;
const int analogInPin4 = 33;
const float referenceVoltage = 5.0;
const float ACS712Sensitivity = 0.66;
const float ACS712Sensitivity2 = 0.66;
const float ACS712Sensitivity3 = 0.66;
const float ACS712Sensitivity4 = 0.66;

const char* ssid = "****";
const char* wifi_password = "****";

const char* mqtt_server = "192.168.0.211";
const int mqtt_port = 1883;
const char* temperature_topic = "home/livingroom/temperature";
const char* humidity_topic = "home/livingroom/humidity";
const char* motion_topic = "home/livingroom/motion";
const char* power_topic_1 = "home/livingroom/power_1";
const char* power_topic_2 = "home/livingroom/power_2";
const char* power_topic_3 = "home/livingroom/power_3";
const char* power_topic_4 = "home/livingroom/power_4";
const char* relay_topic = "home/livingroom/control";
const char* summation_topic = "home/livingroom/summation";
const char* mqtt_username = "project";
const char* mqtt_password = "project";
const char* clientID = "clientid";

float startCount = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  dht.begin();

  pinMode(PIR_PIN, INPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LOW);

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
  pinMode(RELAY_PIN_4, OUTPUT);

  digitalWrite(RELAY_PIN_1, HIGH);
  digitalWrite(RELAY_PIN_2, HIGH);
  digitalWrite(RELAY_PIN_3, HIGH);
  digitalWrite(RELAY_PIN_4, HIGH);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message on topic '");
  payload[length] = '\0';
  String message = String((char*)payload);
  Serial.print(topic);
  Serial.print("': ");
  Serial.println(message);

  if (message == "FAN") {
    digitalWrite(RELAY_PIN_4, LOW);
  } else if (message == "OFFFAN") {
    digitalWrite(RELAY_PIN_4, HIGH);
  } else if (message == "LIGHT") {
    digitalWrite(RELAY_PIN_1, LOW);
  } else if (message == "OFFLIGHT") {
    digitalWrite(RELAY_PIN_1, HIGH);
  } else if (message == "SOCKET") {
    digitalWrite(RELAY_PIN_2, LOW);
  } else if (message == "OFFSOCKET") {
    digitalWrite(RELAY_PIN_2, HIGH);
  } else if (message == "EXTENSION") {
    digitalWrite(RELAY_PIN_3, LOW);
  } else if (message == "OFFEXTENSION") {
    digitalWrite(RELAY_PIN_3, HIGH);
  } else {
    Serial.println("Invalid message received.");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      Serial.println("");
      client.subscribe(relay_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

bool publishData(const char* topic, float value) {
  if (client.publish(topic, String(value, 2).c_str())) {
    Serial.print("Published ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(value);
    return true;
  } else {
    Serial.print("Failed to send data to ");
    Serial.println(topic);
    reconnect();
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  detectMotion();

  static unsigned long lastMsgTime = 0;
  if (millis() - lastMsgTime > 5000) {
    lastMsgTime = millis();

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    publishData(temperature_topic, t);
    publishData(humidity_topic, h);

    float power1 = calculatePower(analogInPin);   
    publishData(power_topic_1, power1);

    float power2 = calculatePower(analogInPin2);
    publishData(power_topic_2, power2);

    float power3 = calculatePower(analogInPin3);  
    publishData(power_topic_3, power3);

    float power4 = calculatePower(analogInPin4); 
    publishData(power_topic_4, power4);


    if (client.publish(motion_topic, String(motionValue).c_str())) {
      Serial.print("Motion sent: ");
      Serial.println(motionValue);
    } else {
      Serial.println("Motion failed to send. Reconnecting to MQTT Broker and trying again");
      reconnect();
      delay(10);
      client.publish(motion_topic, String(motionValue).c_str());
    }

    unsigned long elapsedTime = millis() - startCount; 

    if (elapsedTime >= ENERGY_INTERVAL) {
      float energy = (accumulatedPower) / 60;

      if (publishData(summation_topic, energy)) {
        Serial.print("Published energy per hour is : ");
        Serial.println(energy);
      }
      startCount = millis();
    }

    Serial.println("");
  }
}
float calculatePower(int relayPin) {
  int sensorIn = relayPin;
  int mVperAmp = 185; 
  double Voltage = 0;
  double Amps = 0;

  Voltage = analogRead(sensorIn) * (referenceVoltage / 4096.0);
  Amps = ((Voltage - referenceVoltage / 2.0) / mVperAmp) * ACS712Sensitivity;

  Serial.print("Power on Analog_Pin_");
  Serial.print(sensorIn);
  Serial.print(": ");
  Serial.print(Amps);
  Serial.print(" Amps  ---  ");
  float Watt = Amps * referenceVoltage;

  Serial.print(Watt);
  Serial.println(" Watts");

  return Watt;
}


void detectMotion() {
  if (digitalRead(PIR_PIN) == HIGH) {
    if (!motionDetected) {
      motionDetected = true;
      motionTimestamp = millis();
      digitalWrite(LIGHT_PIN, HIGH);
      digitalWrite(RELAY_PIN_1, LOW);
      printMotionStatus = true;
      motionValue = 1;
    }
  } else {
    if (motionDetected && millis() - motionTimestamp >= MOTION_TIMEOUT) {
      motionDetected = false;
      digitalWrite(LIGHT_PIN, LOW);
      digitalWrite(RELAY_PIN_1, HIGH);
      printMotionStatus = false;
      motionValue = 0;
    }
  }
}