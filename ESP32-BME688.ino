#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <PubSubClient.h>

#include <Adafruit_BME680.h>
#include <bme68x.h>
#include <bme68x_defs.h>

#include <Wire.h>
#include <WiFi.h>

int REFRESH = 10000;

int ledPin = 2;

const char* ssid = "CORK-WIFI";
const char* password = "XXXXXXX"; // replace with password from privatedetails.txt
const int mqttPort = 1883;
const char* mqttServer = "172.16.9.60";

String sensorType = "BME-688";

//OAK = 2, CSU = 3

String sensorID = "2";
const char* topic = "warehouse/OAK/env";

long lastMsg = 0;

long previousMillis = 0;
int interval = 2000;

WiFiClient espClient;
PubSubClient client(espClient);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;

void setup() {
  // put your setup code here, to run once:

  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);

  Serial.println("********************");
  Serial.println("BME-688 Environmental Sensor");
  Serial.println("Harv 81 Group");
  Serial.println("Greg Hirson");
  Serial.println("********************");


  //try to connect to WIFI
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);
  Serial.print("Connected at :");
  Serial.println(WiFi.localIP());

  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);

  delay(3000);

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms


  client.setServer(mqttServer, mqttPort);
}

void loop() {
  // put your main code here, to run repeatedly:

  // reconnect to MQTT server
  if (!client.connected()) {
    reconnect();
  }

  // reconnect to WiFi if connection dropped

  long currentMillis = millis();
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis) >= interval) {
    Serial.print(millis());
    Serial.println(" Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }


  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  //Serial.print(F("Reading started at "));
  //Serial.print(millis());
  //Serial.print(F(" and will finish at "));
  //Serial.println(endTime);

  //Serial.println(F("You can do other work during BME680 measurement."));
  //delay(50);  // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  //Serial.print(F("Reading completed at "));
  //Serial.println(millis());

  StaticJsonDocument<128> doc;
  char output[128];

  doc["sensor"] = sensorType;
  doc["sensorID"] = sensorID;
  doc["temp"] = bme.temperature;
  doc["pres"] = bme.pressure / 100.0;
  doc["hum"] = bme.humidity;
  doc["voc"] = bme.gas_resistance / 1000.0;

  serializeJson(doc, output);
  Serial.println(output);

  //Serial.print(F("Temperature = "));
  //Serial.print(bme.temperature);
  //Serial.println(F(" *C"));

  //  Serial.print(F("Pressure = "));
  //  Serial.print(bme.pressure / 100.0);
  //  Serial.println(F(" hPa"));
  //
  //  Serial.print(F("Humidity = "));
  //  Serial.print(bme.humidity);
  //  Serial.println(F(" %"));
  //
  //  Serial.print(F("Gas = "));
  //  Serial.print(bme.gas_resistance / 1000.0);
  //  Serial.println(F(" KOhms"));
  //
  //  Serial.print(F("Approx. Altitude = "));
  //  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  //  Serial.println(F(" m"));
  //
  //  Serial.println();

  client.publish(topic, output);

  delay(REFRESH);
}


void reconnect() {
  // Loop until reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create Client
    String clientID = "ESP32Client-";
    clientID += String(random(0xffff), HEX);;

    if (client.connect(clientID.c_str())) {
      Serial.println("connected");
      digitalWrite(ledPin, HIGH);
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      digitalWrite(ledPin, LOW);
      delay(2000);
    }
  }
}
