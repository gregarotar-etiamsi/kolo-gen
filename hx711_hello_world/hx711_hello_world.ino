// HX711 calibration program

#include "HX711.h"
#include <WiFi.h>
#include <PubSubClient.h>

const char *ssid = "K2_ext";
const char *pass = "PoveziMeNaK2!";

// MQTT Broker
const char *mqtt_broker = "192.168.1.211";
const int mqtt_port = 1883;
const char *topic = "test/secure";
const char *mqtt_username = "arduino";
const char *mqtt_password = "pass";

#define DOUT_PIN 16 // Replace with your chosen GPIO pin
#define SCK_PIN 4   // Replace with your chosen GPIO pin

HX711 scale;

#define TARE_CYCLES 10 // nastavi po potrebi in ne pozabi, da vsaka številka pomeni 100ms več časa


WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  Serial.begin(115200);
  scale.begin(DOUT_PIN, SCK_PIN);
  scale.tare();
   WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected to WiFi");
  // connecting to a MQTT broker START
    client.setServer(mqtt_broker, mqtt_port);

    // loop until client is connected
    while (!client.connected())
    {
        String client_id = generateRandomClientId();

        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
        {
            Serial.println("Public EMQX MQTT broker connected");
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void loop() {
  if (scale.is_ready()) {
    unsigned int startTime = millis();
    long weight = scale.get_units(2);  // Read 10 times for better stability
    Serial.println(millis() - startTime);
    Serial.println("Weight: " + String(weight) + " units");
    client.publish(topic, String(weight).c_str());
    
  } else {
    Serial.println("Error reading from HX711");
  }
    delay(100);
}




char generateRandomChar() {
    const char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    const int charsetSize = sizeof(charset) - 1;

    // Generate a random index within the character set
    int randomIndex = rand() % charsetSize;

    // Return the random character
    return charset[randomIndex];
}

String generateRandomClientId() {
    String clientId = "ESP32Client-";
    for (int i = 0; i < 8; ++i) {
        clientId += generateRandomChar();
    }
    return clientId;
}

