#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "broker.hivemq.com";

// MUST ADD PubSubClient to the library
WiFiClient espClient;
PubSubClient client(espClient);

const int BL = 5;
const int GD = 25;
const int GL = 12;
// create servo obj
Servo garage_door;
// opened and closed values
int opened = 180;
int closed = 0;

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  if (message == "GL1") {
    digitalWrite(GL, HIGH);
    Serial.println("Garage Light Turned ON");
  } else if (message == "GL0") {
    digitalWrite(GL, LOW);
    Serial.println("Garage Light Turned OFF");
  }
  if (message == "GD1") {
    garage_door.write(opened);
    Serial.println("Garage Door Opened");
  } else if (message == "GD0") {
    garage_door.write(closed);
    Serial.println("Garage Door Closed");
  }
  if (message == "BL1") {
    digitalWrite(BL, HIGH);
    Serial.println("Bedroom Light Turned ON");
  } else if (message == "BL0") {
    digitalWrite(BL, LOW);
    Serial.println("Bedroom Light Turned Off");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("WokwiClient")) {
      Serial.println("connected!");
      client.subscribe("home/central");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(1000);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // initialize the variables as outputs
  pinMode(BL, OUTPUT);
  pinMode(GL, OUTPUT);
  // attach servo
  garage_door.attach(GD);

  // connect to the internet
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}



void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  delay(15);
}
