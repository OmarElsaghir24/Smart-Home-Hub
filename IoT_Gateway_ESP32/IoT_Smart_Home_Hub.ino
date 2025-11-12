#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

// MOSI: IO23, MISO: IO19, SCLK: IO18
#define CE_PIN  22
#define CSN_PIN 21

// Unique identifier for HiveMQ should be different than the one used in code
const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN);

uint16_t dataReceived[32]; // this must match dataToSend in the TX
bool newData = false;

// Replace the next variables with your SSID/Password combination
const char* ssid = "SSID";
const char* password = "PASSWORD";

// Add your MQTT Broker address, example:
const char* mqtt_server = "broker.hivemq.com";
const char* unique_identifier = "clientId-jiszKPkuHf";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
int value = 0;
unsigned long lastPublish = 0;
const unsigned long publishInterval = 3000; // 3 seconds

// LED Pin
const int ledPin = 4;

void setup() {
  Serial.begin(115200);

  // default settings
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  delay(3000);
   Serial.println("SimpleRx Starting");
   radio.begin();
   radio.setChannel(76);             // Use channel 76 for communication
   radio.setPALevel(RF24_PA_LOW);    // reduce noise
   radio.setDataRate(RF24_1MBPS);    // Communication set at a data rate of 1 Mbps
   radio.setAutoAck(false);          // disable auto acknowledge
   radio.disableDynamicPayloads();        // fixed payload
   radio.setPayloadSize(32);              // Payload of packet being transmitted
   radio.openReadingPipe(0, thisSlaveAddress);
   radio.startListening();
   radio.printDetails();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(unique_identifier)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("SF/LED");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void getData()
{
   if ( radio.available() )
   {
      radio.read( &dataReceived, sizeof(dataReceived) );
      newData = true;
   }
}

void showData()
{
   if (newData == true)
   {
      Serial.print("Data received ");
      for (int i = 0; i < sizeof(dataReceived); i++) {
        Serial.print(dataReceived[i], DEC); // print as decimal
        Serial.print(" ");
      }
      Serial.println();
      newData = false;
   }
}

void publishData()
{
  uint8_t temperature = dataReceived[2];
  uint8_t humidity = dataReceived[3];
  uint16_t pressure = dataReceived[4];
  uint8_t motion = dataReceived[5];
  uint8_t light_level = dataReceived[6];
  uint8_t door_status = dataReceived[7];

  char payload[32];

  // Publish temperature
  snprintf(payload, sizeof(payload), "%d", temperature);
  client.publish("home/temperature", payload);

  // Publish humidity
  snprintf(payload, sizeof(payload), "%d", humidity);
  client.publish("home/humidity", payload);

  // Publish pressure
  snprintf(payload, sizeof(payload), "%d", pressure);
  client.publish("home/pressure", payload);

  // Publish motion
  snprintf(payload, sizeof(payload), "%d", motion);
  if(motion == 1)
  {
    client.publish("home/motion/status", "Detected");
  }
  else
  {
    client.publish("home/motion/status", "Idle");
  }

  // Publish light level
  snprintf(payload, sizeof(payload), "%d", light_level);
  client.publish("home/brightness", payload);

  snprintf(payload, sizeof(payload), "%d", door_status);
  if(door_status == 1)
  {
    client.publish("home/door/status", "Door Unlocked");
  }
  else
  {
    client.publish("home/door/status", "Door Locked");
  }

  Serial.println("Published to MQTT:");
  Serial.print("  Temperature: "); Serial.println(temperature);
  Serial.print("  Humidity: ");    Serial.println(humidity);
  Serial.print("  Pressure: ");    Serial.println(pressure);
  Serial.print("  Motion: ");      Serial.println(motion);
  Serial.print("  Brightness: ");  Serial.println(light_level);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  getData();
  showData();
  // Check if it's time to publish
  if (millis() - lastPublish >= publishInterval)
  {
      publishData();
      lastPublish = millis();
  }
}
