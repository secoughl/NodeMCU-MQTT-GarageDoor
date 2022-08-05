
#include "ESP8266WiFi.h"
#include "PubSubClient.h"
#include "ESP8266HTTPClient.h"
#include "secrets.h"

//Version, File, and HostName Info
float firmVersion = 1.0;
String fileLocation = "/src/1_commit/NodeMCU-MQTT-GarageDoor";
String hostName = "GarageController";

//WIFI and MQTT Info
const char* ssid = secret_ssid;
const char* password = secret_password;
const char* mqttServer = secret_mqttServer;
const int mqttPort = secret_mqttPort;
const char* mqttUser = secret_mqttUser;
const char* mqttPassword = secret_mqttPassword;

int seanSensorPin = D1;
int ericaSensorPin = D2;
int seanPin = D6;
int ericaPin = D5;
int seanDoorState = 0;
int ericaDoorState = 0;

//Declarey Needed Intervals
//One Day in Milis
const unsigned long rebootInterval = 86400000;
//Debug 5 minute Milis
//const unsigned long rebootInterval = 300000;
//30 seconds in Milis
const unsigned long updateInterval = 30000;
//5 second Millis
const unsigned long wifiUpdateInterval = 5000;

//Instantiate Timers
unsigned long previousTimeReboot = 0;
unsigned long previousTimeUpdate = 0;
unsigned long previousTimeWifiUpdate = 0;


WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Version tracking info
  Serial.print("Currently running version: ");
  Serial.println(firmVersion);

  Serial.print("located at: ");
  Serial.println(fileLocation);

  Serial.print("Hostname is: ");
  Serial.println(hostName);

  static const uint8_t D0   = 16;
  static const uint8_t D1   = 5;
  static const uint8_t D2   = 4;
  static const uint8_t D3   = 0;
  static const uint8_t D4   = 2;
  static const uint8_t D5   = 14;
  static const uint8_t D6   = 12;
  static const uint8_t D7   = 13;
  static const uint8_t D8   = 15;
  static const uint8_t D9   = 3;
  static const uint8_t D10  = 1;
  pinMode(seanSensorPin, INPUT_PULLUP);
  pinMode(ericaSensorPin, INPUT_PULLUP);

  digitalWrite(seanPin, LOW);
  digitalWrite(ericaPin, LOW);

  pinMode(seanPin, OUTPUT);
  pinMode(ericaPin, OUTPUT);



  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostName.c_str());
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP8266 Local IP Address
  Serial.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("GarageController", mqttUser, mqttPassword )) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }
  client.subscribe("doors/garage");
}


void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  seanDoorState = digitalRead(seanSensorPin);
  ericaDoorState = digitalRead(ericaSensorPin);
  //debug delay
  //delay(1000);

  //Reboot every 24 Hours
  if (currentTime - previousTimeReboot >= rebootInterval) {
    /* Event code */
    Serial.println("It has been One Day, rebooting");
    ESP.restart();

    /* Update the timing for the next time around */
    previousTimeReboot = currentTime;
  }
  //Check wifi state every 5 seconds
  if (currentTime - previousTimeWifiUpdate >= wifiUpdateInterval) {
    /* Event code */
    Serial.println("5 Seconds have elapsed, checking WIFI State");
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Reconnecting to WiFi...");
      ESP.restart();
    }
    /* Update the timing for the next time around */
    previousTimeWifiUpdate = currentTime;
  }
  if (currentTime - previousTimeUpdate >= updateInterval) {
    /* Event code */
    Serial.println("It has been thirty seconds, Publishing state to MQTT");
    if (seanDoorState == 1) {
      Serial.println("Sean Door is Closed");
      client.publish("doors/garage/Sean", "Closed");
    } else {
      Serial.println("Sean Door is Open");
      client.publish("doors/garage/Sean", "Open");
    }
    //debug delay
    // delay(1000);
    if (ericaDoorState == 1) {
      Serial.println("Erica Door is Closed");
      client.publish("doors/garage/Erica", "Closed");
    } else {
      Serial.println("Erica Door is Open");
      client.publish("doors/garage/Erica", "Open");
    }
    previousTimeUpdate = currentTime;
  }
  //MQTT Call for HouseKeeping
  client.loop();
  //Sleep to let the ESP SDK do whatever the fuck it needs to.
  delay(100);
}

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  Serial.print("Message:");
  //  for (int i = 0; i < length; i++) {
  //    Serial.print((char)payload[i]);
  //  }

  Serial.println();
  Serial.println("-----------------------");
  //Formatting with a null at length for ...reasons?

  payload[length] = '\0';
  String recv_payload = String(( char *) payload);

  Serial.print("payload: [");
  Serial.print((char *)payload);
  Serial.println("]");
  Serial.println(recv_payload);


  //CloseAll states
  if (recv_payload == "CloseAll") {

    if (seanDoorState == 0) {
      Serial.println ("Would close Sean on CloseAll");
      digitalWrite(seanPin, HIGH);
      delay(500);
      digitalWrite(seanPin, LOW);
      delay(3000);
    }

    if (ericaDoorState == 0) {
      Serial.println ("Would close Erica on CloseAll");
      digitalWrite(ericaPin, HIGH);
      delay(500);
      digitalWrite(ericaPin, LOW);
    }
  }
  //OpenAll States
  if (recv_payload == "OpenAll") {

    if (seanDoorState == 1) {
      Serial.println ("Would open Sean on OpenAll");
      digitalWrite(seanPin, HIGH);
      delay(500);
      digitalWrite(seanPin, LOW);
      delay(3000);
    }

    if (ericaDoorState == 1) {
      Serial.println ("Would open Erica on OpenAll");
      digitalWrite(ericaPin, HIGH);
      delay(500);
      digitalWrite(ericaPin, LOW);
    }
  }

  //Change Erica Door State Arbitrarily
  if (recv_payload == "Erica") {
    Serial.println ("Would state change Erica");
    digitalWrite(ericaPin, HIGH);
    delay(100);
    digitalWrite(ericaPin, LOW);
  }
  //Change Sean Door State Arbitrarily
  if (recv_payload == "Sean") {
    Serial.println ("Would state change Sean");
    digitalWrite(seanPin, HIGH);
    delay(100);
    digitalWrite(seanPin, LOW);
  }
  //Fancier States
  //Open on request open if closed
  if ((recv_payload == "EricaOpen") && (ericaDoorState == 1)) {
    Serial.println ("Open Erica");
    digitalWrite(ericaPin, HIGH);
    delay(100);
    digitalWrite(ericaPin, LOW);
  }
  //Open on request open if closed
  if ((recv_payload == "SeanOpen") && (seanDoorState == 1)) {
    Serial.println ("Open Sean");
    digitalWrite(seanPin, HIGH);
    delay(100);
    digitalWrite(seanPin, LOW);
  }
  //Close on request close if Open
  if ((recv_payload == "EricaClose") && (ericaDoorState == 0)) {
    Serial.println ("Close Erica");
    digitalWrite(ericaPin, HIGH);
    delay(100);
    digitalWrite(ericaPin, LOW);
  }
  //Close on request close if Open
  if ((recv_payload == "SeanClose") && (seanDoorState == 0)) {
    Serial.println ("Close Sean");
    digitalWrite(seanPin, HIGH);
    delay(100);
    digitalWrite(seanPin, LOW);
  }
}
