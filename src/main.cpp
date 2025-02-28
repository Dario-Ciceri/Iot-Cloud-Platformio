#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFiS3.h>
#include <PubSubClient.h>

// ************************ CONFIGURAZIONE ************************
const char WIFI_SSID[] = "test";
const char WIFI_PASSWORD[] = "12345678";

const char MQTT_BROKER[] = "192.168.1.243";
const int MQTT_PORT = 1883;
const char MQTT_CLIENT_ID[] = "arduino-uno-r4-wifi";
const char MQTT_USER[] = "serverpod";
const char MQTT_PASS[] = "serverpod";

const char PUB_TOPIC[] = "arduino/loopback";
const char SUB_TOPIC[] = "arduino/commands";

const int PUB_INTERVAL = 5000;
const int CONNECTION_RETRY = 15000;
// ***************************************************************

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastPubTime = 0;
unsigned long lastConnAttempt = 0;
bool wifiConnected = false;
bool mqttConnected = false;

// Prototipi modificati per UNO R4
bool initWiFi();
bool initMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishData();
void checkConnections();
void safeDelay(unsigned long ms);
int getFreeMemory();

void setup() {
  Serial.begin(115200);
  while (!Serial); // Rimuovere questa linea in produzione
  
  Serial.println("\n[SYSTEM] Avvio Arduino UNO R4 WiFi...");
  
  // Configurazione specifica per UNO R4
  WiFi.setTimeout(10000); // Timeout aumentato per compatibilità
  mqttClient.setBufferSize(256);
  mqttClient.setKeepAlive(30);
  mqttClient.setCallback(mqttCallback);
  
  // LED integrato
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  unsigned long now = millis();

  if (!wifiConnected || (now - lastConnAttempt > CONNECTION_RETRY)) {
    checkConnections();
    lastConnAttempt = now;
  }

  if (mqttConnected) {
    mqttClient.loop();
    
    if (now - lastPubTime >= PUB_INTERVAL) {
      publishData();
      lastPubTime = now;
    }
  }

  safeDelay(100);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Heartbeat LED
}

// ******************** FUNZIONI WIFI/MQTT ***********************
bool initWiFi() {
  Serial.println("\n[WiFi] Tentativo connessione...");
  
  WiFi.end(); // Metodo corretto per reset su UNO R4
  delay(1000);
  
  int status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  for (int i = 0; i < 20; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("\n[WiFi] Connesso! RSSI: ");
      Serial.println(WiFi.RSSI());
      return true;
    }
    Serial.print(".");
    safeDelay(500);
  }
  
  Serial.println("\n[WiFi] ERRORE: Controllare credenziali o segnale");
  return false;
}

bool initMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  
  Serial.println("\n[MQTT] Connessione al broker...");
  
  if (mqttClient.connect(
    MQTT_CLIENT_ID, 
    MQTT_USER, 
    MQTT_PASS
  )) {
    Serial.println("[MQTT] Connesso!");
    mqttClient.subscribe(SUB_TOPIC);
    return true;
  }
  
  Serial.print("[MQTT] Errore: ");
  switch(mqttClient.state()) {
    case -4: Serial.println("Timeout connessione"); break;
    case -3: Serial.println("Connessione persa"); break;
    case -2: Serial.println("Connessione fallita"); break;
    case -1: Serial.println("Disconnesso"); break;
    default: Serial.println("Sconosciuto");
  }
  return false;
}

void publishData() {
  StaticJsonDocument<128> doc; // Ridotto per UNO R4
  doc["ts"] = millis();
  doc["a0"] = analogRead(A0);
  doc["mem"] = getFreeMemory();

  char buffer[128];
  size_t len = serializeJson(doc, buffer);

  if (mqttClient.publish(PUB_TOPIC, buffer, len)) {
    Serial.print("[MQTT] Inviati ");
    Serial.print(len);
    Serial.println(" bytes");
  } else {
    Serial.println("[MQTT] Errore trasmissione!");
  }
}

// ******************** GESTIONE MESSAGGI *************************
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n[MQTT] Ricevuto: ");
  Serial.println(topic);

  StaticJsonDocument<64> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.print("[JSON] Errore: ");
    Serial.println(error.c_str());
    return;
  }

  // Esempio comando: {"led":1}
  if (doc.containsKey("led")) {
    int state = doc["led"];
    digitalWrite(LED_BUILTIN, state);
    Serial.print("[LED] Impostato a: ");
    Serial.println(state);
  }
}

// *********************** UTILITIES ******************************
void checkConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    mqttConnected = false;
    wifiConnected = initWiFi();
  }
  
  if (wifiConnected && !mqttClient.connected()) {
    mqttConnected = initMQTT();
  }
}

void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    delay(1);
    if (mqttConnected) mqttClient.loop();
  }
}

int getFreeMemory() {
  // Metodo affidabile per UNO R4
  char top;
  return &top - reinterpret_cast<char*>(malloc(1));
}