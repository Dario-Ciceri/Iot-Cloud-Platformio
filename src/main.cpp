#include <Arduino.h>
#include <WiFiS3.h>
#include <PubSubClient.h>
#include <cbor.h>

// ************************ CONFIGURAZIONE ************************
const char WIFI_SSID[] = "test";
const char WIFI_PASSWORD[] = "12345678";

const char MQTT_BROKER[] = "192.168.1.243";
const int MQTT_PORT = 1883;
const char MQTT_CLIENT_ID[] = "arduino-uno-r4-wifi";
const char MQTT_USER[] = "serverpod";
const char MQTT_PASS[] = "serverpod";

const char PUB_TOPIC[] = "arduino-uno-r4-wifi/loopback";
const char SUB_TOPIC[] = "arduino-uno-r4-wifi/commands";

const int PUB_INTERVAL = 5000;
const int CONNECTION_RETRY = 15000;
const size_t CBOR_BUFFER_SIZE = 128;
// ***************************************************************

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

uint8_t cborBuffer[CBOR_BUFFER_SIZE];
unsigned long lastPubTime = 0;
unsigned long lastConnAttempt = 0;
bool wifiConnected = false;
bool mqttConnected = false;

// Prototipi
bool initWiFi();
bool initMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishData();
void checkConnections();
void safeDelay(unsigned long ms);
int getFreeMemory();

void setup() {
  Serial.begin(115200);
  while (!Serial); // Rimuovere in produzione
  
  Serial.println("\n[SYSTEM] Avvio Arduino UNO R4 WiFi...");
  
  // Configurazione specifica per UNO R4
  WiFi.setTimeout(10000);
  mqttClient.setBufferSize(CBOR_BUFFER_SIZE);
  mqttClient.setCallback(mqttCallback);
  
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
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

// ******************** FUNZIONI WIFI/MQTT ***********************
bool initWiFi() {
  Serial.println("\n[WiFi] Tentativo connessione...");
  
  WiFi.end();
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
  
  if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
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
  CborError err;
  CborEncoder encoder, mapEncoder;
  
  cbor_encoder_init(&encoder, cborBuffer, sizeof(cborBuffer), 0);
  
  // Crea mappa con 3 elementi
  err = cbor_encoder_create_map(&encoder, &mapEncoder, 3);
  if(err != CborNoError) {
    Serial.println("Errore creazione mappa CBOR");
    return;
  }

  // Timestamp
  cbor_encode_text_stringz(&mapEncoder, "ts");
  cbor_encode_uint(&mapEncoder, millis());

  // Valore analogico
  cbor_encode_text_stringz(&mapEncoder, "a0");
  cbor_encode_uint(&mapEncoder, analogRead(A0));

  // Memoria libera
  cbor_encode_text_stringz(&mapEncoder, "mem");
  cbor_encode_uint(&mapEncoder, getFreeMemory());

  cbor_encoder_close_container(&encoder, &mapEncoder);

  size_t cborLen = cbor_encoder_get_buffer_size(&encoder, cborBuffer);
  
  if(mqttClient.publish(PUB_TOPIC, cborBuffer, cborLen)) {
    Serial.print("[CBOR] Inviati ");
    Serial.print(cborLen);
    Serial.println(" bytes");
  } else {
    Serial.println("[CBOR] Errore trasmissione!");
  }
}

// ******************** GESTIONE MESSAGGI *************************
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n[MQTT] Ricevuto: ");
  Serial.println(topic);

  CborParser parser;
  CborValue value;
  CborError err = cbor_parser_init(payload, length, 0, &parser, &value);
  
  if(err || !cbor_value_is_map(&value)) {
    Serial.println("CBOR non valido");
    return;
  }

  CborValue mapItem;
  size_t mapSize;
  cbor_value_get_map_length(&value, &mapSize);
  cbor_value_enter_container(&value, &mapItem);

  for(size_t i = 0; i < mapSize; ++i) {
    if(cbor_value_is_text_string(&mapItem)) {
      char key[16];
      size_t keyLen = sizeof(key);
      cbor_value_copy_text_string(&mapItem, key, &keyLen, NULL);
      
      cbor_value_advance(&mapItem);
      
      if(strcmp(key, "led") == 0 && cbor_value_is_integer(&mapItem)) {
        int state;
        cbor_value_get_int(&mapItem, &state);
        digitalWrite(LED_BUILTIN, state);
        Serial.print("[LED] Impostato a: ");
        Serial.println(state);
      }
      cbor_value_advance(&mapItem);
    }
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
  char top;
  return &top - reinterpret_cast<char*>(malloc(1));
}