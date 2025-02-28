#include <Arduino.h>
#include <PubSubClient.h>
#include <cbor.h>

// Platform-specific includes
#ifdef ESP32
  #include <WiFi.h>
  #define LED_PIN 2
#else // Arduino UNO R4 WiFi
  #include <WiFiS3.h>
  #define LED_PIN LED_BUILTIN
#endif

// *********************** CONFIGURAZIONE ************************
const char WIFI_SSID[] = "test";
const char WIFI_PASSWORD[] = "12345678";

const char MQTT_BROKER[] = "192.168.1.243";
const int MQTT_PORT = 1883;
const char MQTT_USER[] = "serverpod";
const char MQTT_PASS[] = "serverpod";

// Platform-specific MQTT settings
#ifdef ESP32
  const char MQTT_CLIENT_ID[] = "esp32-device";
  const char PUB_TOPIC[] = "esp32/loopback";
  const char SUB_TOPIC[] = "esp32/commands";
  const size_t CBOR_BUFFER_SIZE = 256;  // ESP32 ha più memoria
#else
  const char MQTT_CLIENT_ID[] = "arduino-uno-r4-wifi";
  const char PUB_TOPIC[] = "arduino-uno-r4-wifi/loopback";
  const char SUB_TOPIC[] = "arduino-uno-r4-wifi/commands";
  const size_t CBOR_BUFFER_SIZE = 128;
#endif

const int PUB_INTERVAL = 5000;
const int CONNECTION_RETRY = 15000;
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
  
  #ifndef ESP32
  // Solo per UNO R4: wait for serial port to connect
  while (!Serial); // Rimuovere in produzione
  #else
  delay(1000); // ESP32: breve attesa senza bloccare
  #endif
  
  Serial.print("\n[SYSTEM] Avvio ");
  Serial.print(BOARD_NAME);
  Serial.println("...");
  
  // Configurazione specifica per piattaforma
  #ifdef ESP32
    // Eventuali configurazioni specifiche per ESP32
  #else
    // Configurazioni specifiche per UNO R4
    WiFi.setTimeout(10000);
  #endif
  
  mqttClient.setBufferSize(CBOR_BUFFER_SIZE);
  mqttClient.setCallback(mqttCallback);
  
  pinMode(LED_PIN, OUTPUT);
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
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// ******************** FUNZIONI WIFI/MQTT ***********************
bool initWiFi() {
  Serial.println("\n[WiFi] Tentativo connessione...");
  
  #ifdef ESP32
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    delay(1000);
  #else
    WiFi.end();
    delay(1000);
  #endif
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  for (int i = 0; i < 20; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("\n[WiFi] Connesso! RSSI: ");
      Serial.println(WiFi.RSSI());
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
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
  
  // Crea mappa con elementi (più per ESP32)
  #ifdef ESP32
  err = cbor_encoder_create_map(&encoder, &mapEncoder, 4);
  #else
  err = cbor_encoder_create_map(&encoder, &mapEncoder, 3);
  #endif
  
  if(err != CborNoError) {
    Serial.println("Errore creazione mappa CBOR");
    return;
  }

  // Timestamp
  cbor_encode_text_stringz(&mapEncoder, "ts");
  cbor_encode_uint(&mapEncoder, millis());

  // Valore analogico
  #ifdef ESP32
  cbor_encode_text_stringz(&mapEncoder, "a0");
  cbor_encode_uint(&mapEncoder, analogRead(36)); // ADC1_CHANNEL_0 su ESP32
  
  // Aggiunta di un valore specifico per ESP32
  cbor_encode_text_stringz(&mapEncoder, "hall");
  cbor_encode_int(&mapEncoder, hallRead()); // Sensore effetto Hall (solo ESP32)
  #else
  cbor_encode_text_stringz(&mapEncoder, "a0");
  cbor_encode_uint(&mapEncoder, analogRead(A0));
  #endif

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
        digitalWrite(LED_PIN, state);
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
  #ifdef ESP32
    return ESP.getFreeHeap();
  #else
    char top;
    return &top - reinterpret_cast<char*>(malloc(1));
  #endif
}