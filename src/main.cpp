/*********************************************************************
 * MQTT OTA Firmware Updater per Arduino UNO R4 WiFi e ESP32
 * 
 * DESCRIZIONE:
 * Firmware ottimizzato per ricevere comandi di aggiornamento via MQTT.
 * Gestisce sia URL completi che campi separati (host, port, path).
 * Include gestione robusta degli errori e ottimizzazione memoria.
 * 
 * CARATTERISTICHE:
 * - Supporto multi-piattaforma (UNO R4 WiFi e ESP32)
 * - Ricezione comandi aggiornamento tramite MQTT con targeting dispositivo
 * - Compatibilità con il formato CBOR generato dal client Dart
 * - Debug completo su seriale e MQTT
 * - Download HTTP con supporto per trasferimenti normali e chunked
 * - Gestione robusta degli errori e riavvii
 * - Prevenzione doppi aggiornamenti simultanei
 * 
 * Versione: 2.1.2
 * Data: 07 Marzo 2025
 * Autore: Dario Ciceri (ottimizzato)
 *********************************************************************/

// Abilita debug avanzato
#define DEBUG_ENABLED 1

// Usa macro F() per memorizzare stringhe in flash
#define PROGMEM_STR(s) F(s)

// Inclusione librerie specifiche per piattaforma
#ifdef ESP32
  #include <WiFi.h>
  #include <Update.h>
  #include <esp_ota_ops.h>
  #include <esp_partition.h>
  #define LED_PIN 2
  #define BOARD_NAME "ESP32"
#else // Arduino UNO R4 WiFi
  #include <WiFiS3.h>
  #define NO_OTA_NETWORK
  #include <ArduinoOTA.h>
  #define LED_PIN LED_BUILTIN
  #define BOARD_NAME "Arduino UNO R4 WiFi"
#endif

// Librerie comuni
#include <PubSubClient.h>
#include <CRC32.h>
#include <cbor.h>

/*** CONFIGURAZIONI ***/

// Configurazione WiFi
const char* WIFI_SSID = "test";
const char* WIFI_PASSWORD = "12345678";

// Configurazione MQTT
const char* MQTT_BROKER = "192.168.1.25";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "serverpod";
const char* MQTT_PASS = "serverpod";

// Identificazione dispositivo
// Questo ID deve essere univoco per ogni dispositivo nella rete
#ifdef ESP32
  const char* DEVICE_ID = "esp32-device";
  const char* DEVICE_TYPE = "ESP32";
#else
  const char* DEVICE_ID = "UNOWIFIR4";
  const char* DEVICE_TYPE = "ARDUINO_UNO_R4_WIFI";
#endif

// MQTT Topics
const char* OTA_COMMAND_TOPIC = "firmware/update/command";   // Riceve comandi di aggiornamento
const char* OTA_RESPONSE_TOPIC = "firmware/update/response"; // Invia risposte ai comandi
const char* DEBUG_TOPIC = "debug/";                          // Prefisso per i messaggi di debug
const char* STATUS_TOPIC = "status/";                        // Prefisso per i messaggi di stato
const char* PING_TOPIC = "ping/";                            // Prefisso per i messaggi di ping

// OTA Settings
const int DOWNLOAD_TIMEOUT = 60000;          // Timeout download (60 secondi)
const size_t BUFFER_SIZE = 1024;             // Dimensione buffer lettura (1KB)
const size_t CBOR_BUFFER_SIZE = 1536;        // Dimensione buffer CBOR - ridotta per risparmiare memoria

// Time tracking
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastMqttActivity = 0;         // Time of last MQTT activity
const int MQTT_PING_INTERVAL = 10000;       // Send ping every 10 seconds if inactive
const int RECONNECT_INTERVAL = 5000;        // Intervallo riconnessione in ms

/*** VARIABILI GLOBALI ***/

// Client WiFi e MQTT
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Client WiFi per download OTA
WiFiClient otaClient;

// Buffer CBOR - allocato dinamicamente solo quando necessario
uint8_t cborBuffer[CBOR_BUFFER_SIZE];

// Oggetto per calcolo CRC32
CRC32 crc;

// Stato aggiornamento
volatile bool updateInProgress = false;        // Modificato a volatile per evitare ottimizzazioni del compilatore
volatile unsigned long updateStartTime = 0;    // Per tracking dell'aggiornamento
volatile unsigned long lastCommandId = 0;      // Per evitare elaborazione duplicati

// Buffer per download
uint8_t downloadBuffer[BUFFER_SIZE];

// Parametri firmware da aggiornare
struct {
  char targetDevice[32];      // ID dispositivo destinatario
  char firmwareUrl[384];      // URL del firmware (costruito o ricevuto)
  char serverHost[64];        // Host del server
  int serverPort;             // Porta del server
  char serverPath[512];       // Path sul server - aumentata per supportare path più lunghe
  long contentLength;         // Dimensione prevista del firmware
  bool chunkedTransfer;       // Indica se il trasferimento è chunked
} updateParams;

/*** PROTOTIPI FUNZIONI ***/

// Funzioni di debug
void serialDebug(const char* message);
void serialDebugf(const char* format, ...);
void printHex(const uint8_t* data, size_t length);

// Funzioni di connessione
bool initWiFi();
bool initMQTT();
void reconnect();

// Funzioni MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length);
void parseUpdateCommand(byte* payload, unsigned int length);
void publishDebug(const char* message);
void publishDebugf(const char* format, ...);
void publishProgress(unsigned long current, unsigned long total);
void publishResponse(const char* status, const char* message);
void publishPing();

// Funzioni OTA
bool downloadAndUpdate();
bool startFirmwareUpdate(size_t contentLength);
bool finishFirmwareUpdate();
void abortFirmwareUpdate();
size_t getMaxFirmwareSize();
bool parseUrl(const char* url, char* host, int* port, char* path);
void constructUrl(const char* host, int port, const char* path, char* url, size_t urlSize);

// Funzioni di sistema
void blinkLED(int times, int delayMs);
int getFreeMemory();

// Prevenzione elaborazioni duplicate
bool isMessageDuplicate(unsigned long commandTimestamp);

/**
 * Setup - inizializzazione dispositivo
 */
void setup() {
  // Inizializzazione comunicazione seriale
  Serial.begin(115200);
  delay(2000);
  
  // Configurazione LED
  pinMode(LED_PIN, OUTPUT);
  
  serialDebug("\n\n*********************************************");
  serialDebug("*    MQTT OTA FIRMWARE UPDATER v2.1.2     *");
  serialDebug("*********************************************");
  
  serialDebugf("\n[SYSTEM] Avvio %s...", BOARD_NAME);
  serialDebugf("Device ID: %s", DEVICE_ID);
  serialDebugf("MQTT broker: %s:%d", MQTT_BROKER, MQTT_PORT);
  serialDebugf("Command topic: %s", OTA_COMMAND_TOPIC);
  serialDebugf("Memoria libera: %d bytes", getFreeMemory());
  
  // Reset dello stato di aggiornamento
  updateInProgress = false;
  updateStartTime = 0;
  lastCommandId = 0;
  
  // Configurazione MQTT client
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(CBOR_BUFFER_SIZE * 2); // Buffer per messaggi MQTT
  mqttClient.setKeepAlive(MQTT_KEEPALIVE);
  
  // Inizializzazione WiFi e MQTT
  if (initWiFi()) {
    initMQTT();
  }
  
  // Segnalazione visiva di avvio completato
  blinkLED(3, 200);
  
  serialDebug("Inizializzazione completata, in attesa di comandi...");
}

/**
 * Loop - ciclo principale
 */
void loop() {
  unsigned long currentMillis = millis();
  
  // Verifica connessione WiFi
  if (WiFi.status() != WL_CONNECTED) {
    // Riprova al massimo ogni 5 secondi
    if (currentMillis - lastMqttReconnectAttempt > RECONNECT_INTERVAL) {
      reconnect();
      lastMqttReconnectAttempt = currentMillis;
    }
  } 
  // Verifica connessione MQTT
  else if (!mqttClient.connected()) {
    // Limita i tentativi di riconnessione a 1 ogni 5 secondi
    if (currentMillis - lastMqttReconnectAttempt > RECONNECT_INTERVAL) {
      lastMqttReconnectAttempt = currentMillis;
      if (initMQTT()) {
        lastMqttActivity = currentMillis;  // Reset activity timer on successful connection
      }
    }
  } 
  // Gestione connessione attiva
  else {
    // Loop MQTT client (processa messaggi in arrivo) solo se non è in corso un aggiornamento
    if (!updateInProgress) {
      mqttClient.loop();
      
      // Invia ping periodico per mantenere la connessione attiva
      if (currentMillis - lastMqttActivity > MQTT_PING_INTERVAL) {
        // Invia un ping MQTT (messaggio di keep-alive) con CBOR
        publishPing();
        lastMqttActivity = currentMillis;
      }
    }
  }
  
  // Toggle LED per indicare funzionamento
  static unsigned long lastBlink = 0;
  if (currentMillis - lastBlink > 1000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = currentMillis;
  }
  
  // Evita delay fissi che potrebbero bloccare operazioni MQTT
  yield();  // Consente alla CPU di gestire altre operazioni in background
}

/**
 * Funzione di debug su seriale
 */
void serialDebug(const char* message) {
  if (DEBUG_ENABLED) {
    Serial.println(message);
  }
}

/**
 * Funzione di debug formattato su seriale
 */
void serialDebugf(const char* format, ...) {
  if (DEBUG_ENABLED) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    Serial.println(buffer);
  }
}

/**
 * Stampa dati in formato esadecimale 
 */
void printHex(const uint8_t* data, size_t length) {
  if (DEBUG_ENABLED) {
    char buffer[8];
    serialDebug("HEX dump:");
    for (size_t i = 0; i < length; i++) {
      if (i > 0) {
        if (i % 16 == 0) {
          Serial.println();
        } else if (i % 8 == 0) {
          Serial.print("  ");
        } else {
          Serial.print(" ");
        }
      }
      snprintf(buffer, sizeof(buffer), "%02X", data[i]);
      Serial.print(buffer);
    }
    Serial.println();
  }
}

/**
 * Inizializzazione WiFi
 * @return true se connesso, false altrimenti
 */
bool initWiFi() {
  serialDebug("Connessione WiFi...");
  
  #ifdef ESP32
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    delay(1000);
  #else
    WiFi.end();
    delay(1000);
  #endif
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Attende connessione con timeout
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    serialDebug("WiFi connesso!");
    serialDebugf("Indirizzo IP: %d.%d.%d.%d", 
                WiFi.localIP()[0], WiFi.localIP()[1], 
                WiFi.localIP()[2], WiFi.localIP()[3]);
    return true;
  } else {
    Serial.println();
    serialDebug("Errore: connessione WiFi fallita");
    return false;
  }
}

/**
 * Inizializzazione MQTT
 * @return true se connesso, false altrimenti
 */
bool initMQTT() {
  serialDebug("Connessione MQTT...");
  serialDebugf("Broker: %s:%d", MQTT_BROKER, MQTT_PORT);
  serialDebugf("Client ID: %s", DEVICE_ID);
  
  // Configura opzioni MQTT
  mqttClient.setKeepAlive(MQTT_KEEPALIVE);
  
  // Disconnetti eventuali connessioni precedenti ancora attive
  if (mqttClient.connected()) {
    mqttClient.disconnect();
    delay(100);  // Breve attesa per completare la disconnessione
  }
  
  // Crea ID Client (senza suffisso random per maggiore stabilità)
  String clientId = String(DEVICE_ID);
  
  // Opzioni di connessione
  bool cleanSession = true;
  
  // Tentativo di connessione con opzioni LWT (Last Will and Testament)
  // LWT permette al broker di notificare altri client se questo dispositivo si disconnette inaspettatamente
  String willTopic = String(STATUS_TOPIC) + DEVICE_ID;
  
  // Prepara messaggio LWT in CBOR
  CborEncoder willEncoder, willMapEncoder;
  cbor_encoder_init(&willEncoder, cborBuffer, sizeof(cborBuffer), 0);
  cbor_encoder_create_map(&willEncoder, &willMapEncoder, 2);
  cbor_encode_text_stringz(&willMapEncoder, "device");
  cbor_encode_text_stringz(&willMapEncoder, DEVICE_ID);
  cbor_encode_text_stringz(&willMapEncoder, "status");
  cbor_encode_text_stringz(&willMapEncoder, "offline");
  cbor_encoder_close_container(&willEncoder, &willMapEncoder);
  size_t willLen = cbor_encoder_get_buffer_size(&willEncoder, cborBuffer);
  
  int mqttResult;
  // Correzione firma metodo connect() - rimosso parametro willLen
  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, 
                         willTopic.c_str(), 0, true, (const char*)cborBuffer, cleanSession)) {
    serialDebug("MQTT connesso!");
    
    // Sottoscrizione topic comandi OTA
    serialDebugf("Sottoscrizione a %s", OTA_COMMAND_TOPIC);
    if (mqttClient.subscribe(OTA_COMMAND_TOPIC)) {
      serialDebug("Sottoscrizione completata con successo");
    } else {
      serialDebug("ERRORE: Sottoscrizione fallita");
    }
    
    // Pubblica messaggio di stato online (usando CBOR)
    CborEncoder statusEncoder, statusMapEncoder;
    cbor_encoder_init(&statusEncoder, cborBuffer, sizeof(cborBuffer), 0);
    cbor_encoder_create_map(&statusEncoder, &statusMapEncoder, 2);
    cbor_encode_text_stringz(&statusMapEncoder, "device");
    cbor_encode_text_stringz(&statusMapEncoder, DEVICE_ID);
    cbor_encode_text_stringz(&statusMapEncoder, "status");
    cbor_encode_text_stringz(&statusMapEncoder, "online");
    cbor_encoder_close_container(&statusEncoder, &statusMapEncoder);
    size_t statusLen = cbor_encoder_get_buffer_size(&statusEncoder, cborBuffer);
    mqttClient.publish(willTopic.c_str(), cborBuffer, statusLen, true);  // retained=true
    
    // Pubblica messaggio di avvio utilizzando publishDebug invece di direct publish
    publishDebug("Dispositivo avviato e pronto per aggiornamenti OTA");
    
    lastMqttActivity = millis();  // Resetta timer attività MQTT
    return true;
  } else {
    mqttResult = mqttClient.state();
    serialDebugf("Errore MQTT: %d", mqttResult);
    switch (mqttResult) {
      case -4: serialDebug("Timeout connessione"); break;
      case -3: serialDebug("Connessione persa"); break;
      case -2: serialDebug("Connessione fallita"); break;
      case -1: serialDebug("Client disconnesso"); break;
      case 1: serialDebug("Versione protocollo non supportata"); break;
      case 2: serialDebug("Client ID rifiutato"); break;
      case 3: serialDebug("Server non disponibile"); break;
      case 4: serialDebug("Username/Password non validi"); break;
      case 5: serialDebug("Non autorizzato"); break;
      default: serialDebug("Errore sconosciuto");
    }
    return false;
  }
}

/**
 * Riconnessione in caso di perdita connessione
 */
void reconnect() {
  // Se è in corso un aggiornamento, non tentare riconnessione
  if (updateInProgress) {
    return;
  }
  
  // Riconnessione WiFi
  if (WiFi.status() != WL_CONNECTED) {
    serialDebug("Riconnessione WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    delay(5000); // Attesa per connessione
  }
  
  // Riconnessione MQTT
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    initMQTT();
  }
}

/**
 * Callback per messaggi MQTT ricevuti
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Aggiorna timestamp ultima attività MQTT
  lastMqttActivity = millis();
  
  // Log messaggio ricevuto
  serialDebugf("Ricevuto messaggio sul topic: %s", topic);
  serialDebugf("Lunghezza payload: %u bytes", length);
  
  if (DEBUG_ENABLED && length > 0) {
    // Mostra solo i primi 64 bytes se messaggio grande
    printHex(payload, length > 64 ? 64 : length);
    if (length > 64) {
      serialDebugf("...altri %u bytes", length - 64);
    }
  }
  
  // Protezione atomica contro doppi aggiornamenti
  if (updateInProgress) {
    serialDebug("⚠️ IGNORATO - Aggiornamento già in corso");
    return;
  }
  
  // Verifica se è un messaggio di comando aggiornamento
  if (strcmp(topic, OTA_COMMAND_TOPIC) == 0) {
    serialDebug("Rilevato comando di aggiornamento firmware");
    // Impedisci immediatamente ulteriori elaborazioni
    updateInProgress = true;
    // Analizza il comando di aggiornamento
    parseUpdateCommand(payload, length);
  }
}

/**
 * Verifica se un messaggio è un duplicato basandosi sul timestamp
 * @return true se è un duplicato
 */
bool isMessageDuplicate(unsigned long commandTimestamp) {
  // Considera duplicato se abbiamo già elaborato un comando con timestamp >= a questo
  // E il comando è arrivato negli ultimi 30 secondi (per evitare problemi con overflow millis)
  if (commandTimestamp > 0 && lastCommandId > 0 && 
      commandTimestamp <= lastCommandId && 
      (millis() - updateStartTime < 30000)) {
    return true;
  }
  
  // Aggiorna l'ultimo ID comando elaborato
  lastCommandId = commandTimestamp;
  return false;
}

/**
 * Analizza il comando di aggiornamento ricevuto via MQTT
 */
void parseUpdateCommand(byte* payload, unsigned int length) {
  // Anche se abbiamo impostato updateInProgress = true in mqttCallback,
  // controlliamo di nuovo per garantire l'atomicità
  if (updateInProgress && updateStartTime > 0) {
    serialDebug("⚠️ IGNORATO - Aggiornamento già in corso");
    publishDebug("⚠️ IGNORATO - Aggiornamento già in corso");
    // Rilascia il blocco se non stiamo davvero elaborando un aggiornamento
    if (millis() - updateStartTime > 60000) {
      serialDebug("🔓 Reset stato aggiornamento - timeout superato");
      updateInProgress = false;
    }
    return;
  }
  
  // Registra l'inizio dell'elaborazione
  updateStartTime = millis();
  
  CborParser parser;
  CborValue value, mapValue;
  CborError err;
  
  // Inizializzazione parser CBOR
  serialDebug("Inizializzazione parser CBOR...");
  err = cbor_parser_init(payload, length, 0, &parser, &value);
  
  if (err || !cbor_value_is_map(&value)) {
    serialDebugf("Errore: payload CBOR non valido (errore: %d)", err);
    publishDebug("Errore: payload CBOR non valido");
    updateInProgress = false;  // Reset stato aggiornamento
    return;
  }
  
  // Parametri da estrarre
  bool hasTargetDevice = false;
  bool hasUrl = false;
  bool hasHost = false;
  bool hasPath = false;
  int port = 80; // Default port
  updateParams.chunkedTransfer = false;
  updateParams.contentLength = 0;
  unsigned long commandTimestamp = 0;  // Per verificare duplicati
  
  // Reset parametri
  memset(updateParams.targetDevice, 0, sizeof(updateParams.targetDevice));
  memset(updateParams.firmwareUrl, 0, sizeof(updateParams.firmwareUrl));
  memset(updateParams.serverHost, 0, sizeof(updateParams.serverHost));
  memset(updateParams.serverPath, 0, sizeof(updateParams.serverPath));
  
  // Estrai valori dalla mappa CBOR
  serialDebug("Analisi valori CBOR...");
  cbor_value_enter_container(&value, &mapValue);
  
  while (!cbor_value_at_end(&mapValue)) {
    // Leggi chiave
    char keyBuffer[32];
    size_t keyLength = sizeof(keyBuffer);
    
    if (cbor_value_is_text_string(&mapValue)) {
      cbor_value_copy_text_string(&mapValue, keyBuffer, &keyLength, &mapValue);
      serialDebugf("Trovata chiave: %s", keyBuffer);
      
      // Processa i vari campi
      if (strcmp(keyBuffer, "target") == 0 && cbor_value_is_text_string(&mapValue)) {
        size_t valueLength = sizeof(updateParams.targetDevice);
        cbor_value_copy_text_string(&mapValue, updateParams.targetDevice, &valueLength, &mapValue);
        serialDebugf("Target device: %s", updateParams.targetDevice);
        hasTargetDevice = true;
      }
      // Processa URL completo se presente
      else if (strcmp(keyBuffer, "url") == 0 && cbor_value_is_text_string(&mapValue)) {
        size_t valueLength = sizeof(updateParams.firmwareUrl);
        cbor_value_copy_text_string(&mapValue, updateParams.firmwareUrl, &valueLength, &mapValue);
        serialDebugf("URL: %s", updateParams.firmwareUrl);
        hasUrl = true;
      }
      // Processa host se presente (alternativa a url)
      else if (strcmp(keyBuffer, "host") == 0 && cbor_value_is_text_string(&mapValue)) {
        size_t valueLength = sizeof(updateParams.serverHost);
        cbor_value_copy_text_string(&mapValue, updateParams.serverHost, &valueLength, &mapValue);
        serialDebugf("Host: %s", updateParams.serverHost);
        hasHost = true;
      }
      // Processa port se presente
      else if (strcmp(keyBuffer, "port") == 0 && cbor_value_is_integer(&mapValue)) {
        int portValue;
        cbor_value_get_int(&mapValue, &portValue);
        port = portValue;
        serialDebugf("Port: %d", port);
        cbor_value_advance(&mapValue);
      }
      // Processa path se presente
      else if (strcmp(keyBuffer, "path") == 0 && cbor_value_is_text_string(&mapValue)) {
        // Per path lunghe, gestiamo con particolare attenzione
        // Prima controlliamo la lunghezza effettiva
        size_t actualLength;
        cbor_value_calculate_string_length(&mapValue, &actualLength);
        
        serialDebugf("Path rilevata (lunghezza: %u bytes)", actualLength);
        
        if (actualLength >= sizeof(updateParams.serverPath) - 1) {
          serialDebugf("ATTENZIONE: Path è troppo lunga (%u bytes) e verrà troncata", actualLength);
          publishDebug("ATTENZIONE: Path troppo lunga - possibile troncamento");
        }
        
        size_t valueLength = sizeof(updateParams.serverPath) - 1;
        cbor_value_copy_text_string(&mapValue, updateParams.serverPath, &valueLength, &mapValue);
        updateParams.serverPath[valueLength] = '\0';  // Assicura terminazione stringa
        
        serialDebugf("Path: '%s'", updateParams.serverPath);
        hasPath = true;
      }
      // Processa content_length se presente
      else if (strcmp(keyBuffer, "content_length") == 0 && cbor_value_is_integer(&mapValue)) {
        cbor_value_get_int(&mapValue, (int*)&updateParams.contentLength);
        serialDebugf("Content Length: %ld bytes", updateParams.contentLength);
        cbor_value_advance(&mapValue);
      }
      // Processa chunked se presente
      else if (strcmp(keyBuffer, "chunked") == 0 && cbor_value_is_boolean(&mapValue)) {
        bool chunked;
        cbor_value_get_boolean(&mapValue, &chunked);
        updateParams.chunkedTransfer = chunked;
        serialDebugf("Chunked Transfer: %s", chunked ? "true" : "false");
        cbor_value_advance(&mapValue);
      }
      // Processa timestamp se presente (per deduplicazione)
      else if (strcmp(keyBuffer, "timestamp") == 0 && cbor_value_is_integer(&mapValue)) {
        cbor_value_get_int(&mapValue, (int*)&commandTimestamp);
        serialDebugf("Command Timestamp: %lu", commandTimestamp);
        cbor_value_advance(&mapValue);
      }
      else {
        // Campo non supportato, salta
        serialDebugf("Campo ignorato: %s", keyBuffer);
        cbor_value_advance(&mapValue);
      }
    }
    else {
      // Formato non valido, salta
      serialDebug("Formato chiave non valido, salto...");
      cbor_value_advance(&mapValue);
      if (cbor_value_is_valid(&mapValue)) {
        cbor_value_advance(&mapValue);
      }
    }
  }
  
  // Controlla se questo è un comando duplicato (per messaggi con timestamp)
  if (commandTimestamp > 0 && isMessageDuplicate(commandTimestamp)) {
    serialDebug("⚠️ IGNORATO - Comando duplicato rilevato");
    publishDebug("⚠️ IGNORATO - Comando duplicato rilevato");
    updateInProgress = false;
    return;
  }
  
  // Gestione migliorata della path - assicuriamoci che non venga troncata
  if (hasPath) {
    // Verifica se la path è troppo lunga e mostra avviso
    if (strlen(updateParams.serverPath) >= sizeof(updateParams.serverPath) - 1) {
      serialDebug("ATTENZIONE: Path potrebbe essere stata troncata");
      publishDebug("ATTENZIONE: Path potrebbe essere stata troncata");
    }
    
    // Verifica che la path inizi con '/'
    if (updateParams.serverPath[0] != '/') {
      // Prependi '/' alla path
      char tempPath[sizeof(updateParams.serverPath)];
      strcpy(tempPath, updateParams.serverPath);
      strcpy(updateParams.serverPath, "/");
      strncat(updateParams.serverPath, tempPath, sizeof(updateParams.serverPath) - 2);
      serialDebugf("Path corretta con slash iniziale: %s", updateParams.serverPath);
    }
  }
  
  // Se abbiamo host e path ma non URL, costruisci URL dai componenti
  if (hasHost && hasPath && !hasUrl) {
    updateParams.serverPort = port;
    constructUrl(updateParams.serverHost, port, updateParams.serverPath, 
                 updateParams.firmwareUrl, sizeof(updateParams.firmwareUrl));
    serialDebugf("URL costruito: %s", updateParams.firmwareUrl);
    hasUrl = true;
  }
  
  // Se abbiamo host ma path vuoto, imposta path di default
  if (hasHost && !hasPath) {
    strcpy(updateParams.serverPath, "/");
    hasPath = true;
    updateParams.serverPort = port;
    constructUrl(updateParams.serverHost, port, updateParams.serverPath, 
                 updateParams.firmwareUrl, sizeof(updateParams.firmwareUrl));
    serialDebugf("URL costruito con path di default: %s", updateParams.firmwareUrl);
    hasUrl = true;
  }
  
  // Valida i parametri obbligatori
  if (!hasTargetDevice || !hasUrl) {
    serialDebug("Errore: parametri obbligatori mancanti nel comando di aggiornamento");
    publishDebug("Errore: parametri obbligatori mancanti nel comando di aggiornamento");
    if (!hasTargetDevice) serialDebug("Manca target device");
    if (!hasUrl && !(hasHost && hasPath)) serialDebug("Manca URL o host+path");
    updateInProgress = false;  // Reset stato aggiornamento
    return;
  }
  
  // Verifica se questo dispositivo è il target del comando
  serialDebugf("Confronto target '%s' con ID locale '%s'", updateParams.targetDevice, DEVICE_ID);
  if (strcmp(updateParams.targetDevice, DEVICE_ID) != 0) {
    // Questo messaggio è per un altro dispositivo, ignora silenziosamente
    serialDebug("Comando destinato ad altro dispositivo, ignorato");
    updateInProgress = false;  // Reset stato aggiornamento
    return;
  }
  
  // Verifica se l'URL è ben formattato e estrae host, porta e path per HTTP
  serialDebug("Parsing componenti URL per HTTP...");
  if (!parseUrl(updateParams.firmwareUrl, updateParams.serverHost, &updateParams.serverPort, updateParams.serverPath)) {
    serialDebug("Errore: URL firmware non valido");
    publishDebug("Errore: URL firmware non valido");
    publishResponse("error", "URL firmware non valido");
    updateInProgress = false;  // Reset stato aggiornamento
    return;
  }
  
  // Avvia processo di aggiornamento
  serialDebug("Comando di aggiornamento ricevuto per questo dispositivo");
  publishDebug("Comando di aggiornamento ricevuto per questo dispositivo");
  serialDebugf("URL: %s", updateParams.firmwareUrl);
  publishDebugf("URL: %s", updateParams.firmwareUrl);
  
  if (updateParams.contentLength > 0) {
    serialDebugf("Dimensione prevista: %ld bytes", updateParams.contentLength);
    publishDebugf("Dimensione prevista: %ld bytes", updateParams.contentLength);
  } else {
    serialDebug("Dimensione non specificata, sarà determinata durante il download");
    publishDebug("Dimensione non specificata, sarà determinata durante il download");
  }
  
  // Avvia il processo di download e aggiornamento
  // updateInProgress è già true (impostato sopra)
  
  if (downloadAndUpdate()) {
    // L'aggiornamento è stato completato con successo
    // Nota: in realtà il dispositivo si riavvierà prima di arrivare qui
    serialDebug("Aggiornamento completato con successo");
    publishDebug("Aggiornamento completato con successo");
    publishResponse("success", "Aggiornamento completato");
    updateInProgress = false;  // Reset stato aggiornamento (in caso non riavvii)
  } else {
    // L'aggiornamento è fallito
    serialDebug("Aggiornamento fallito");
    publishDebug("Aggiornamento fallito");
    publishResponse("error", "Aggiornamento fallito");
    updateInProgress = false;  // Reset stato aggiornamento
  }
}

/**
 * Costruisce un URL completo dai suoi componenti
 */
void constructUrl(const char* host, int port, const char* path, char* url, size_t urlSize) {
  if (port == 80) {
    // URL standard senza porta esplicita
    snprintf(url, urlSize, "http://%s%s", host, path);
  } else {
    // URL con porta esplicita
    snprintf(url, urlSize, "http://%s:%d%s", host, port, path);
  }
}

/**
 * Estrae host, porta e path da un URL completo
 */
bool parseUrl(const char* url, char* host, int* port, char* path) {
  serialDebugf("Parsing URL: %s", url);
  
  // URL deve iniziare con http://
  if (strncmp(url, "http://", 7) != 0) {
    serialDebug("URL non inizia con http://");
    return false;
  }
  
  // Punta dopo il protocollo
  const char* ptr = url + 7;
  
  // Estrai host
  const char* hostEnd = strchr(ptr, ':');
  if (hostEnd == NULL) {
    // Nessuna porta specificata
    hostEnd = strchr(ptr, '/');
    if (hostEnd == NULL) {
      // Nessun path specificato
      strncpy(host, ptr, sizeof(updateParams.serverHost)-1);
      host[sizeof(updateParams.serverHost)-1] = '\0';
      *port = 80; // Porta di default
      strcpy(path, "/");
      serialDebugf("Host: %s, Port: 80, Path: /", host);
      return true;
    }
  }
  
  // Copia l'host
  size_t hostLen = hostEnd - ptr;
  if (hostLen >= sizeof(updateParams.serverHost)) {
    hostLen = sizeof(updateParams.serverHost) - 1;
  }
  strncpy(host, ptr, hostLen);
  host[hostLen] = '\0';
  
  // Estrai porta
  if (*hostEnd == ':') {
    ptr = hostEnd + 1;
    *port = atoi(ptr);
    
    // Trova inizio path
    const char* pathStart = strchr(ptr, '/');
    if (pathStart == NULL) {
      // Nessun path
      strcpy(path, "/");
      serialDebugf("Host: %s, Port: %d, Path: /", host, *port);
      return true;
    }
    
    // Copia path
    strncpy(path, pathStart, sizeof(updateParams.serverPath)-1);
    path[sizeof(updateParams.serverPath)-1] = '\0';
    serialDebugf("Host: %s, Port: %d, Path: %s", host, *port, path);
  } else {
    // Porta default
    *port = 80;
    
    // Copia path
    strncpy(path, hostEnd, sizeof(updateParams.serverPath)-1);
    path[sizeof(updateParams.serverPath)-1] = '\0';
    serialDebugf("Host: %s, Port: 80, Path: %s", host, path);
  }
  
  return true;
}

/**
 * Scarica e aggiorna il firmware
 * @return true se completato con successo, false in caso di errore
 */
bool downloadAndUpdate() {
  serialDebug("Avvio download e aggiornamento firmware");
  publishDebug("Avvio download e aggiornamento firmware");
  
  // Connessione al server
  serialDebugf("Connessione a %s:%d", updateParams.serverHost, updateParams.serverPort);
  publishDebugf("Connessione a %s:%d", updateParams.serverHost, updateParams.serverPort);
  
  if (!otaClient.connect(updateParams.serverHost, updateParams.serverPort)) {
    serialDebug("Errore: connessione al server fallita");
    publishDebug("Errore: connessione al server fallita");
    return false;
  }
  
  // Invio richiesta HTTP GET
  serialDebugf("Richiesta file: %s", updateParams.serverPath);
  publishDebugf("Richiesta file: %s", updateParams.serverPath);
  
  // Verifica che la path inizi con '/' per sicurezza
  char safePath[384];
  if (updateParams.serverPath[0] != '/') {
    safePath[0] = '/';
    strncpy(safePath + 1, updateParams.serverPath, sizeof(safePath) - 2);
    safePath[sizeof(safePath) - 1] = '\0';
    serialDebugf("Path corretta con slash iniziale: %s", safePath);
  } else {
    strncpy(safePath, updateParams.serverPath, sizeof(safePath) - 1);
    safePath[sizeof(safePath) - 1] = '\0';
  }
  
  // Costruzione richiesta HTTP dettagliata - usa safePath invece di updateParams.serverPath
  String httpRequest = "GET " + String(safePath) + " HTTP/1.1\r\n";
  httpRequest += "Host: " + String(updateParams.serverHost) + "\r\n";
  httpRequest += "Connection: close\r\n";
  httpRequest += "User-Agent: ArduinoOTA/2.1\r\n";
  httpRequest += "\r\n";
  
  serialDebugf("Invio richiesta HTTP:\n%s", httpRequest.c_str());
  
  // Debug aggiuntivo per aiutare la diagnosi
  serialDebugf("URL completo: http://%s:%d%s", 
              updateParams.serverHost, 
              updateParams.serverPort, 
              safePath);
  publishDebugf("URL completo: http://%s:%d%s", 
               updateParams.serverHost, 
               updateParams.serverPort, 
               safePath);
  
  // Invia la richiesta HTTP
  otaClient.print(httpRequest);
  
  // Attesa risposta dal server con timeout
  serialDebug("Attesa risposta dal server...");
  unsigned long timeout = millis();
  while (otaClient.available() == 0) {
    if (millis() - timeout > 10000) {  // Timeout 10 secondi
      serialDebug("Errore: timeout connessione");
      publishDebug("Errore: timeout connessione");
      otaClient.stop();
      return false;
    }
    delay(100);
  }
  
  // Parsing header HTTP
  serialDebug("Risposta dal server ricevuta, analisi headers HTTP...");
  String line;
  int statusCode = 0;
  long contentLength = updateParams.contentLength;
  bool chunkedTransfer = updateParams.chunkedTransfer;
  
  // Analisi degli header della risposta HTTP
  while (otaClient.connected()) {
    line = otaClient.readStringUntil('\n');
    line.trim();
    
    serialDebugf("Header: %s", line.c_str());
    
    // Estrazione codice stato
    if (line.startsWith("HTTP/")) {
      statusCode = line.substring(9, 12).toInt();
      serialDebugf("Codice stato: %d", statusCode);
      publishDebugf("Codice stato: %d", statusCode);
      
      // Aggiungi informazioni aggiuntive in caso di errori comuni
      if (statusCode == 404) {
        serialDebug("ERRORE: File non trovato (404). Verificare path e URL");
        publishDebug("ERRORE: File non trovato (404). Verificare path e URL");
      } else if (statusCode == 403) {
        serialDebug("ERRORE: Accesso negato (403). Verificare permessi");
        publishDebug("ERRORE: Accesso negato (403). Verificare permessi");
      } else if (statusCode >= 500) {
        serialDebug("ERRORE: Errore server. Riprova più tardi");
        publishDebug("ERRORE: Errore server. Riprova più tardi");
      }
    } 
    // Estrazione Content-Length se non già specificato
    else if (contentLength <= 0 && line.startsWith("Content-Length:")) {
      contentLength = line.substring(15).toInt();
      serialDebugf("Dimensione file (da HTTP): %ld bytes", contentLength);
      publishDebugf("Dimensione file (da HTTP): %ld bytes", contentLength);
    } 
    // Rilevamento trasferimento chunked
    else if (line.startsWith("Transfer-Encoding: chunked")) {
      chunkedTransfer = true;
      serialDebug("Trasferimento chunked rilevato");
      publishDebug("Trasferimento chunked rilevato");
    }
    
    // Linea vuota indica fine degli header
    if (line.length() == 0) {
      serialDebug("Fine headers HTTP");
      break;
    }
  }
  
  // Controllo risposta HTTP valida
  if (statusCode != 200) {
    serialDebugf("Errore: risposta HTTP %d", statusCode);
    publishDebugf("Errore: risposta HTTP %d", statusCode);
    otaClient.stop();
    
    // Mostra contenuto della risposta di errore per diagnosi
    if (otaClient.available()) {
      serialDebug("Contenuto risposta errore:");
      String errorContent = "";
      while (otaClient.available() && errorContent.length() < 1024) {
        errorContent += (char)otaClient.read();
      }
      serialDebug(errorContent.c_str());
    }
    
    return false;
  }
  
  // Reset CRC
  crc.reset();
  
  // Per trasferimenti chunked, determinazione dimensione da allocare se non specificata
  if (chunkedTransfer && contentLength <= 0) {
    // Determina lo spazio massimo disponibile per il firmware
    size_t maxFirmwareSize = getMaxFirmwareSize();
    contentLength = maxFirmwareSize;
    serialDebugf("Trasferimento chunked, allocazione di %ld KB", contentLength / 1024);
    publishDebugf("Trasferimento chunked, allocazione di %ld KB", contentLength / 1024);
  }
  
  // Verifica dimensione valida
  if (contentLength <= 0) {
    serialDebug("Errore: dimensione file sconosciuta");
    publishDebug("Errore: dimensione file sconosciuta");
    otaClient.stop();
    return false;
  }
  
  // Verifica che dimensione firmware non superi un minimo ragionevole (500 bytes)
  if (contentLength < 500) {
    serialDebugf("Errore: dimensione firmware troppo piccola (%ld bytes)", contentLength);
    publishDebugf("Errore: dimensione firmware troppo piccola (%ld bytes)", contentLength);
    otaClient.stop();
    return false;
  }
  
  // Apertura storage per il firmware
  serialDebugf("Allocazione spazio: %ld bytes", contentLength);
  publishDebugf("Allocazione spazio: %ld bytes", contentLength);
  
  if (!startFirmwareUpdate(contentLength)) {
    serialDebug("Errore: impossibile inizializzare l'aggiornamento");
    publishDebug("Errore: impossibile inizializzare l'aggiornamento");
    otaClient.stop();
    return false;
  }
  
  // Download e scrittura del firmware
  serialDebug("Download in corso...");
  publishDebug("Download in corso...");
  
  unsigned long bytesWritten = 0;
  unsigned long lastProgressReport = 0;
  unsigned long downloadStart = millis();
  
  // Download con trasferimento chunked
  if (chunkedTransfer) {
    serialDebug("Avvio download in modalità chunked");
    // Elaborazione trasferimento chunked
    while (otaClient.connected()) {
      // Leggi dimensione chunk
      String chunkSizeStr = otaClient.readStringUntil('\r');
      otaClient.read();  // Consuma \n
      
      // Converti da hex a int
      int chunkSize = strtol(chunkSizeStr.c_str(), NULL, 16);
      serialDebugf("Chunk size: %d bytes", chunkSize);
      
      // Chunk size 0 indica fine trasferimento
      if (chunkSize == 0) {
        serialDebug("Fine trasferimento chunked");
        publishDebug("Fine trasferimento chunked");
        otaClient.readStringUntil('\r');
        otaClient.read();  // Consuma \n
        break;
      }
      
      // Download e processing del chunk
      int bytesRead = 0;
      while (bytesRead < chunkSize && otaClient.connected()) {
        // Attesa dati con timeout
        if (!otaClient.available()) {
          if (millis() - downloadStart > DOWNLOAD_TIMEOUT) {
            serialDebug("Errore: timeout download");
            publishDebug("Errore: timeout download");
            abortFirmwareUpdate();
            otaClient.stop();
            return false;
          }
          delay(1);
          continue;
        }
        
        // Calcolo dimensione buffer da leggere
        int bytesAvailable = otaClient.available();
        int remainingBytes = chunkSize - bytesRead;
        int toRead = BUFFER_SIZE;
        
        // Limitazione dimensione lettura
        if (remainingBytes < toRead) toRead = remainingBytes;
        if (bytesAvailable < toRead) toRead = bytesAvailable;
        
        // Lettura dati in buffer
        int actualRead = otaClient.read(downloadBuffer, toRead);
        
        if (actualRead > 0) {
          // Aggiornamento CRC
          crc.update(downloadBuffer, actualRead);
          
          // Scrittura dati nel firmware storage
          #ifdef ESP32
          if (Update.write(downloadBuffer, actualRead) != actualRead) {
            serialDebugf("Errore scrittura update: %s", Update.errorString());
            publishDebugf("Errore scrittura update: %s", Update.errorString());
            abortFirmwareUpdate();
            otaClient.stop();
            return false;
          }
          #else
          for (int i = 0; i < actualRead; i++) {
            InternalStorage.write(downloadBuffer[i]);
          }
          #endif
          
          bytesRead += actualRead;
          bytesWritten += actualRead;
          
          // Aggiornamento progresso ogni 4KB
          if (bytesWritten - lastProgressReport >= 4096) {
            lastProgressReport = bytesWritten;
            int percentage = (contentLength > 0) ? (bytesWritten * 100) / contentLength : 0;
            serialDebugf("Download: %d%% (%lu bytes)", percentage, bytesWritten);
            publishProgress(bytesWritten, contentLength);
          }
        }
      }
      
      // Lettura terminatore chunk (CRLF)
      otaClient.readStringUntil('\r');
      otaClient.read();  // Consuma \n
    }
  } else {
    serialDebug("Avvio download standard (non chunked)");
    // Download normale con Content-Length
    while (bytesWritten < contentLength && otaClient.connected()) {
      // Attesa dati con timeout
      if (!otaClient.available()) {
        if (millis() - downloadStart > DOWNLOAD_TIMEOUT) {
          serialDebug("Errore: timeout download");
          publishDebug("Errore: timeout download");
          abortFirmwareUpdate();
          otaClient.stop();
          return false;
        }
        delay(1);
        continue;
      }
      
      // Calcolo dimensione buffer da leggere
      int bytesAvailable = otaClient.available();
      long remainingBytes = contentLength - bytesWritten;
      int toRead = BUFFER_SIZE;
      
      // Limitazione dimensione lettura
      if (remainingBytes < toRead) toRead = (int)remainingBytes;
      if (bytesAvailable < toRead) toRead = bytesAvailable;
      
      // Lettura dati in buffer
      int actualRead = otaClient.read(downloadBuffer, toRead);
      
      if (actualRead > 0) {
        // Aggiornamento CRC
        crc.update(downloadBuffer, actualRead);
        
        // Scrittura dati nel firmware storage
        #ifdef ESP32
        if (Update.write(downloadBuffer, actualRead) != actualRead) {
          serialDebugf("Errore scrittura update: %s", Update.errorString());
          publishDebugf("Errore scrittura update: %s", Update.errorString());
          abortFirmwareUpdate();
          otaClient.stop();
          return false;
        }
        #else
        for (int i = 0; i < actualRead; i++) {
          InternalStorage.write(downloadBuffer[i]);
        }
        #endif
        
        bytesWritten += actualRead;
        
        // Aggiornamento progresso ogni 4KB
        if (bytesWritten - lastProgressReport >= 4096) {
          lastProgressReport = bytesWritten;
          int percentage = (contentLength > 0) ? (bytesWritten * 100) / contentLength : 0;
          serialDebugf("Download: %d%% (%lu bytes)", percentage, bytesWritten);
          publishProgress(bytesWritten, contentLength);
        }
      }
    }
  }
  
  // Calcolo CRC finale
  uint32_t firmwareCRC = crc.finalize();
  
  // Chiusura connessione
  otaClient.stop();
  
  // Verifica completezza download
  if ((chunkedTransfer && bytesWritten > 0) || (bytesWritten == contentLength)) {
    serialDebugf("Download completato: %lu bytes (CRC32: %08lX)", bytesWritten, firmwareCRC);
    publishDebugf("Download completato: %lu bytes (CRC32: %08lX)", bytesWritten, firmwareCRC);
    
    // Verifica minima dimensione firmware
    if (bytesWritten < 500) {
      serialDebug("Errore: dimensione firmware scaricato troppo piccola");
      publishDebug("Errore: dimensione firmware scaricato troppo piccola");
      abortFirmwareUpdate();
      return false;
    }
    
    // Finalizzazione update e riavvio
    serialDebug("Applicazione aggiornamento e riavvio...");
    publishDebug("Applicazione aggiornamento e riavvio...");
    
    if (finishFirmwareUpdate()) {
      // Questa linea non dovrebbe mai essere eseguita se l'aggiornamento ha successo
      // perché il dispositivo si riavvierà prima
      return true;
    } else {
      serialDebug("Errore: applicazione aggiornamento fallita");
      publishDebug("Errore: applicazione aggiornamento fallita");
      return false;
    }
  } else {
    // Download incompleto
    serialDebugf("Errore: download incompleto. Ricevuti %lu di %ld bytes", bytesWritten, contentLength);
    publishDebugf("Errore: download incompleto. Ricevuti %lu di %ld bytes", bytesWritten, contentLength);
    abortFirmwareUpdate();
    return false;
  }
}

/**
 * Inizializza l'aggiornamento del firmware
 * @param contentLength Dimensione del firmware da scaricare
 * @return true se inizializzato correttamente, false in caso di errore
 */
bool startFirmwareUpdate(size_t contentLength) {
  #ifdef ESP32
    // Per ESP32, verifica spazio nella partizione
    const esp_partition_t* updatePartition = esp_ota_get_next_update_partition(NULL);
    if (updatePartition == NULL) {
      serialDebug("Errore: impossibile trovare partizione per update");
      publishDebug("Errore: impossibile trovare partizione per update");
      return false;
    }
    
    if (contentLength > updatePartition->size) {
      serialDebugf("Avviso: richiesta dimensione %u > partizione %u, limitando...", 
                 contentLength, updatePartition->size);
      publishDebugf("Avviso: richiesta dimensione %u > partizione %u, limitando...", 
                  contentLength, updatePartition->size);
      contentLength = updatePartition->size - 4 * 1024;  // 4KB margine
    }
    
    // Inizia update ESP32
    if (!Update.begin(contentLength)) {
      serialDebugf("Errore: impossibile allocare memoria per l'update: %s", Update.errorString());
      publishDebugf("Errore: impossibile allocare memoria per l'update: %s", Update.errorString());
      return false;
    }
    
    return true;
  #else
    // Verifica se la dimensione richiesta eccede il massimo
    size_t maxFirmwareSize = getMaxFirmwareSize();
    if (contentLength > maxFirmwareSize) {
      serialDebugf("Avviso: richiesta dimensione %u > max %u, limitando...", 
                 contentLength, maxFirmwareSize);
      publishDebugf("Avviso: richiesta dimensione %u > max %u, limitando...", 
                 contentLength, maxFirmwareSize);
      contentLength = maxFirmwareSize;
    }
    
    // Apertura storage interno per firmware
    if (!InternalStorage.open(contentLength)) {
      serialDebug("Errore: spazio insufficiente per l'aggiornamento");
      publishDebug("Errore: spazio insufficiente per l'aggiornamento");
      return false;
    }
    
    return true;
  #endif
}

/**
 * Finalizza l'aggiornamento del firmware e riavvia
 * @return true se completato con successo (in realtà non dovrebbe mai ritornare)
 */
bool finishFirmwareUpdate() {
  #ifdef ESP32
    if (Update.end(true)) {
      serialDebug("Update completato correttamente!");
      publishDebug("Update completato correttamente!");
      serialDebug("Riavvio in corso...");
      publishDebug("Riavvio in corso...");
      
      // Pulisci e disconnetti tutti i sistemi
      Serial.flush();
      mqttClient.disconnect();
      otaClient.stop();
      
      // Blink LED per indicare riavvio
      blinkLED(5, 100);
      
      delay(1000);
      ESP.restart();
      return true; // Non dovrebbe mai arrivare qui
    } else {
      serialDebugf("Errore update: %s", Update.errorString());
      publishDebugf("Errore update: %s", Update.errorString());
      return false;
    }
  #else
    // Chiudi lo storage interno
    InternalStorage.close();
    
    // Notifica l'aggiornamento imminente
    serialDebug("Applicazione aggiornamento e riavvio...");
    publishDebug("Applicazione aggiornamento e riavvio...");
    
    // Pulisci e disconnetti tutti i sistemi prima di riavviare
    otaClient.stop();
    
    // Assicurati che tutti i messaggi di log vengano inviati prima del riavvio
    Serial.flush();
    delay(200);
    
    // Disconnetti MQTT in modo pulito e attendi che venga completato
    mqttClient.disconnect();
    delay(500);
    
    // Segnalazione visiva prima del riavvio
    blinkLED(5, 100);
    
    // Ultima attesa prima di applicare l'aggiornamento
    delay(500);
    
    // Disattiva interruzioni per evitare interferenze durante l'applicazione
    noInterrupts();
    
    // Questa funzione non dovrebbe ritornare se l'aggiornamento ha successo
    // ma applica l'aggiornamento e riavvia il dispositivo
    InternalStorage.apply();
    
    // Se arriviamo qui, c'è stato un errore - riattiva interruzioni
    interrupts();
    
    // Qui solo in caso di errore
    serialDebug("Errore: applicazione aggiornamento fallita");
    publishDebug("Errore: applicazione aggiornamento fallita");
    return false;
  #endif
}

/**
 * Annulla l'aggiornamento del firmware in corso
 */
void abortFirmwareUpdate() {
  serialDebug("Annullamento aggiornamento firmware");
  #ifdef ESP32
    Update.abort();
  #else
    InternalStorage.close();
  #endif
}

/**
 * Determina lo spazio massimo disponibile per il firmware
 * @return Dimensione massima in bytes
 */
size_t getMaxFirmwareSize() {
  #ifdef ESP32
    const esp_partition_t* updatePartition = esp_ota_get_next_update_partition(NULL);
    if (updatePartition != NULL) {
      size_t size = updatePartition->size - 4 * 1024; // Dimensione partizione - 4KB
      serialDebugf("Spazio disponibile per firmware (ESP32): %u KB", size / 1024);
      return size;
    } else {
      serialDebug("Errore: impossibile determinare partizione update, usando 1MB default");
      return 1024 * 1024; // 1MB default
    }
  #else
    size_t size = 120 * 1024; // 120KB (valore conservativo per UNO R4 WiFi)
    serialDebugf("Spazio disponibile per firmware (UNO R4): %u KB", size / 1024);
    return size;
  #endif
}

/**
 * Pubblica messaggio di ping con encoding CBOR
 */
void publishPing() {
  // Crea messaggio CBOR per il ping
  CborEncoder encoder, mapEncoder;
  cbor_encoder_init(&encoder, cborBuffer, sizeof(cborBuffer), 0);
  
  // Crea mappa con elementi
  cbor_encoder_create_map(&encoder, &mapEncoder, 3);
  
  // Device ID
  cbor_encode_text_stringz(&mapEncoder, "device");
  cbor_encode_text_stringz(&mapEncoder, DEVICE_ID);
  
  // Timestamp
  cbor_encode_text_stringz(&mapEncoder, "timestamp");
  cbor_encode_uint(&mapEncoder, millis());
  
  // Type
  cbor_encode_text_stringz(&mapEncoder, "type");
  cbor_encode_text_stringz(&mapEncoder, "ping");
  
  cbor_encoder_close_container(&encoder, &mapEncoder);
  
  // Pubblica messaggio CBOR
  size_t cborLen = cbor_encoder_get_buffer_size(&encoder, cborBuffer);
  String pingTopic = String(PING_TOPIC) + DEVICE_ID;
  mqttClient.publish(pingTopic.c_str(), cborBuffer, cborLen);
  
  serialDebug("Inviato ping di keep-alive (CBOR)");
}

/**
 * Pubblica un messaggio di debug su MQTT con encoding CBOR
 */
void publishDebug(const char* message) {
  // Crea messaggio CBOR con il debug
  CborEncoder encoder, mapEncoder;
  cbor_encoder_init(&encoder, cborBuffer, sizeof(cborBuffer), 0);
  
  // Crea mappa con elementi
  cbor_encoder_create_map(&encoder, &mapEncoder, 3);
  
  // Device ID
  cbor_encode_text_stringz(&mapEncoder, "device");
  cbor_encode_text_stringz(&mapEncoder, DEVICE_ID);
  
  // Timestamp
  cbor_encode_text_stringz(&mapEncoder, "timestamp");
  cbor_encode_uint(&mapEncoder, millis());
  
  // Message
  cbor_encode_text_stringz(&mapEncoder, "message");
  cbor_encode_text_stringz(&mapEncoder, message);
  
  cbor_encoder_close_container(&encoder, &mapEncoder);
  
  // Pubblica messaggio CBOR
  size_t cborLen = cbor_encoder_get_buffer_size(&encoder, cborBuffer);
  String debugTopic = String(DEBUG_TOPIC) + DEVICE_ID;
  mqttClient.publish(debugTopic.c_str(), cborBuffer, cborLen);
  
  // Echo su seriale per debug locale
  serialDebug(message);
}

/**
 * Pubblica un messaggio di debug formattato su MQTT
 */
void publishDebugf(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  publishDebug(buffer);
}

/**
 * Pubblica lo stato di avanzamento del download
 */
void publishProgress(unsigned long current, unsigned long total) {
  // Crea messaggio CBOR con lo stato di avanzamento
  CborEncoder encoder, mapEncoder;
  cbor_encoder_init(&encoder, cborBuffer, sizeof(cborBuffer), 0);
  
  // Crea mappa con elementi
  cbor_encoder_create_map(&encoder, &mapEncoder, 4);
  
  // Device ID
  cbor_encode_text_stringz(&mapEncoder, "device");
  cbor_encode_text_stringz(&mapEncoder, DEVICE_ID);
  
  // Bytes correnti
  cbor_encode_text_stringz(&mapEncoder, "current");
  cbor_encode_uint(&mapEncoder, current);
  
  // Bytes totali
  cbor_encode_text_stringz(&mapEncoder, "total");
  cbor_encode_uint(&mapEncoder, total);
  
  // Percentuale
  int percentage = (total > 0) ? (current * 100) / total : 0;
  cbor_encode_text_stringz(&mapEncoder, "percentage");
  cbor_encode_uint(&mapEncoder, percentage);
  
  cbor_encoder_close_container(&encoder, &mapEncoder);
  
  // Pubblica messaggio CBOR
  size_t cborLen = cbor_encoder_get_buffer_size(&encoder, cborBuffer);
  mqttClient.publish(OTA_RESPONSE_TOPIC, cborBuffer, cborLen);
  
  // Echo su seriale
  serialDebugf("Progresso: %d%% (%lu di %lu bytes)", 
             percentage, current, total);
}

/**
 * Pubblica una risposta di stato
 */
void publishResponse(const char* status, const char* message) {
  // Crea messaggio CBOR con risposta
  CborEncoder encoder, mapEncoder;
  cbor_encoder_init(&encoder, cborBuffer, sizeof(cborBuffer), 0);
  
  // Crea mappa con elementi
  cbor_encoder_create_map(&encoder, &mapEncoder, 3);
  
  // Device ID
  cbor_encode_text_stringz(&mapEncoder, "device");
  cbor_encode_text_stringz(&mapEncoder, DEVICE_ID);
  
  // Status
  cbor_encode_text_stringz(&mapEncoder, "status");
  cbor_encode_text_stringz(&mapEncoder, status);
  
  // Message
  cbor_encode_text_stringz(&mapEncoder, "message");
  cbor_encode_text_stringz(&mapEncoder, message);
  
  cbor_encoder_close_container(&encoder, &mapEncoder);
  
  // Pubblica messaggio CBOR
  size_t cborLen = cbor_encoder_get_buffer_size(&encoder, cborBuffer);
  mqttClient.publish(OTA_RESPONSE_TOPIC, cborBuffer, cborLen);
  
  // Echo su seriale
  serialDebugf("Risposta: %s - %s", status, message);
}

/**
 * Funzione per far lampeggiare il LED come segnalazione visiva
 * @param times Numero di lampeggi
 * @param delayMs Durata di ogni stato (on/off)
 */
void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

/**
 * Funzione per ottenere una stima della memoria libera
 * Nota: l'implementazione varia a seconda della piattaforma
 * @return Stima della memoria RAM libera in bytes
 */
int getFreeMemory() {
  #ifdef ESP32
    return ESP.getFreeHeap();
  #else
    // Per Arduino, implementazione semplificata
    // Attenzione: questa è una stima approssimativa per UNO R4 WiFi
    // extern int __heap_start, *__brkval;
    // int v;
    return 0; //(int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  #endif
}