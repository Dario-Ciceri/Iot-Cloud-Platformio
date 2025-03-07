/*********************************************************************
 * OTA Firmware Updater per Arduino UNO R4 WiFi e ESP32
 * 
 * DESCRIZIONE:
 * Questo firmware implementa un sistema di aggiornamento Over-The-Air (OTA)
 * che scarica aggiornamenti firmware tramite HTTP e li applica al dispositivo.
 * È compatibile sia con Arduino UNO R4 WiFi che con schede ESP32, adattando
 * automaticamente il comportamento in base alla piattaforma rilevata.
 * 
 * CARATTERISTICHE PRINCIPALI:
 * - Supporto multi-piattaforma (UNO R4 WiFi e ESP32)
 * - Download HTTP con supporto per trasferimenti normali e chunked
 * - Visualizzazione percentuale avanzamento download
 * - Allocazione dinamica della memoria in base alla piattaforma
 * - Determinazione automatica dello spazio disponibile per firmware
 * - Verifica del CRC32 del firmware scaricato
 * - Debugging avanzato con messaggi informativi dettagliati
 * 
 * DIPENDENZE:
 * - CRC32 library
 * - ArduinoOTA/InternalStorage (per UNO R4 WiFi)
 * - WiFiS3 (per UNO R4 WiFi) o WiFi (per ESP32)
 * - Update e librerie OTA di ESP-IDF (per ESP32)
 * 
 * UTILIZZO:
 * 1. Configurare le impostazioni WiFi (SSID/password)
 * 2. Configurare URL e parametri del server di aggiornamento
 * 3. Caricare questo firmware sulla scheda
 * 4. Il dispositivo verificherà periodicamente la disponibilità di aggiornamenti
 * 
 * PROCEDURA DI AGGIORNAMENTO:
 * 1. Connessione al server HTTP
 * 2. Download del firmware
 * 3. Verifica CRC32
 * 4. Installazione e riavvio
 * 
 * Autore: Dario Ciceri
 *         Basato su OTA Firmware Updater originale, migliorato con
 *         debugging avanzato e supporto multi-piattaforma
 * 
 * Versione: 1.1
 * Data: 06 Marzo 2025
 *********************************************************************/

// Inclusione librerie specifiche per piattaforma
#if defined(ARDUINO_UNO_R4_WIFI)
  #include <WiFiS3.h>                 // Libreria WiFi per Arduino UNO R4 WiFi
  #define NO_OTA_NETWORK              // Disabilita la rete OTA (usiamo solo InternalStorage)
  #include <ArduinoOTA.h>             // Per accedere a InternalStorage
#elif defined(ESP32_BOARD)
  #include <WiFi.h>                   // Libreria WiFi per ESP32
  #include <Update.h>                 // Per aggiornamenti firmware
  #include <esp_ota_ops.h>            // API per operazioni OTA di ESP32
  #include <esp_partition.h>          // API per gestione partizioni ESP32
#else
  #error "Scheda non supportata! Usa Arduino UNO R4 WiFi o ESP32."
#endif

#include <CRC32.h>                    // Per calcolare CRC32 del firmware

// Definizione versione attuale del firmware
const short VERSION = 1.1;

/*** CONFIGURAZIONI ***/

// Configurazione WiFi
const char* WIFI_SSID = "test";
const char* WIFI_PASS = "12345678";

// Configurazione server di aggiornamento
const char* SERVER = "192.168.1.25";
const int SERVER_PORT = 8082;
const char* PATH = "/s/s4QoUxkV";

// Configurazione OTA
const unsigned long CHECK_INTERVAL = 30000;  // Intervallo tra controlli aggiornamenti (30 secondi)
const int DOWNLOAD_TIMEOUT = 60000;          // Timeout download (60 secondi)
const int BUFFER_SIZE = 1024;                // Dimensione buffer lettura (1KB)

// Funzione per determinare dinamicamente lo spazio disponibile per firmware
size_t getMaxFirmwareSize() {
  #if defined(ARDUINO_UNO_R4_WIFI)
    // Per UNO R4 WiFi, usiamo un valore stimato
    // Nota: UNO R4 WiFi non ha API per determinare lo spazio disponibile dinamicamente
    return 120 * 1024; // 120KB (valore conservativo)
  #elif defined(ESP32_BOARD)
    // Per ESP32, determina dinamicamente in base alla partizione OTA disponibile
    const esp_partition_t* updatePartition = esp_ota_get_next_update_partition(NULL);
    if (updatePartition != NULL) {
      // Ritorna la dimensione della partizione meno un piccolo margine di sicurezza
      return updatePartition->size - 4 * 1024; // Dimensione partizione - 4KB
    } else {
      // Fallback se non riusciamo a determinare la partizione
      return 1024 * 1024; // 1MB default
    }
  #else
    return 64 * 1024; // Fallback generico se non identificata la piattaforma
  #endif
}

/*** VARIABILI GLOBALI ***/

// Client WiFi per connessione HTTP
WiFiClient client;

// Buffer per lettura dati
uint8_t buffer[BUFFER_SIZE];

// Oggetto per calcolo CRC32
CRC32 crc;

/*** PROTOTIPI FUNZIONI ***/

// Funzioni di utilità
void printDebug(const char* message);
void printDebugf(const char* format, ...);
void printProgress(unsigned long current, unsigned long total);
bool connectToWiFi();
bool downloadAndUpdate();

/**
 * Setup - inizializzazione dispositivo
 * Configura seriale, controlla stato dispositivo, 
 * avvia connessione WiFi e verifica aggiornamenti
 */
void setup() {
  // Inizializzazione comunicazione seriale
  Serial.begin(115200);
  delay(3000);  // Attesa per stabilizzazione seriale
  
  // Messaggi iniziali
  printDebug("\n\n--- OTA Firmware Updater ---");
  printDebugf("Versione: %d\n", VERSION);
  printDebugf("Scheda: %s\n", BOARD_NAME);
  
  // Mostra spazio disponibile per firmware
  size_t maxSize = getMaxFirmwareSize();
  printDebugf("Spazio disponibile per firmware: %lu KB\n", maxSize / 1024);
  
  // Connessione alla rete WiFi
  printDebug("Tentativo connessione WiFi...");
  
  if (connectToWiFi()) {
    // Controllo aggiornamenti all'avvio
    downloadAndUpdate();
  } else {
    printDebug("Errore: connessione WiFi fallita");
  }
}

/**
 * Loop - ciclo principale
 * Controlla periodicamente la disponibilità di aggiornamenti
 * e mantiene la connessione WiFi
 */
void loop() {
  static unsigned long lastCheck = 0;  // Timestamp ultimo controllo aggiornamenti
  unsigned long currentTime = millis();
  
  // Verifica periodica aggiornamenti (ogni CHECK_INTERVAL ms)
  if (currentTime - lastCheck >= CHECK_INTERVAL) {
    lastCheck = currentTime;
    
    // Controlla e ripristina connessione WiFi se necessario
    if (WiFi.status() != WL_CONNECTED) {
      printDebug("Riconnessione WiFi...");
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      delay(5000);  // Attesa per connessione
    }
    
    // Controlla aggiornamenti solo se connesso
    if (WiFi.status() == WL_CONNECTED) {
      downloadAndUpdate();
    }
  }
  
  // Spazio per altro codice applicativo...
  delay(100);
}

/**
 * Funzione per stampare messaggi di debug
 * @param message Messaggio da visualizzare
 */
void printDebug(const char* message) {
  Serial.println(message);
}

/**
 * Funzione per stampare messaggi di debug formattati (simile a printf)
 * @param format Stringa di formato
 * @param ... Argomenti variabili per format
 */
void printDebugf(const char* format, ...) {
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
}

/**
 * Visualizza progresso download con percentuale
 * @param current Bytes scaricati finora
 * @param total Dimensione totale del file (0 se sconosciuta)
 */
void printProgress(unsigned long current, unsigned long total) {
  // Calcola percentuale solo se abbiamo il totale
  int percentage = (total > 0) ? (current * 100) / total : 0;
  
  printDebugf("[%3d%%] Download: %lu", percentage, current);
  
  if (total > 0) {
    printDebugf(" di %lu", total);
  }
  
  printDebug(" bytes");
}

/**
 * Connessione alla rete WiFi configurata
 * @return true se connesso, false altrimenti
 */
bool connectToWiFi() {
  printDebugf("Connessione a %s...\n", WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  // Attende connessione con timeout
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    printDebug("WiFi connesso!");
    // Visualizza indirizzo IP ottenuto
    printDebugf("Indirizzo IP: %d.%d.%d.%d\n", 
             WiFi.localIP()[0], WiFi.localIP()[1], 
             WiFi.localIP()[2], WiFi.localIP()[3]);
    return true;
  } else {
    Serial.println();
    printDebug("Errore: connessione WiFi fallita");
    return false;
  }
}

/**
 * Funzione principale di aggiornamento OTA
 * Scarica e installa il firmware dal server configurato
 * @return true se aggiornamento completato, false in caso di errore
 */
bool downloadAndUpdate() {
  printDebug("\n--- Avvio aggiornamento OTA ---");
  
  // Step 1: Connessione al server
  printDebugf("Connessione a %s:%d\n", SERVER, SERVER_PORT);
  
  if (!client.connect(SERVER, SERVER_PORT)) {
    printDebug("Errore: connessione al server fallita");
    return false;
  }
  
  // Step 2: Invio richiesta HTTP GET
  printDebugf("Richiesta file: %s\n", PATH);
  
  client.print("GET ");
  client.print(PATH);
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(SERVER);
  client.println("Connection: close");
  client.println();
  
  // Step 3: Attesa risposta dal server con timeout
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 10000) {  // Timeout 10 secondi
      printDebug("Errore: timeout connessione");
      client.stop();
      return false;
    }
  }
  
  // Step 4: Parsing header HTTP
  String line;
  int statusCode = 0;
  bool chunkedTransfer = false;
  long contentLength = -1;
  
  // Analisi degli header della risposta HTTP
  while (client.connected()) {
    line = client.readStringUntil('\n');
    line.trim();
  
    String lowerLine = line;
    lowerLine.toLowerCase();
  
    // Estrazione codice stato
    if (lowerLine.startsWith("http/")) {
      statusCode = line.substring(9, 12).toInt();
      printDebugf("\nCodice stato: %d\n", statusCode);
    } 
    // Estrazione Content-Length (case-insensitive)
    else if (lowerLine.startsWith("content-length:")) {
      // Puoi anche chiamare trim() per eliminare eventuali spazi
      contentLength = line.substring(15).toInt();
      printDebugf("Dimensione file: %ld bytes\n", contentLength);
    } 
    // Rilevamento trasferimento chunked
    else if (lowerLine.startsWith("transfer-encoding: chunked")) {
      chunkedTransfer = true;
      printDebug("Trasferimento chunked");
    }
    
    if (line.length() == 0) {
      break;
    }
  }
  
  // Controllo risposta HTTP valida
  if (statusCode != 200) {
    printDebugf("Errore: risposta HTTP %d\n", statusCode);
    client.stop();
    return false;
  }
  
  // Step 5: Preparazione per download - Reset CRC
  crc.reset();
  
  // Step 6: Per trasferimenti chunked, determinazione dimensione da allocare
  if (chunkedTransfer) {
    // Determina lo spazio massimo disponibile per il firmware
    size_t maxFirmwareSize = getMaxFirmwareSize();
    
    #if defined(ARDUINO_UNO_R4_WIFI)
      // Lascia un margine di sicurezza
      contentLength = maxFirmwareSize - 8 * 1024;
      printDebugf("Trasferimento chunked, allocazione di %ld KB\n", contentLength / 1024);
    #elif defined(ESP32_BOARD)
      // Usa lo spazio determinato dinamicamente
      contentLength = maxFirmwareSize;
      printDebugf("Trasferimento chunked, allocazione di %ld KB\n", contentLength / 1024);
    #endif
  }
  
  // Verifica dimensione valida
  if (contentLength <= 0) {
    printDebug("Errore: dimensione file sconosciuta");
    client.stop();
    return false;
    //contentLength = 54200;
  }
  
  // Step 7: Apertura storage per il firmware
  printDebugf("Allocazione spazio: %ld bytes\n", contentLength);
  
  #if defined(ARDUINO_UNO_R4_WIFI)
    // Verifica se la dimensione richiesta eccede il massimo
    size_t maxFirmwareSize = getMaxFirmwareSize();
    if (contentLength > maxFirmwareSize) {
      printDebugf("Avviso: richiesta dimensione %ld > max %lu, limitando...\n", 
                 contentLength, maxFirmwareSize);
      contentLength = maxFirmwareSize;
    }
    
    // Apertura storage interno per firmware
    if (!InternalStorage.open(contentLength)) {
      printDebug("Errore: spazio insufficiente per l'aggiornamento");
      client.stop();
      return false;
    }
  #elif defined(ESP32_BOARD)
    // Per ESP32, verifica spazio nella partizione
    const esp_partition_t* updatePartition = esp_ota_get_next_update_partition(NULL);
    if (updatePartition != NULL) {
      if (contentLength > updatePartition->size) {
        printDebugf("Avviso: richiesta dimensione %ld > partizione %u, limitando...\n", 
                   contentLength, updatePartition->size);
        contentLength = updatePartition->size - 4 * 1024;  // 4KB margine
      }
    }
    
    // Inizia update ESP32
    if (!Update.begin(contentLength)) {
      printDebugf("Errore: impossibile allocare memoria per l'update: %s\n", Update.errorString());
      client.stop();
      return false;
    }
  #endif
  
  // Step 8: Download e scrittura del firmware
  printDebug("Download in corso...");
  
  unsigned long bytesWritten = 0;
  unsigned long lastProgressReport = 0;
  unsigned long downloadStart = millis();
  
  // Step 8a: Download con trasferimento chunked
  if (chunkedTransfer) {
    // Elaborazione trasferimento chunked
    while (client.connected()) {
      // Leggi dimensione chunk
      String chunkSizeStr = client.readStringUntil('\r');
      client.read();  // Consuma \n
      
      // Converti da hex a int
      int chunkSize = strtol(chunkSizeStr.c_str(), NULL, 16);
      
      // Chunk size 0 indica fine trasferimento
      if (chunkSize == 0) {
        printDebug("Fine trasferimento chunked");
        client.readStringUntil('\r');
        client.read();  // Consuma \n
        break;
      }
      
      // Download e processing del chunk
      int bytesRead = 0;
      while (bytesRead < chunkSize && client.connected()) {
        // Attesa dati con timeout
        if (!client.available()) {
          if (millis() - downloadStart > DOWNLOAD_TIMEOUT) {
            printDebug("Errore: timeout download");
            
            #if defined(ARDUINO_UNO_R4_WIFI)
              InternalStorage.close();
            #elif defined(ESP32_BOARD)
              Update.abort();
            #endif
            
            client.stop();
            return false;
          }
          delay(1);
          continue;
        }
        
        // Calcolo dimensione buffer da leggere
        int bytesAvailable = client.available();
        int remainingBytes = chunkSize - bytesRead;
        int toRead = BUFFER_SIZE;
        
        // Limitazione dimensione lettura
        if (remainingBytes < toRead) toRead = remainingBytes;
        if (bytesAvailable < toRead) toRead = bytesAvailable;
        
        // Lettura dati in buffer
        int actualRead = client.read(buffer, toRead);
        
        if (actualRead > 0) {
          // Aggiornamento CRC
          crc.update(buffer, actualRead);
          
          // Scrittura dati nel firmware storage
          #if defined(ARDUINO_UNO_R4_WIFI)
            for (int i = 0; i < actualRead; i++) {
              InternalStorage.write(buffer[i]);
            }
          #elif defined(ESP32_BOARD)
            if (Update.write(buffer, actualRead) != actualRead) {
              printDebugf("Errore scrittura update: %s\n", Update.errorString());
              Update.abort();
              client.stop();
              return false;
            }
          #endif
          
          bytesRead += actualRead;
          bytesWritten += actualRead;
          
          // Aggiornamento progresso ogni 4KB
          if (bytesWritten - lastProgressReport >= 4096) {
            lastProgressReport = bytesWritten;
            printProgress(bytesWritten, 0);  // Dimensione totale sconosciuta
          }
        }
      }
      
      // Lettura terminatore chunk (CRLF)
      client.readStringUntil('\r');
      client.read();  // Consuma \n
    }
  } else {
    // Step 8b: Download normale con Content-Length
    while (bytesWritten < contentLength && client.connected()) {
      // Attesa dati con timeout
      if (!client.available()) {
        if (millis() - downloadStart > DOWNLOAD_TIMEOUT) {
          printDebug("Errore: timeout download");
          
          #if defined(ARDUINO_UNO_R4_WIFI)
            InternalStorage.close();
          #elif defined(ESP32_BOARD)
            Update.abort();
          #endif
          
          client.stop();
          return false;
        }
        delay(1);
        continue;
      }
      
      // Calcolo dimensione buffer da leggere
      int bytesAvailable = client.available();
      long remainingBytes = contentLength - bytesWritten;
      int toRead = BUFFER_SIZE;
      
      // Limitazione dimensione lettura
      if (remainingBytes < toRead) toRead = (int)remainingBytes;
      if (bytesAvailable < toRead) toRead = bytesAvailable;
      
      // Lettura dati in buffer
      int actualRead = client.read(buffer, toRead);
      
      if (actualRead > 0) {
        // Aggiornamento CRC
        crc.update(buffer, actualRead);
        
        // Scrittura dati nel firmware storage
        #if defined(ARDUINO_UNO_R4_WIFI)
          for (int i = 0; i < actualRead; i++) {
            InternalStorage.write(buffer[i]);
          }
        #elif defined(ESP32_BOARD)
          if (Update.write(buffer, actualRead) != actualRead) {
            printDebugf("Errore scrittura update: %s\n", Update.errorString());
            Update.abort();
            client.stop();
            return false;
          }
        #endif
        
        bytesWritten += actualRead;
        
        // Aggiornamento progresso ogni 4KB
        if (bytesWritten - lastProgressReport >= 4096) {
          lastProgressReport = bytesWritten;
          printProgress(bytesWritten, contentLength);
        }
      }
    }
  }
  
  // Step 9: Calcolo CRC finale
  uint32_t firmwareCRC = crc.finalize();
  
  // Step 10: Chiusura connessione
  client.stop();
  
  // Step 11: Verifica completezza download
  if ((chunkedTransfer && bytesWritten > 0) || (bytesWritten == contentLength)) {
    printDebugf("Download completato: %lu bytes (CRC32: %08lX)\n", bytesWritten, firmwareCRC);
    
    // Step 12: Finalizzazione update e riavvio
    #if defined(ARDUINO_UNO_R4_WIFI)
      InternalStorage.close();
      
      // Applica update e riavvia
      printDebug("Applicazione aggiornamento e riavvio...");
      Serial.flush();
      delay(1000);
      
      // Questa linea non ritorna se l'aggiornamento ha successo
      InternalStorage.apply();
      
      // Qui solo in caso di errore
      printDebug("Errore: applicazione aggiornamento fallita");
    #elif defined(ESP32_BOARD)
      if (Update.end(true)) {
        printDebug("Update completato correttamente!");
        printDebug("Riavvio in corso...");
        
        Serial.flush();
        delay(1000);
        ESP.restart();
      } else {
        printDebugf("Errore update: %s\n", Update.errorString());
      }
    #endif
    
    return false;  // Non dovremmo mai arrivare qui se successo
  } else {
    // Download incompleto
    printDebugf("Errore: download incompleto. Ricevuti %lu", bytesWritten);
    if (!chunkedTransfer) {
      printDebugf(" di %ld", contentLength);
    }
    printDebug(" bytes");
    
    #if defined(ARDUINO_UNO_R4_WIFI)
      InternalStorage.close();
    #elif defined(ESP32_BOARD)
      Update.abort();
    #endif
    
    return false;
  }
}