/*
  OTA Firmware Updater semplificato per Arduino UNO R4 WiFi
  
  Scarica un firmware tramite HTTP e lo applica utilizzando InternalStorage
  Supporta sia trasferimenti normali che chunked
*/

#include <WiFiS3.h>

#define NO_OTA_NETWORK
#include <ArduinoOTA.h> // solo per InternalStorage

// Definizione versione
const short VERSION = 10;

// Configurazione WiFi
const char* WIFI_SSID = "test";
const char* WIFI_PASS = "12345678";

// Configurazione server
const char* SERVER = "192.168.1.25";
const int SERVER_PORT = 9003;
const char* PATH = "/api/v1/download-shared-object/aHR0cDovLzEyNy4wLjAuMTo5MDAwL290YS9QbGF5QW5pbWF0aW9uLmluby5iaW4_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD0zSUJWOEkzWEJEMDlDRks0SFdSUSUyRjIwMjUwMzA2JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI1MDMwNlQyMTMwMjBaJlgtQW16LUV4cGlyZXM9NDMyMDAmWC1BbXotU2VjdXJpdHktVG9rZW49ZXlKaGJHY2lPaUpJVXpVeE1pSXNJblI1Y0NJNklrcFhWQ0o5LmV5SmhZMk5sYzNOTFpYa2lPaUl6U1VKV09Fa3pXRUpFTURsRFJrczBTRmRTVVNJc0ltVjRjQ0k2TVRjME1UTXdOak0yTWl3aWNHRnlaVzUwSWpvaVlXUnRhVzRpZlEuTER3a2dqTy1QY0hCYXJQb3FIdW9UT3NxZnlsMlVkVHZtUjdPWVRiMWI3T1BEZHEyaEsyYnVOb2NWb21QZFBNQW4xUGw0ZWdqWkhpRUo3ZGczWUM4aHcmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JnZlcnNpb25JZD1udWxsJlgtQW16LVNpZ25hdHVyZT05ZWY4NmQ1MTcwMzBiZTI4ODU2Yzc5MjcxOTY3MTlhNjJjY2ZlZjU5ODVmNDAzNWM0NmEyOTMyZjAxNmRjZDky";

// Configurazione OTA
const unsigned long CHECK_INTERVAL = 30000;  // 30 secondi
const int DOWNLOAD_TIMEOUT = 60000;          // 60 secondi
const int MAX_FIRMWARE_SIZE = 128 * 1024;    // 128KB massimo
const int BUFFER_SIZE = 1024;                // Buffer per lettura blocchi

WiFiClient client;

// Buffer per lettura blocchi
uint8_t buffer[BUFFER_SIZE];

// Funzione per scaricare e applicare l'aggiornamento
bool downloadAndUpdate() {
  Serial.println("\n--- Avvio aggiornamento OTA ---");

  // Connessione al server
  Serial.print("Connessione a ");
  Serial.println(SERVER);
  
  if (!client.connect(SERVER, SERVER_PORT)) {
    Serial.println("Errore: connessione al server fallita");
    return false;
  }
  
  // Invio richiesta HTTP
  Serial.print("Richiesta file: ");
  Serial.println(PATH);
  
  client.print("GET ");
  client.print(PATH);
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(SERVER);
  client.println("Connection: close");
  client.println();
  
  // Attesa risposta
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 10000) {
      Serial.println("Errore: timeout connessione");
      client.stop();
      return false;
    }
  }
  
  // Parsing headers HTTP
  String line;
  int statusCode = 0;
  bool chunkedTransfer = false;
  long contentLength = -1;
  
  while (client.connected()) {
    line = client.readStringUntil('\n');
    line.trim();
    
    if (line.startsWith("HTTP/")) {
      statusCode = line.substring(9, 12).toInt();
      Serial.print("Codice stato: ");
      Serial.println(statusCode);
    } else if (line.startsWith("Content-Length:")) {
      contentLength = line.substring(15).toInt();
      Serial.print("Dimensione file: ");
      Serial.print(contentLength);
      Serial.println(" bytes");
    } else if (line.startsWith("Transfer-Encoding: chunked")) {
      chunkedTransfer = true;
      Serial.println("Trasferimento chunked");
    }
    
    // Linea vuota indica fine degli headers
    if (line.length() == 0) {
      break;
    }
  }
  
  // Controllo errori
  if (statusCode != 200) {
    Serial.print("Errore: risposta HTTP ");
    Serial.println(statusCode);
    client.stop();
    return false;
  }
  
  // Se è un trasferimento chunked, dobbiamo prima determinare la dimensione
  if (chunkedTransfer) {
    // Possiamo stimare una dimensione massima o usare una dimensione fissa
    contentLength = 64 * 1024; // 64KB come dimensione sicura di default
    Serial.println("Trasferimento chunked, utilizzo dimensione fissa di 64KB");
  }
  
  if (contentLength <= 0) {
    Serial.println("Errore: dimensione file sconosciuta");
    client.stop();
    return false;
  }
  
  // Apertura InternalStorage
  Serial.print("Allocazione spazio: ");
  Serial.print(contentLength);
  Serial.println(" bytes");
  
  if (!InternalStorage.open(contentLength)) {
    Serial.println("Errore: spazio insufficiente per l'aggiornamento");
    client.stop();
    return false;
  }
  
  // Download e scrittura del firmware
  Serial.println("Download in corso...");
  
  unsigned long bytesWritten = 0;
  unsigned long downloadStart = millis();
  
  if (chunkedTransfer) {
    // Gestione trasferimento chunked
    while (client.connected()) {
      // Leggi dimensione chunk
      String chunkSizeStr = client.readStringUntil('\r');
      client.read(); // Consuma \n
      
      // Converti da hex a int
      int chunkSize = strtol(chunkSizeStr.c_str(), NULL, 16);
      
      if (chunkSize == 0) {
        // Ultimo chunk
        Serial.println("Fine trasferimento chunked");
        client.readStringUntil('\r');
        client.read(); // Consuma \n
        break;
      }
      
      // Scarica il chunk
      int bytesRead = 0;
      while (bytesRead < chunkSize && client.connected()) {
        if (!client.available()) {
          if (millis() - downloadStart > DOWNLOAD_TIMEOUT) {
            Serial.println("Errore: timeout download");
            InternalStorage.close();
            client.stop();
            return false;
          }
          delay(1);
          continue;
        }
        
        // Leggi in blocchi
        int toRead = min(min(BUFFER_SIZE, chunkSize - bytesRead), client.available());
        int actualRead = client.read(buffer, toRead);
        
        if (actualRead > 0) {
          // Scrivi in InternalStorage
          for (int i = 0; i < actualRead; i++) {
            InternalStorage.write(buffer[i]);
          }
          
          bytesRead += actualRead;
          bytesWritten += actualRead;
          
          // Mostra progresso
          if (bytesWritten % 4096 == 0) {
            Serial.print("Download: ");
            Serial.print(bytesWritten);
            Serial.println(" bytes");
          }
        }
      }
      
      // Leggi CRLF alla fine del chunk
      client.readStringUntil('\r');
      client.read(); // Consuma \n
    }
  } else {
    // Download normale con Content-Length
    while (bytesWritten < contentLength && client.connected()) {
      if (!client.available()) {
        if (millis() - downloadStart > DOWNLOAD_TIMEOUT) {
          Serial.println("Errore: timeout download");
          InternalStorage.close();
          client.stop();
          return false;
        }
        delay(1);
        continue;
      }
      
      // Leggi in blocchi
      int toRead = min(min(BUFFER_SIZE, contentLength - bytesWritten), client.available());
      int actualRead = client.read(buffer, toRead);
      
      if (actualRead > 0) {
        // Scrivi in InternalStorage
        for (int i = 0; i < actualRead; i++) {
          InternalStorage.write(buffer[i]);
        }
        
        bytesWritten += actualRead;
        
        // Mostra progresso
        if (bytesWritten % 4096 == 0) {
          Serial.print("Download: ");
          Serial.print(bytesWritten);
          Serial.print(" di ");
          Serial.print(contentLength);
          Serial.println(" bytes");
        }
      }
    }
  }
  
  // Chiudi connessione e InternalStorage
  client.stop();
  InternalStorage.close();
  
  // Verifica download
  if ((chunkedTransfer && bytesWritten > 0) || (bytesWritten == contentLength)) {
    Serial.print("Download completato: ");
    Serial.print(bytesWritten);
    Serial.println(" bytes");
    
    // Applica aggiornamento
    Serial.println("Applicazione aggiornamento e riavvio...");
    Serial.flush();
    delay(1000);
    
    // Questa linea non ritorna se l'aggiornamento ha successo
    InternalStorage.apply();
    
    // Se arriviamo qui, c'è stato un errore
    Serial.println("Errore: applicazione aggiornamento fallita");
    return false;
  } else {
    Serial.print("Errore: download incompleto. Ricevuti ");
    Serial.print(bytesWritten);
    if (!chunkedTransfer) {
      Serial.print(" di ");
      Serial.print(contentLength);
    }
    Serial.println(" bytes");
    return false;
  }
}

void setup() {
  // Inizializza seriale
  Serial.begin(115200);
  delay(3000); // Attendi inizializzazione seriale
  
  Serial.println("\n\n--- OTA Firmware Updater ---");
  Serial.print("Versione: ");
  Serial.println(VERSION);
  
  // Connessione WiFi
  Serial.print("Connessione a ");
  Serial.print(WIFI_SSID);
  Serial.println("...");
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connesso!");
    Serial.print("Indirizzo IP: ");
    Serial.println(WiFi.localIP());
    
    // Esegui aggiornamento all'avvio
    downloadAndUpdate();
  } else {
    Serial.println();
    Serial.println("Errore: connessione WiFi fallita");
  }
}

void loop() {
  static unsigned long lastCheck = 0;
  unsigned long currentTime = millis();
  
  // Controlla periodicamente aggiornamenti
  if (currentTime - lastCheck >= CHECK_INTERVAL) {
    lastCheck = currentTime;
    
    // Riconnetti se necessario
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Riconnessione WiFi...");
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      delay(5000);
    }
    
    // Verifica aggiornamenti
    if (WiFi.status() == WL_CONNECTED) {
      downloadAndUpdate();
    }
  }
  
  // Altro codice loop...
  delay(100);
}