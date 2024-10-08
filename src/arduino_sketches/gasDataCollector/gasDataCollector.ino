/*
  This file references the logical structure and parts of 
  the code in the loop function of the file 'gasAcquisition2.ino' 
  from bhopal84 (https://github.com/ronibandini/bhopal84), 
  and heavily refactors the original code.
  
  Original work Copyright (c) 2022 Roni Bandini
  Modified work Copyright (c) 2024 Hang Yu

  See NOTICE file for full copyright and license details.
*/

/*
  Diese Datei bezieht sich auf die logische Struktur und Teile 
  des Codes in der loop-Funktion der Datei 'gasAcquisition2.ino' 
  von bhopal84 (https://github.com/ronibandini/bhopal84), 
  und überarbeitet den originalen Code stark.
  
  Ursprüngliches Werk Copyright (c) 2022 Roni Bandini
  Modifiziertes Werk Copyright (c) 2024 Hang Yu

  Siehe NOTICE-Datei für vollständige Copyright- und Lizenzdetails.
*/

/* Headerdateien ------------------------------------------------------------------*/
// I2C
#include <Wire.h>

// SPI
#include <SPI.h>

// Gassensor
#include <DFRobot_MICS.h>

// SD
#include <SD.h>

// OLED
#include <SSD_13XX.h>


/* Globale Konstanten und Variablen ------------------------------------------------------------------*/
// Gassensor
#define MICS_SDA_PIN 6
#define MICS_SCL_PIN 7
constexpr uint8_t CALIBRATION_TIME = 3;  // Standardmäßige Vorwärmkalibrierungszeit von 3 Minuten
DFRobot_MICS_I2C mics(&Wire, 0x75);

// SD
#define SD_CS_PIN 22
File dataFile;
String fileName;

// OLED
#define OLED_CS_PIN 18
#define OLED_DC_PIN 16
SSD_13XX oled = SSD_13XX(OLED_CS_PIN, OLED_DC_PIN);

// Gasarten (für MiCS-Bibliotheksfunktionen, da sie nur uint8_t unsigned Typ (Zahlen) unterstützen)
constexpr uint8_t GAS_TYPE[] = { CH4, C2H5OH, H2, NH3, CO, NO2 };

// Datenstatistik
#define SERIAL_PRINT_CSV true           // Ob CSV-Inhalte auch auf der seriellen Schnittstelle gedruckt werden sollen
#define SERIAL_PRINT_SAMPLE false       // Ob die Messwerte jeder Probe gedruckt werden sollen
constexpr int SAMPLE_NUMBER = 4;        // Anzahl der Probenahmen pro Runde
constexpr int SAMPLE_TIME_FRAME = 375;  // Probenahme alle 375ms
constexpr float DEFAULT_MIN = 50000;    // Standardminimalwert
int sampleCounter = 0;                  // Probenzähler für diese Runde

int timeStamp = SAMPLE_NUMBER * SAMPLE_TIME_FRAME;                       // Aktueller Zeitstempel (Anfangswert berechnet)
constexpr int TIME_STAMP_INCREMENT = SAMPLE_NUMBER * SAMPLE_TIME_FRAME;  // Zeitstempelinkrement
int roundCounter = 0;                                                    // Zähler für Erkennungsrunden


// Einzelprobenkonzentrationen für sechs Gase
float gasConcentrations[6] = { 0, 0, 0, 0, 0, 0 };

// Datenverarbeitungseinstellungen
constexpr uint8_t WMA_WINDOW_SIZE = 5;                                         // WMA-Filter Gleitfenstergröße
constexpr float WMA_WEIGHTS[WMA_WINDOW_SIZE] = { 0.1, 0.15, 0.2, 0.25, 0.3 };  // WMA-Filter Gewichtungsarray, neueste Daten haben das höchste Gewicht

// Filterpufferdaten
float gasBuffer[6][WMA_WINDOW_SIZE];        // Erstellt einen zirkulären Puffer für jeden Gastyp
int bufferIndex[6] = { 0, 0, 0, 0, 0, 0 };  // Aktueller Index für jeden Puffer


// Struktur definieren
struct MLGasData {
  float sum;
  float min;
  float max;
  float avg;
};

// Array mit der definierten Struktur initialisieren, um Probendaten für jede Runde zu speichern und zu berechnen
MLGasData mlGasData[6] = {
  { 0, DEFAULT_MIN, 0, 0 },  // CH4: Summe, Min, Max, Durchschnitt
  { 0, DEFAULT_MIN, 0, 0 },  // C2H5OH: Summe, Min, Max, Durchschnitt
  { 0, DEFAULT_MIN, 0, 0 },  // H2: Summe, Min, Max, Durchschnitt
  { 0, DEFAULT_MIN, 0, 0 },  // NH3: Summe, Min, Max, Durchschnitt
  { 0, DEFAULT_MIN, 0, 0 },  // CO: Summe, Min, Max, Durchschnitt
  { 0, DEFAULT_MIN, 0, 0 }   // NO2: Summe, Min, Max, Durchschnitt
};


/* Flags ------------------------------------------------------------------*/
// SD
bool sdConnected_flag = false;




/* Gaspuffer für Filterung initialisieren ------------------------------------------------------------------*/
void initGasBuffers() {
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < WMA_WINDOW_SIZE; ++j) {
      gasBuffer[i][j] = 0.0;
    }
    bufferIndex[i] = 0;
  }
}


/* WMA-Filter anwenden ------------------------------------------------------------------*/
float applyWMA(float newValue, int gasType) {
  // Puffer aktualisieren
  gasBuffer[gasType][bufferIndex[gasType]] = newValue;
  bufferIndex[gasType] = (bufferIndex[gasType] + 1) % WMA_WINDOW_SIZE;

  // Gewichteten Durchschnitt berechnen
  float sum = 0;
  for (int i = 0; i < WMA_WINDOW_SIZE; ++i) {
    int index = (bufferIndex[gasType] - i + WMA_WINDOW_SIZE) % WMA_WINDOW_SIZE;
    sum += gasBuffer[gasType][index] * WMA_WEIGHTS[i];
  }

  return sum;
}


/* Gaskonzentrationsdaten vom Sensor holen, filtern und in Array schreiben ------------------------------------------------------------------*/
void getGasData() {
  for (int i = 0; i < 6; ++i) {
    float rawValue = mics.getGasData(GAS_TYPE[i]);
    gasConcentrations[i] = applyWMA(rawValue, i);
  }
}


/* Überprüfen Sie die Anzahl der vorhandenen Dateien auf der SD-Karte und generieren Sie einen neuen Dateinamen ------------------------------------------------------------------*/
String generateFileName() {
  int fileIndex = 1;
  String fileName;
  while (true) {
    fileName = "/GasCollection" + String(fileIndex) + ".csv";
    if (!SD.exists(fileName)) {
      return fileName;
    }
    fileIndex++;
  }
}


/* OLED-Bildschirm initialisieren ------------------------------------------------------------------*/
void setupOLED() {
  oled.begin();
  oled.setBrightness(15);
  oled.clearScreen();
  oled.setRotation(0);
}


/* SD-Karte initialisieren ------------------------------------------------------------------*/
void setupSD() {
  SPI.begin(21, 20, 19, SD_CS_PIN);  // SCK, MISO, MOSI, SS (für SD)
  Serial.print("Verbindung zum MicroSD-Modul ");
  int retryCount = 0;
  while (!SD.begin(SD_CS_PIN) && retryCount < 5) {
    delay(1000);
    Serial.print(".");
    retryCount++;
  }
  // Prüfen, ob das SD-Modul verbunden ist
  if (SD.begin(SD_CS_PIN)) {
    Serial.println("  [OK]");
    Serial.println();
  } else {
    Serial.println("  [Fehlgeschlagen]");
    Serial.println("[FEHLER] Kartenmontage fehlgeschlagen!");
    Serial.println();
    // Funktion verlassen, wenn die SD-Modulverbindung fehlschlägt
    return;
  }

  // Versuchen Sie, den SD-Kartentyp zu erhalten, um zu prüfen, ob die SD-Karte korrekt angeschlossen ist
  uint8_t cardType = SD.cardType();
  if (cardType != CARD_NONE) {
    sdConnected_flag = true;
    Serial.println("[INFO] SD-Karte angeschlossen");
    Serial.println();
  } else {
    Serial.println("[FEHLER] Keine SD-Karte angeschlossen!");
    Serial.println();
    // Funktion verlassen, wenn die SD-Kartenerkennung fehlschlägt
    return;
  }

  // SD-Karteninformationen drucken
  Serial.println();
  Serial.println("---------- MicroSD Info ------------");
  Serial.print("SD-Kartentyp: ");
  switch (cardType) {
    case CARD_MMC: Serial.println("MMC"); break;
    case CARD_SD: Serial.println("SDSC"); break;
    case CARD_SDHC: Serial.println("SDHC"); break;
    default: Serial.println("UNBEKANNT");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD-Kartengröße: %lluMB\n", cardSize);
  Serial.println("------------------------------------");
  Serial.println();


  // Neuen Dateinamen generieren
  fileName = generateFileName();

  // Neue CSV-Datei erstellen und Kopfzeile schreiben
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("timestamp,CH4_avg,C2H5OH_avg,H2_avg,NH3_avg,CO_avg,NO2_avg,CH4_min,C2H5OH_min,H2_min,NH3_min,CO_min,NO2_min,CH4_max,C2H5OH_max,H2_max,NH3_max,CO_max,NO2_max");
    dataFile.close();
    Serial.println(fileName + " erfolgreich erstellt.");
  } else {
    Serial.println("Fehler beim Erstellen von " + fileName);
  }
}


/* Gassensor initialisieren ------------------------------------------------------------------*/
void setupMiCS() {
  Wire.begin(MICS_SDA_PIN, MICS_SCL_PIN);

  Serial.print("Verbindung zum Gassensor ");
  int retryCount = 0;
  while (!mics.begin() && retryCount < 10) {
    Serial.print(".");
    delay(1000);
    retryCount++;
  }
  if (mics.begin()) {
    Serial.println("  [OK]");
  } else {
    Serial.println("  [Fehlgeschlagen]");
    Serial.println("[SCHWERWIEGENDER FEHLER] Gassensor NICHT verbunden, bitte überprüfen Sie die Verbindung!");
    // ESP-System neu starten
    Serial.printf("[WARNUNG] System wird neu gestartet!\n\n\n\n\n");
    ESP.restart();
    return;
  }

  uint8_t mode = mics.getPowerState();
  if (mode == SLEEP_MODE) {
    mics.wakeUpMode();
    Serial.println("Sensor erfolgreich aufgeweckt");
  } else {
    Serial.println("Der Sensor wacht auf");
  }
  Serial.println();

  // Titel zeichnen

  oled.setCursor(0, 10);
  oled.setTextColor(ORANGE);
  oled.setTextScale(2);
  oled.print("TinyML");

  oled.setTextColor(WHITE);
  oled.setCursor(0, 30);
  oled.print("Mehrgas-");
  oled.setCursor(0, 50);
  oled.print("Datensammler");

  oled.setTextColor(YELLOW);
  oled.setCursor(0, 80);
  oled.print("Kalibrierung...");

  int totalCalibrationTime = CALIBRATION_TIME * 60;  // Gesamte Vorwärmzeit berechnen (in Sekunden)
  int remainingTime = totalCalibrationTime;          // Verbleibende Zeit initialisieren
  int currentMillis;
  int previousMillis;

  // Schleife fortsetzen, solange sich der Sensor im Vorwärmzustand befindet, alle 10 Sekunden die verbleibende Zeit auf der seriellen Schnittstelle ausgeben
  while (!mics.warmUpTime(CALIBRATION_TIME)) {
    unsigned long currentMillis = millis();  // Aktuelle Millisekunden abrufen

    // Bildschirminhalt alle 1 Sekunde aktualisieren
    if (currentMillis - previousMillis >= 1000) {
      previousMillis = currentMillis;

      // Alte verbleibende Zeit überdecken und neue zeichnen
      oled.fillRect(0, 100, 60, 20, BLACK);
      oled.setTextScale(2);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 100);
      oled.printf("%d s", remainingTime);


      // Verbleibende Zeit auf serieller Schnittstelle ausgeben, alle 10 Sekunden
      if (remainingTime % 10 == 0) {
        Serial.printf("[INFO] Bitte warten, verbleibende Zeit: %ds\n", remainingTime);
      }

      remainingTime--;  // Verbleibende Zeit um 1s verringern
    }

    delay(1);  // Kleine Verzögerung einfügen, um CPU-Auslastung zu reduzieren
  }
}


/* Hauptinitialisierung ------------------------------------------------------------------*/
void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(">>>>>>>>>> TinyML Gasdatenerfassungsprogramm <<<<<<<<<<");
  Serial.println();

  setupSD();  // SD-Karte initialisieren

  setupOLED();  // OLED-Bildschirm initialisieren

  initGasBuffers();  // Gaspuffer initialisieren (für Filterung)

  setupMiCS();  // Gassensor initialisieren

  // Titel zeichnen
  oled.clearScreen();
  oled.setTextScale(2);
  oled.setTextColor(BLUE);
  oled.setCursor(0, 10);
  oled.print("TinyML");

  oled.setTextColor(WHITE);
  oled.setCursor(0, 30);
  oled.print("Mehrgas-");
  oled.setCursor(0, 50);
  oled.print("Datensammler");

  oled.setCursor(0, 80);
  oled.print("Runde #");
}


/* Hauptschleife ------------------------------------------------------------------*/
void loop() {

  // Wenn die maximale Anzahl von Messungen pro Gruppe erreicht ist, Berechnung durchführen
  if (sampleCounter == SAMPLE_NUMBER) {

    sampleCounter = 0;  // Zähler zurücksetzen
    roundCounter++;     // Rundenzähler erhöhen

    // Durchschnitt berechnen
    for (int i = 0; i < 6; ++i) {
      mlGasData[i].avg = mlGasData[i].sum / SAMPLE_NUMBER;
    }

    // Aktuelle Runde auf dem Bildschirm anzeigen
    oled.fillRect(70, 80, 58, 20, BLACK);
    oled.setTextScale(2);
    oled.setTextColor(ORANGE);
    oled.setCursor(70, 80);
    oled.print(roundCounter);

    // Minimalwerte normalisieren
    for (int i = 0; i < 6; ++i) {
      if (mlGasData[i].min == DEFAULT_MIN) {
        mlGasData[i].min = 0;
      }
    }


#if SERIAL_PRINT_CSV  // Wenn CSV-Druck eingestellt ist, die in CSV gespeicherten Daten auch auf der seriellen Schnittstelle ausgeben

    // Zeitstempel ausgeben
    Serial.print(timeStamp);
    Serial.print(",");

    // Durchschnittswerte dieser Runde ausgeben
    for (int i = 0; i < 6; ++i) {
      Serial.print(mlGasData[i].avg);
      Serial.print(",");
    }

    // Minimalwerte dieser Runde ausgeben
    for (int i = 0; i < 6; ++i) {
      Serial.print(mlGasData[i].min, 1);
      Serial.print(",");
    }

    // Maximalwerte dieser Runde ausgeben
    for (int i = 0; i < 6; ++i) {
      Serial.print(mlGasData[i].max, 1);
      if (i < 5) {  // Wenn es nicht das letzte Element ist, ein Komma ausgeben
        Serial.print(",");
      }
    }
    Serial.println();

#endif


    // Wenn eine SD-Karte angeschlossen ist, neue Daten zur CSV-Datei auf der SD-Karte hinzufügen
    if (sdConnected_flag) {
      dataFile = SD.open(fileName, FILE_APPEND);
      if (dataFile) {
        dataFile.print(timeStamp);
        dataFile.print(",");

        for (int i = 0; i < 6; ++i) {
          dataFile.print(mlGasData[i].avg);
          dataFile.print(",");
        }

        for (int i = 0; i < 6; ++i) {
          dataFile.print(mlGasData[i].min, 1);
          dataFile.print(",");
        }

        for (int i = 0; i < 6; ++i) {
          dataFile.print(mlGasData[i].max, 1);
          if (i < 5) {
            dataFile.print(",");
          }
        }
        dataFile.println();
        dataFile.close();
        // Serial.println("Daten an " + fileName + " angehängt");
      } else {
        Serial.println("[FEHLER] Fehler beim Öffnen von " + fileName);
      }
    }


    // Gasdaten zurücksetzen, sum, min, max zurücksetzen; avg muss nicht zurückgesetzt werden, da es bei jeder Rundenberechnung überschrieben wird
    for (int i = 0; i < 6; ++i) {
      mlGasData[i].sum = 0;
      mlGasData[i].min = DEFAULT_MIN;
      mlGasData[i].max = 0;
    }

    timeStamp = timeStamp + TIME_STAMP_INCREMENT;  // Zeitstempel um entsprechenden Wert erhöhen

  } else {

    sampleCounter++;  // Probenzähler erhöhen

    // Gaslesungen für diese Gruppe vom Sensor abrufen
    getGasData();

    // Daten summieren, um am Ende jeder Runde den Durchschnitt zu berechnen
    for (int i = 0; i < 6; ++i) {
      mlGasData[i].sum = mlGasData[i].sum + gasConcentrations[i];
    }


#if SERIAL_PRINT_SAMPLE  // Wenn das Drucken jeder Probenablesung eingestellt ist, auf der seriellen Schnittstelle ausgeben

    Serial.println("----------------------- Probe " + String(sampleCounter));
    for (int i = 0; i < 6; ++i) {
      Serial.print(gasConcentrations[i], 1);
      if (i < 5) {  // Wenn es nicht das letzte Element ist, ein Komma ausgeben
        Serial.print(",");
      }
    }
    Serial.println();

#endif

    // Maximalwert dieser Runde bestimmen
    for (int i = 0; i < 6; ++i) {
      if (gasConcentrations[i] > mlGasData[i].max) {  // Wenn es nicht das letzte Element ist, ein Komma ausgeben
        mlGasData[i].max = gasConcentrations[i];
      }
    }


    // Minimalwert dieser Runde bestimmen
    for (int i = 0; i < 6; ++i) {
      if (gasConcentrations[i] < mlGasData[i].min) {  // Wenn es nicht das letzte Element ist, ein Komma ausgeben
        mlGasData[i].min = gasConcentrations[i];
      }
    }
  }

  delay(SAMPLE_TIME_FRAME);  // Verzögerung für einzelne Probenahme
}