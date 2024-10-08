/*******************************************************************************
* Copyright (c) 2024 Hang Yu
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*     http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and 
* limitations under the License. 
*******************************************************************************/

/* ========================================================== Bereich für Definitionen ========================================================== */

/* Header-Dateien ------------------------------------------------------------------*/

// System- und Hardwarebezogen
#include <Wire.h>  // I2C
#include <SPI.h>   // SPI

// Sensorbezogen
#include <DFRobot_MICS.h>  // Gassensor

// Anzeigebezogen
#include <SSD_13XX.h>     // OLED-Bildschirm
#include "_icons/logo.c"  // Startbildschirm-Logo

// Speicherbezogen
#include <SD.h>
#include <ArduinoJson.h>

// Zeitbezogen
#include <RTClib.h>     // Externe RTC
#include <ESP32Time.h>  // Interne RTC

// Netzwerkbezogen
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>  // Thingsboard

// Andere Funktionen
#include <cfloat>                    // Für Kurvendarstellung, um FLT_MAX und FLT_MIN zu verwenden
#include <vector>                    // Für Kurvendarstellung, um dynamisches Array zu verwenden
#include <Gas-TinyML_inferencing.h>  // TinyML


/* Konfiguration ------------------------------------------------------------------*/

// Vorheizzeit-Einstellung
constexpr uint8_t CALIBRATION_TIME = 3;  // Standard-Vorheizkalibrierungszeit beträgt 3 Minuten

// Mutex-Einstellungen
#define USE_MUTEX true  // Mutex (Mutual Exclusion), standardmäßig aktiviert für SPI und I2C

// WLAN-Einstellungen
constexpr char *WIFI_SSID = "S23U";         // WLAN-Name
constexpr char *WIFI_PASSWORD = "QWERASD10242048";  // WLAN-Passwort
constexpr uint8_t WIFI_INIT_RETRY = 5;              // WLAN-Neustartversuche beim Start

// Zeiteinstellungen
constexpr int32_t UTC_OFFSET_SEC = 3600;                                                                                       // UTC-Differenz, Winterzeit in Deutschland ist UTC+1, daher 3600
constexpr int32_t DAYLIGHT_OFFSET_SEC = 3600;                                                                                  // Sommerzeit-Einstellung, Sommerzeit in Deutschland ist UTC+2, daher 3600
constexpr char DAYS_OF_THE_WEEK[7][12] = { "Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag" };  // Namen der Wochentage
constexpr char *NTP_SERVER[] = { "pool.ntp.org", "time.nist.gov", "time.windows.com" };                                        // NTP-Serveradressen
constexpr uint8_t NTP_MAX_RETRY = 2;                                                                                           // Maximale Anzahl der Wiederholungen pro NTP-Serveradresse

// Abtastung und Bildschirmeinstellungen
constexpr uint16_t SAMPLING_INTERVAL = 250;  // Abtastintervall des Gassensors (Millisekunden), Änderungen sind normalerweise nicht erforderlich
constexpr uint16_t REFRESH_INTERVAL = 250;   // Bildschirmaktualisierungsintervall (Millisekunden), Änderungen sind normalerweise nicht erforderlich

// Datenverarbeitungseinstellungen
constexpr uint8_t WMA_WINDOW_SIZE = 5;                                         // WMA-Filter Gleitfenstergröße
constexpr float WMA_WEIGHTS[WMA_WINDOW_SIZE] = { 0.1, 0.15, 0.2, 0.25, 0.3 };  // WMA-Filter Gewichtungsarray, wobei die neuesten Daten das größte Gewicht haben

// Alarmeinstellungen
uint8_t GAS_CHECK_time = 3;     // Globale Variable, Schwellenwert für die Anzahl der aufeinanderfolgenden Konzentrationsmessungen (Anzahl der aufeinanderfolgenden Überprüfungen, um zu bestätigen, ob die Konzentration den Grenzwert überschreitet, um Fehlalarme zu vermeiden)
uint16_t ALARM_interval = 250;  // Globale Variable, Intervallzeit für das intermittierende Umschalten des akustisch-visuellen Alarms (in Millisekunden)
uint16_t ALARM_delay = 1500;    // Globale Variable, Verzögerungszeit nach dem Ende des Alarms (in Millisekunden)

// Hauptanzeigeneinstellungen
uint8_t YELLOW_percentage = 80;               // Globale Variable, Startschwellenwert für die Anzeige der Gaskonzentration in Gelb (prozentualer Anteil des festgelegten Alarmgrenzwertes)
constexpr uint16_t KEEP_REFRESH_TIME = 3000;  // Dauer der kontinuierlichen Aktualisierung des Displays nach einer Gaswertänderung (Millisekunden)

// SD-Karteneinstellungen
constexpr uint32_t SD_CHECK_PERIOD_MS = 60 * 1000;  // Überprüfungsintervall für SD-Karte (Millisekunden), standardmäßig 1 Minute
uint8_t MIN_FREE_SPACE_percentage = 10;             // Globale Variable, minimaler verbleibender Speicherplatz auf der SD-Karte in Prozent, standardmäßig 10%
constexpr char *LOG_FILE_EXTENSION = ".txt";
constexpr char *CONFIG_FILE_NAME = "/config.json";

// Tastatureinstellungen
constexpr uint16_t LONG_PRESS_DURATION = 600;   // Dauer bis zur Aktivierung durch Langdruck (Millisekunden)
constexpr uint16_t HOLD_REPEAT_INTERVAL = 200;  // Intervall für wiederholte Aktivierungen bei gehaltener Taste (Millisekunden)

// Serielle Schnittstelleneinstellungen
constexpr uint8_t SERIAL_RETRY_TIME = 3;  // Versuche zur Herstellung einer seriellen Verbindung

// ThingsBoard-Einstellungen
#define ENCRYPTED false                                           // Kommunikation verschlüsselt?
constexpr char *TOKEN = "n61OedkJQgV1xS5A7g33";                   // TOKEN
constexpr char *THINGSBOARD_SERVER = "192.168.179.37";             // Server-IP-Adresse
constexpr uint16_t THINGSBOARD_PORT = ENCRYPTED ? 8883U : 1883U;  // Server-Port, abhängig von der Verschlüsselung
constexpr uint16_t MAX_MESSAGE_SIZE = 256U;                       // Maximale Nachrichtengröße
// ThingsBoard Gemeinsameattribute-Einstellungen
constexpr size_t ATTRIBUTE_COUNT = 14U;  // Anzahl der gemeinsamen Attribute [Achtung: Wenn die Anzahl der zu abonnierenden gemeinsamen Attribute geändert werden soll, müssen auch die Inhalte der Arrays SHARED_ATTRIBUTES und SUBSCRIBED_SHARED_ATTRIBUTES angepasst werden]
/* !!! Achtung: Weitere Einstellungen für die ThingsBoard-Gemeinsame-Attribute finden Sie im Abschnitt „ThingsBoard Gemeinsame Attribute Einzeleinstellungen“ weiter unten !!! */

// TinyML-Einstellungen
#define SERIAL_PRINT_ROUND true              // Enddaten jeder Runde drucken?
#define SERIAL_PRINT_SAMPLE true             // Jedes Sample drucken?
constexpr uint8_t SAMPLE_NUMBER = 4;         // Anzahl der Samples pro Runde
constexpr uint16_t SAMPLE_TIME_FRAME = 375;  // Sampling-Intervall in ms
float CONFIDENCE_threshold = 0.80;           // Globale Variable, Schwellenwert für die Konfidenz

// Einstellungen des Spannungssensors (Teiler)
constexpr float R1 = 30000.0f;                      // Widerstand R1 beträgt 30 kOhm
constexpr float R2 = 7500.0f;                       // Widerstand R2 beträgt 7,5 kOhm
constexpr float ADC_OFFSET = 0.02;                  // Manuelle Kalibrierung der ADC-Spannung
constexpr float VOL_OFFSET = 0.00;                  // Manuelle Kompensation der Batteriespannung
constexpr uint32_t BAT_CHECK_PERIOD_MS = 5 * 1000;  // Überprüfungsintervall der Batterie (Millisekunden), standardmäßig 30 Sekunden
constexpr uint32_t ADC_SAMPLING_FREQ = 1000;        // ADC-Samplingfrequenz
constexpr uint16_t CONVERSIONS_PER_PIN = 50;        // Anzahl der Umwandlungen pro Pin zur Mittelwertberechnung
// Einstellung der Batterie-Kapazitätsschätzung
constexpr float BATTERY_MAX_VOLTAGE = 9.0;           // Maximale Spannung einer 9V-Batterie
constexpr float BATTERY_MIN_VOLTAGE = 5.4;           // Minimale Betriebsspannung einer 9V-Batterie
constexpr float BATTERY_CONNECTION_MIN_THRESHOLD = 3.0;  // Annahme: Bei weniger als 3V gilt die Batterie als nicht verbunden
constexpr float BATTERY_CONNECTION_MAX_THRESHOLD = 11.0;  // Annahme: Bei mehr als 11V gilt die Batterie als nicht verbunden
#define TEMP_COMPENSATION true                       // Bei der Spannungsberechnung Temperaturkompensation verwenden


/* Hardware-Definitionen ------------------------------------------------------------------*/

//// Pin-Definitionen
// Gassensor
#define MICS_SDA_PIN 6
#define MICS_SCL_PIN 7
// Externe RTC
#define RTC_SDA_PIN 6
#define RTC_SCL_PIN 7
// OLED
#define OLED_CS_PIN 18
#define OLED_DC_PIN 16
// SD-Karte
#define SD_CS_PIN 22
// Akustisch-optischer Alarm
#define GREENLED_PIN 1
#define REDLED_PIN 8
#define BUZZER_PIN 10
// Tasten
#define BUTTON_SELECT 3
#define BUTTON_LEFT 2
#define BUTTON_RIGHT 11
// Spannungssensor
#define BATTERY_VOLTAGE_PIN 0  // ADC1_CHANNEL_0

//// Adressdefinitionen
// Gassensor
#define Mics_I2C_ADDRESS MICS_ADDRESS_0  // MICS_ADDRESS_0 == 0x75
DFRobot_MICS_I2C mics(&Wire, Mics_I2C_ADDRESS);

// Zeitbezogene Objekte
RTC_DS3231 outRTC;
ESP32Time inRTC;

// Anzeigeobjekte
SSD_13XX oled = SSD_13XX(OLED_CS_PIN, OLED_DC_PIN);

// ThingsBoard-bezogen
#if ENCRYPTED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif
Arduino_MQTT_Client mqttClient(espClient);
// ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);  // Verwenden Sie dies, wenn Sie abonnierte gemeinsame Attribute benötigen
ThingsBoardSized<Default_Fields_Amount, Default_Subscriptions_Amount, ATTRIBUTE_COUNT> tb(mqttClient, MAX_MESSAGE_SIZE);  // ThingsBoard-Instanz initialisieren mit der maximal benötigten Puffergröße


/* Globale Konstanten ------------------------------------------------------------------*/

// Gaskonstanten
constexpr char *GAS_NAME[] = { "CH4", "C2H5OH", "H2", "NH3", "CO", "NO2" };  // Namen der Gase
constexpr uint8_t GAS_TYPE[] = { CH4, C2H5OH, H2, NH3, CO, NO2 };            // Gaskategorien (Spezifisch für MiCS-Bibliotheksfunktionen, erwartet uint8_t Typ)

// Zeitformate
constexpr char TIME_FORMAT[10] = "hh:mm:ss";  // Format: Stunden, Minuten, Sekunden
constexpr char HOUR_FORMAT[4] = "hh";         // Format: Stunden
constexpr char MINUTE_FORMAT[4] = "mm";       // Format: Minuten
constexpr char SECOND_FORMAT[4] = "ss";       // Format: Sekunden

// NTP-Serveranzahl
constexpr uint8_t NTP_SERVER_COUNT = sizeof(NTP_SERVER) / sizeof(NTP_SERVER[0]);  // Berechnet die Anzahl der NTP-Server automatisch

// Menüanzahl
constexpr uint8_t NUM_EXTRA_SETTING_ITEM = 2;                                                            // Anzahl der zusätzlichen Menüpunkte
constexpr uint8_t NUM_SETTING_ITEM = (sizeof(GAS_NAME) / sizeof(GAS_NAME[0])) + NUM_EXTRA_SETTING_ITEM;  // Berechnet die Gesamtanzahl der Menüpunkte (Gasgrenzwerte + zusätzliche Menüpunkte)

// Koordinatensystemparameter
constexpr int8_t X_OFFSET = 0;
constexpr int8_t Y_OFFSET = 1;
constexpr uint8_t COORD_WIDTH = 60;
constexpr uint8_t COORD_HEIGHT = 40;
constexpr int8_t X_GAP = 4;  // Horizontaler Abstand zwischen den Koordinatensystemen: 4 Pixel
constexpr int8_t Y_GAP = 3;  // Vertikaler Abstand zwischen den Koordinatensystemen: 3 Pixel

// Kurvendarstellungsparameter
constexpr int8_t X_OFFSET_CURVE = X_OFFSET + 1;
constexpr int8_t Y_OFFSET_CURVE = Y_OFFSET - 1;
constexpr uint8_t NUM_COORD_SYSTEM = 6;  // Anzahl der Koordinatensysteme

// TinyML-Parameter
constexpr float ML_DEFAULT_MIN = 50000.00;  // Standardminimumwert

// Spannungssensorparameter
constexpr float VOLTAGE_DIVISION_FACTOR = R2 / (R1 + R2);               // Berechnung des Spannungsteilers
uint8_t ADC_PINS[] = { BATTERY_VOLTAGE_PIN };                           // ADC-Pin-Array, hier nur ein Pin verwendet
constexpr uint8_t ADC_PINS_COUNT = sizeof(ADC_PINS) / sizeof(uint8_t);  // Berechnet die Anzahl der Elemente im ADC-Pin-Array
// Berechnung der Batterie-Spannungs-Abschnittspunkteinstellungen
constexpr float V100 = BATTERY_MAX_VOLTAGE;
constexpr float V90 = BATTERY_MAX_VOLTAGE - 0.2;
constexpr float V70 = BATTERY_MAX_VOLTAGE - 0.6;
constexpr float V40 = BATTERY_MAX_VOLTAGE - 1.4;
constexpr float V20 = BATTERY_MIN_VOLTAGE + 1.6;
constexpr float V0 = BATTERY_MIN_VOLTAGE;
constexpr float V90_100_RATIO = (100.0 - 90.0) / (V100 - V90);
constexpr float V70_90_RATIO = (90.0 - 70.0) / (V90 - V70);
constexpr float V40_70_RATIO = (70.0 - 40.0) / (V70 - V40);
constexpr float V20_40_RATIO = (40.0 - 20.0) / (V40 - V20);
constexpr float V0_20_RATIO = (20.0 - 0.0) / (V20 - V0);

// Thingsboard
#if ENCRYPTED
/* Siehe https://comodosslstore.com/resources/what-is-a-root-ca-certificate-and-how-do-i-download-it/
um das Root-Zertifikat des Servers zu erhalten, mit dem eine sichere Verbindung hergestellt werden soll,
dies ändert sich je nach Website. */
constexpr char ROOT_CERT[] = R"(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)";
#endif


/* Datenstrukturen ------------------------------------------------------------------*/

//// Enumerations
// Tastenzustände
enum ButtonState {
  IDLE,          // Ruhemodus
  PRESSED,       // Gedrückt
  LONG_PRESSED,  // Lang gedrückt
  HELD           // Gehalten
};

// Benutzeroberflächenstatus
enum UIState {
  MAIN_UI,       // Hauptbenutzeroberfläche
  SECONDARY_UI,  // Nebenbenutzeroberfläche
  SETTINGS_UI    // Einstellungen-Oberfläche
};

// Stromversorgungsstatus
enum PowerState {
  POWER_NONE,    // Keine Stromversorgung
  POWER_USB,     // USB-Stromversorgung
  POWER_BATTERY  // Batteriebetrieben
};

//// Strukturen
// Einstellungsobjekt
struct SettingItem {
  const char *name;
  float minValue;
  float maxValue;
  float threshold;
};

// Tastenstruktur
struct Button {
  uint8_t pin;             // Pin, an den die Taste angeschlossen ist
  ButtonState state;       // Aktueller Zustand der Taste
  uint32_t lastPressTime;  // Zeitpunkt des letzten Drückens
  uint32_t lastHoldTime;   // Zeitpunkt des letzten Haltens
};

// Koordinatensystem: Struktur für die Historie der Gaskonzentrationen jedes Gases
struct GasHistory {
  std::vector<float> concentrations;
  float minConcentration;
  float maxConcentration;
};

// Definition der ThingsBoard-Gemeinsame-Attribute-Struktur
struct SharedAttribute {
  const char *name;
  void *value;
  void (*updateFunc)(void *, JsonVariantConst);
};

// TinyML-Datenstruktur
struct MLGasData {
  float sum;
  float min;
  float max;
  float avg;
};


/* Globale Variablen ------------------------------------------------------------------*/

// Gaskonzentrationen
float gasConcentrations[6] = { 0, 0, 0, 0, 0, 0 };                              // Aktuelle Konzentrationen der sechs Gase
float lastGasConcentrations[6] = { 50000, 50000, 50000, 50000, 50000, 50000 };  // Letzte Konzentrationen der sechs Gase

// Gefilterte Pufferdaten
float gasBuffer[6][WMA_WINDOW_SIZE];        // Erstelle einen Ringpuffer für jeden Gastyp
int bufferIndex[6] = { 0, 0, 0, 0, 0, 0 };  // Der aktuelle Index jedes Puffers

// Konzentrationsüberprüfung
int32_t gasAboveThresholdCount[6] = { 0, 0, 0, 0, 0, 0 };  // Anzahl der Male, die jedes Gas den Schwellenwert überschritten hat
int32_t gasBelowThresholdCount[6] = { 0, 0, 0, 0, 0, 0 };  // Anzahl der Male, die jedes Gas unter dem Schwellenwert blieb

// Akustisch-optischer Alarm
uint32_t alarmCurrentMillis = 0;   // Aktuelle Zeit in Millisekunden
uint32_t alarmPreviousMillis = 0;  // Zeit des letzten Wechsels in Millisekunden
uint32_t alarmEndMillis = 0;       // Zeitpunkt des Alarmendes in Millisekunden

// Tastenstatus
Button buttons[3] = {
  { BUTTON_SELECT, IDLE, 0, 0 },
  { BUTTON_LEFT, IDLE, 0, 0 },
  { BUTTON_RIGHT, IDLE, 0, 0 }
};

// Benutzeroberflächenstatus
UIState currentUI = MAIN_UI;
UIState lastUI = MAIN_UI;

// Stromversorgungsicon
PowerState currentPowerIcon = POWER_NONE;  // Aktueller Status des Stromversorgungsicons
int8_t lastFillHeight = -1;                // Füllhöhe des letzten Batterieicons

// Hauptoberfläche
uint32_t lastUpdateTimestamp = 0;  // Zeitstempel des letzten Gaswerte-Updates
uint16_t colorGasData = WHITE;
uint16_t colorStatusBar = BLUE;
uint16_t lastColorStatusBar = BLACK;

// Einstellungsoberfläche
SettingItem settings[6] = {
  { "CH4", 1000.00, 25000.00, 10000.00 },  // CH4: Name, Mindestwert, Höchstwert, Schwellenwert
  { "C2H5OH", 10.00, 500.00, 250.00 },     // C2H5OH: Name, Mindestwert, Höchstwert, Schwellenwert
  { "H2", 1.00, 1000.00, 500.00 },         // H2: Name, Mindestwert, Höchstwert, Schwellenwert
  { "NH3", 1.00, 500.00, 150.00 },         // NH3: Name, Mindestwert, Höchstwert, Schwellenwert
  { "CO", 1.00, 1000.00, 200.00 },         // CO: Name, Mindestwert, Höchstwert, Schwellenwert
  { "NO2", 0.10, 10.00, 5.00 }             // NO2: Name, Mindestwert, Höchstwert, Schwellenwert
};

float lastThresholds[6] = { 50000, 50000, 50000, 50000, 50000, 50000 };  // Letzte Schwellenwerte
float tempThresholds[6] = { 50000, 50000, 50000, 50000, 50000, 50000 };  // Temporäre Schwellenwerte in der Einstellungsoberfläche
uint8_t currentSettingItem = 0;                                          // Aktueller Index des Einstellungselements
uint8_t lastSettingItem = currentSettingItem;                            // Letzter Index des Einstellungselements

uint8_t updateSettingsUiCallCounter = 0;  // Zähler für Funktionsaufrufe in der Einstellungsoberfläche
bool blinkState = false;                  // Variable zur Steuerung des Blinkzustands in der Einstellungsoberfläche

// Koordinatensystem
GasHistory gasHistories[6];  // Array zur Speicherung der historischen Gaskonzentrationen jedes Gases

// SD-Karte
File logFile;
uint32_t startTime;  // Auf der ESP32-Plattform entspricht uint32_t einem unsigned long

// Zeit- und Temperaturbezogene Variablen
char outRtcTime[10] = "hh:mm:ss";      // Format: Stunden, Minuten, Sekunden
char outRtcHour[4] = "hh";             // Format: Stunden
char outRtcMinute[4] = "mm";           // Format: Minuten
char outRtcSecond[4] = "ss";           // Format: Sekunden
char lastOutRtcTime[10] = "xx:xx:xx";  // Format: Stunden, Minuten, Sekunden
char lastOutRtcHour[4] = "xx";         // Format: Stunden
char lastOutRtcMinute[4] = "xx";       // Format: Minuten
char lastOutRtcSecond[4] = "xx";       // Format: Sekunden
float outRtcTemperature = 0.0;
float lastOutRtcTemperature = 999.9;

// WLAN-Status
uint8_t wifiStatus = WL_IDLE_STATUS;  // WL_IDLE_STATUS entspricht der Zahl 0
uint8_t lastWiFiStatus = WL_IDLE_STATUS;

// TinyML-bezogen
MLGasData mlGasData[6] = {
  { 0.00, ML_DEFAULT_MIN, 0.00, 0.00 },  // CH4: Summe, Min, Max, Durchschnitt
  { 0.00, ML_DEFAULT_MIN, 0.00, 0.00 },  // C2H5OH: Summe, Min, Max, Durchschnitt
  { 0.00, ML_DEFAULT_MIN, 0.00, 0.00 },  // H2: Summe, Min, Max, Durchschnitt
  { 0.00, ML_DEFAULT_MIN, 0.00, 0.00 },  // NH3: Summe, Min, Max, Durchschnitt
  { 0.00, ML_DEFAULT_MIN, 0.00, 0.00 },  // CO: Summe, Min, Max, Durchschnitt
  { 0.00, ML_DEFAULT_MIN, 0.00, 0.00 }   // NO2: Summe, Min, Max, Durchschnitt
};

float myScoreHarmful = 0;
float myScoreRegular = 0;

float features[18];  // Array zur Definition der Merkmale

uint8_t sampleCounter = 0;  // Zähler für die aktuelle Sampling-Runde

String line1 = "Alle Werte";  // Inhalt für TinyML-Bildschirmausgabe
String line2 = "sind 0";
String line3 = "Keine Gase";
String line4 = "erkannt";
String lastLine1 = "";
String lastLine2 = "";
String lastLine3 = "";
String lastLine4 = "";
uint16_t colorline1 = GREEN;
uint16_t colorline2 = GREEN;
uint16_t colorline3 = GREEN;
uint16_t colorline4 = GREEN;

// Spannungssensor
adc_continuous_data_t *adcResult = NULL;  // Deklariert einen Zeiger auf eine Struktur für kontinuierliche ADC-Daten, um die Ergebnisse der ADC-Lesung zu speichern
float capacityPercentage = 101.0;         // Aktueller Batteriekapazitätsprozentsatz



/* FreeRTOS-Aufgaben und Mutex ------------------------------------------------------------------*/

// Aufgaben-Handles
TaskHandle_t HandleButtonsTask_handle = NULL;
TaskHandle_t ReadGasTask_handle = NULL;
TaskHandle_t AlarmTask_handle = NULL;
TaskHandle_t UpdateUiTask_handle = NULL;
TaskHandle_t CheckGasTask_handle = NULL;
TaskHandle_t LogTask_handle = NULL;
TaskHandle_t SerialPrintTask_handle = NULL;
TaskHandle_t ManageSdSpaceTask_handle = NULL;
TaskHandle_t LoadingScreenTask_handle = NULL;
TaskHandle_t ThingsboardTask_handle = NULL;
TaskHandle_t TinyMLTask_handle = NULL;
TaskHandle_t DaemonTask_handle = NULL;
TaskHandle_t GetTimeAndTempTask_handle = NULL;
TaskHandle_t CheckBatteryTask_handle = NULL;

// Timer-Handles
TimerHandle_t ManageSdSpaceTimer_handle = NULL;
TimerHandle_t CheckBatteryTimer_handle = NULL;

// Mutex (Mutual Exclusion) Handles
SemaphoreHandle_t i2cMutex_handle;
SemaphoreHandle_t spiMutex_handle;


/* Flags ------------------------------------------------------------------*/

// Alarmbezogen
bool isTotalAlarmState = false;                                          // Speichert den Gesamtalarmstatus
bool isLastTotalAlarmState = false;                                      // Speichert den vorherigen Gesamtalarmstatus
bool isAlarmState[6] = { false, false, false, false, false, false };     // Array zur Speicherung des Alarmstatus jedes Gases
bool isTriangleDrawn[6] = { false, false, false, false, false, false };  // Array zur Speicherung, ob das rote Dreieck für jedes Gas bereits gezeichnet wurde

// Verbindungsstatus
bool isOutRtcConnected = false;     // Verbindungsstatus der externen RTC
bool isWifiConnected = false;       // WLAN-Verbindungsstatus
bool isWifiEnabled = true;          // WLAN-Funktionsstatus
bool isGasSensorConnected = false;  // Gassensor-Verbindungsstatus
bool isSdConnected = false;         // SD-Karten-Verbindungsstatus
bool isSerialConnected = false;     // Serielle Schnittstellenverbindung
bool isBatteryConnected = false;    // Batterieverbindungsstatus

// Zeitbezogene Zustände
bool isTimeSynced = false;  // Zustand der Zeitsynchronisation mit NTP, wird im Programm momentan nicht verwendet

// Zustände der Benutzeroberflächenelemente
bool isForceRefresh = false;          // Hauptoberfläche: Erzwingt eine Aktualisierung
bool isNotifyConfigSaved = false;     // Einstellungsoberfläche: Stoppt die Aktualisierung des Auswahlrahmens bei einem Hinweis
bool isStatusBarRed = false;          // Statusleiste: Farbe
bool isStatusBarYellow = false;       // Statusleiste: Farbe
bool isStatusBarGreen = false;        // Statusleiste: Farbe
bool isNoRtcConnectionDrawn = false;  // Statusleiste: Zeichnet einen Hinweis, wenn keine RTC-Verbindung besteht
bool isMLIconDrawn = false;           // Statusleiste: ML-Symbol gezeichnet
bool isAlarmIconDrawn = false;        // Statusleiste: Alarm-Symbol gezeichnet
bool isWifiIconDrawn = false;         // Statusleiste: WLAN-Symbol gezeichnet
bool isSdIconDrawn = false;           // Statusleiste: SD-Karten-Symbol gezeichnet


// FreeRTOS-Aufgabensteuerung
bool isAlarmTaskEnabled = true;       // Schalter für die akustisch-optische Alarm-Aufgabe
bool isLastAlarmTaskEnabled = false;  // Zustand des letzten Schalters für die akustisch-optische Alarm-Aufgabe
bool isTempAlarmTaskEnabled;          // Temporärer Schalter
bool isTinyMLTaskEnabled = false;     // Schalter für die TinyML-Aufgabe
bool isLastTinyMLTaskEnabled = true;  // Zustand des letzten Schalters für die TinyML-Aufgabe
bool isTempTinyMLTaskEnabled;         // Temporärer Schalter

// Thingsboard-Aufgabensteuerung
bool isSubscribed = false;  // Abonnementstatus der gemeinsamen Eigenschaften




/* ========================================================== Bereich für Initialisierungsfunktionen ========================================================== */


/* Initialisierung des OLED-Displays ------------------------------------------------------------------*/
void setupOLED() {
  oled.begin();
  oled.setBrightness(15);
  oled.setRotation(0);
  oled.clearScreen();
}



/* Verbindung über serielle Schnittstelle ------------------------------------------------------------------*/
bool connectSerialWithRetry() {
  uint8_t retries = SERIAL_RETRY_TIME;
  while (retries > 0) {
    Serial.begin(115200);  // Versuch, die serielle Kommunikation zu starten
    vTaskDelay(pdMS_TO_TICKS(750));
    if (Serial) {                // Überprüfung, ob die Verbindung erfolgreich war
      isSerialConnected = true;  // Bei erfolgreicher Verbindung Flag auf true setzen
      return true;               // Funktion verlassen
    }
    retries--;  // Bei Fehlschlag, die Anzahl der Wiederholungen verringern
  }
  isSerialConnected = false;  // Wenn drei Versuche fehlschlagen, Flag auf false setzen
  return false;
}



/* Gas Sensor Initialisierung ------------------------------------------------------------------*/
void setupMiCS() {

  Wire.begin(MICS_SDA_PIN, MICS_SCL_PIN);

  Serial.print("Connecting to Gas sensor ");
  uint8_t retryCount = 0;
  while (!mics.begin() && retryCount < 10) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    retryCount++;
  }
  if (mics.begin()) {
    isGasSensorConnected = true;
    Serial.println("  [OK]");
  } else {
    isGasSensorConnected = false;
    Serial.println("  [Failed]");
    Serial.println("[CRITICAL] Gas sensor NOT connected, please check the connection!");
    // 重启ESP系统
    Serial.printf("[WARN] System Restarting!\n\n\n\n\n");
    ESP.restart();
    return;
  }

  uint8_t mode = mics.getPowerState();
  if (mode == SLEEP_MODE) {
    mics.wakeUpMode();
    Serial.println("Wake up sensor success");
  } else {
    Serial.println("The sensor is waking up");
  }
  Serial.println();
}



/* Kalibrierungsladebildschirm ------------------------------------------------------------------*/
void loadingScreen() {
  int totalCalibrationTime = CALIBRATION_TIME * 60;  // Gesamtkalibrierungszeit in Sekunden berechnen
  int remainingTime = totalCalibrationTime;          // Restzeit initialisieren
  int currentMillis;
  int previousMillis;
  int progress;

  // Titel zeichnen
  oled.setTextScale(2);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);
  oled.print("Kalibrierung...");

  // Text und Rahmen für Hardwarestatus zeichnen
  oled.setTextScale(1);
  oled.setCursor(0, 87);
  oled.print("Gassensor");
  oled.setCursor(70, 87);
  oled.print("WiFi");
  oled.setCursor(0, 112);
  oled.print("Ext. RTC");
  oled.setCursor(70, 112);
  oled.print("MicroSD");
  oled.drawRect(50, 85, 10, 10, WHITE);
  oled.drawRect(108, 85, 10, 10, WHITE);
  oled.drawRect(50, 110, 10, 10, WHITE);
  oled.drawRect(108, 110, 10, 10, WHITE);

  // Gassensor-Verbindungsstatus aktualisieren
  if (isGasSensorConnected) {
    oled.fillRect(52, 87, 6, 6, GREEN);  // Hardwarestatus auf dem Ladebildschirm aktualisieren
  } else {
    oled.fillRect(52, 87, 6, 6, RED);  // Hardwarestatus auf dem Ladebildschirm aktualisieren
  }

  // Fortschrittsbalken initialisieren (Rahmen zeichnen)
  int progressBarWidth = 120;  // Breite des Fortschrittsbalkens
  int progressBarHeight = 20;  // Höhe des Fortschrittsbalkens
  int progressBarX = 0;        // X-Startposition des Fortschrittsbalkens
  int progressBarY = 50;       // Y-Startposition des Fortschrittsbalkens
  int borderWidth = 2;         // Abstand des Balkens zum Rahmen
  oled.drawRect(progressBarX, progressBarY, progressBarWidth, progressBarHeight, WHITE);


  Serial.println("[INFO] Calibration Started");  // Kalibrierungsstart im seriellen Monitor ausgeben

  // Schleife zur Überwachung der Kalibrierung, aktualisiert den Fortschrittsbalken jede Sekunde und gibt alle 10 Sekunden die verbleibende Zeit aus
  while (!mics.warmUpTime(CALIBRATION_TIME)) {
    uint32_t currentMillis = millis();  // Aktuelle Millisekunden abrufen

    // Bildschirminhalte alle 1 Sekunde aktualisieren
    if (currentMillis - previousMillis >= 1000) {
      previousMillis = currentMillis;

      // Alte Zeit überdecken und neue Restzeit zeichnen
      oled.fillRect(70, 30, 30, 7, BLACK);
      oled.setTextScale(1);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 30);
      oled.printf("Verbleibende Zeit: %d s", remainingTime);

      progress = (totalCalibrationTime - remainingTime + 1) * (progressBarWidth - borderWidth * 2) / totalCalibrationTime;          // Länge des Fortschrittsbalkens berechnen
      oled.fillRect(progressBarX + borderWidth, progressBarY + borderWidth, progress, progressBarHeight - borderWidth * 2, WHITE);  // Neuen Fortschrittsbalken zeichnen

      // Verbleibende Zeit im seriellen Monitor ausgeben, alle 10 Sekunden
      if (remainingTime % 10 == 0) {
        Serial.printf("[INFO] Please wait, remaining time: %ds\n", remainingTime);
      }

      remainingTime--;  // Restzeit um 1 Sekunde verringern
    }

    vTaskDelay(pdMS_TO_TICKS(1));  // Kurze Verzögerung einfügen, um CPU-Auslastung zu reduzieren
  }

  Serial.println("[INFO] Calibration Done");  // Kalibrierungsende im seriellen Monitor ausgeben
  Serial.println();

  createTasks();  // Nach Abschluss der Kalibrierung Aufgaben erstellen
}


/* Ausführen der restlichen Systeminitialisierungsfunktionen und Erstellen von persistenten Aufgaben ------------------------------------------------------------------*/
void createTasks() {
  // Systemstartzeit für SD-Kartenprotokollierung abrufen
  startTime = millis();
  // Hauptbildschirm-Oberfläche einmalig initialisieren
  oled.clearScreen();
  oledDrawMainUI();

  // Aufgabe erstellen: Gaskonzentration vom Sensor lesen
  xTaskCreate(
    ReadGasTask_func,    // Aufgabenfunktion
    "Read Gas Sensor",   // Aufgabenname
    2048,                // Stapelgröße
    NULL,                // Aufgabenparameter
    4,                   // Priorität (je höher die Zahl, desto höher die Priorität)
    &ReadGasTask_handle  // Aufgabenhandle
  );

  // Aufgabe erstellen: Überprüfen, ob die Konzentration überschritten ist
  xTaskCreate(
    CheckGasTask_func,           // Aufgabenfunktion
    "Check Gas Concentrations",  // Aufgabenname
    2048,                        // Stapelgröße
    NULL,                        // Aufgabenparameter
    4,                           // Priorität
    &CheckGasTask_handle         // Aufgabenhandle
  );

  // Aufgabe erstellen: Akustischer und visueller Alarm
  xTaskCreate(
    AlarmTask_func,       // Aufgabenfunktion
    "Check Alarm State",  // Aufgabenname
    2048,                 // Stapelgröße
    NULL,                 // Aufgabenparameter
    4,                    // Priorität
    &AlarmTask_handle     // Aufgabenhandle
  );

  // Aufgabe erstellen: Bildschirminhalt aktualisieren
  xTaskCreate(
    UpdateUiTask_func,    // Aufgabenfunktion
    "Update UI",          // Aufgabenname
    8192,                 // Stapelgröße
    NULL,                 // Aufgabenparameter
    5,                    // Priorität
    &UpdateUiTask_handle  // Aufgabenhandle
  );

  // Aufgabe erstellen: Tasten verarbeiten
  xTaskCreate(
    HandleButtonsTask_func,    // Aufgabenfunktion
    "Handle Buttons",          // Aufgabenname
    4096,                      // Stapelgröße
    NULL,                      // Aufgabenparameter
    3,                         // Priorität
    &HandleButtonsTask_handle  // Aufgabenhandle
  );

  // Wenn SD-Karte bei Initialisierung verbunden ist, Aufgabe erstellen: SD-Kartenprotokollierung
  if (isSdConnected) {
    xTaskCreate(
      LogTask_func,    // Aufgabenfunktion
      "Log to SD",     // Aufgabenname
      4096,            // Stapelgröße
      NULL,            // Aufgabenparameter
      2,               // Priorität
      &LogTask_handle  // Aufgabenhandle
    );
  }

  // Wenn serielle Verbindung bei Initialisierung vorhanden, Aufgabe erstellen: Serielle Protokollausgabe
  if (isSerialConnected) {
    xTaskCreate(
      SerialPrintTask_func,
      "Serial Print Output",   // Aufgabenname
      2048,                    // Stapelgröße
      NULL,                    // Aufgabenparameter
      1,                       // Priorität
      &SerialPrintTask_handle  // Aufgabenhandle
    );
  }

  // Wenn WLAN bei Initialisierung verbunden, Aufgabe erstellen: Synchronisation mit Thingsboard-Plattform
  if (isWifiConnected) {
    xTaskCreate(
      ThingsboardTask_func,     // Aufgabenfunktion
      "Sync with Thingsboard",  // Aufgabenname
      8192,                     // Stapelgröße
      NULL,                     // Aufgabenparameter
      2,                        // Priorität
      &ThingsboardTask_handle   // Aufgabenhandle
    );
  }

  // Aufgabe erstellen: Daemon
  xTaskCreate(
    DaemonTask_func,    // Aufgabenfunktion
    "Daemon",           // Aufgabenname
    2048,               // Stapelgröße
    NULL,               // Aufgabenparameter
    6,                  // Priorität, sollte höher sein als die zu kontrollierenden Aufgaben
    &DaemonTask_handle  // Aufgabenhandle
  );
}



/* Erstmalige Initialisierung und Verbindung mit WiFi sowie Überprüfung des Verbindungserfolgs ------------------------------------------------------------------*/
void initWiFi() {
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.print(WIFI_SSID);
  Serial.print(" ");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint8_t retryCount = 0;

  while (WiFi.status() != WL_CONNECTED && retryCount < WIFI_INIT_RETRY) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    isWifiConnected = true;
    oled.fillRect(110, 87, 6, 6, GREEN);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
    Serial.println("  [OK]");
    Serial.print("[INFO] LAN IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println();
#if ENCRYPTED
    espClient.setCACert(ROOT_CERT);
#endif

  } else {
    isWifiConnected = false;
    oled.fillRect(110, 87, 6, 6, RED);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
    Serial.println("  [Failed]");
    Serial.println();
    disableWiFi();  // WiFi-Funktion vollständig abschalten, um Strom zu sparen
  }
}


/* WiFi-Funktion vollständig ausschalten ------------------------------------------------------------------*/
void disableWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  isWifiEnabled = false;
  Serial.println("[INFO] WiFi disabled");
  Serial.println();
}



/* Initialisierung des externen RTC ------------------------------------------------------------------*/
void setupOutRTC() {

  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);

  Serial.print("Connecting to OutRTC ");
  uint8_t retryCount = 0;
  while (!outRTC.begin() && retryCount < 5) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.print(".");
    retryCount++;
  }
  if (outRTC.begin()) {
    isOutRtcConnected = true;
    oled.fillRect(52, 112, 6, 6, YELLOW);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
    Serial.println("  [OK]");
    Serial.println();
  } else {
    oled.fillRect(52, 112, 6, 6, RED);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
    isOutRtcConnected = false;
    Serial.println("  [Failed]");
    Serial.println();
    // Funktion verlassen, wenn die Verbindung zum externen RTC fehlschlägt
    return;
  }

  if (outRTC.lostPower()) {
    Serial.println("[WARN] OutRTC lost power, OutRTC reset to the date & time this sketch was compiled!");
    Serial.println();
    // Wenn die Uhrzeit auf einem neuen Gerät eingestellt werden muss oder nach einem Stromausfall,
    // setzt die folgende Zeile das RTC auf das Datum & die Uhrzeit, wann dieser Sketch kompiliert wurde
    outRTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Überprüfen der WiFi-Verbindung und Synchronisation der Zeit mit dem NTP-Server
  if (isWifiConnected) {
    syncTimeWithNTP();  // Synchronisieren der Zeit von einem NTP-Server mit dem internen RTC
  } else {
    Serial.println("[WARN] No WiFi connection, using unsynchronized OutRTC time");
    // Synchronisieren der Zeit vom externen RTC zum internen RTC
    syncInRTCWithOutRTC();  // Synchronisieren der Zeit vom externen RTC zum internen RTC
  }

  // Seriell die aktuelle Zeitinformation zur Überprüfung ausdrucken
  printTimeInfo();
}


/* Überprüfen, ob die Zeitsynchronisation erfolgreich war ------------------------------------------------------------------*/
bool checkTimeSync() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    if (timeinfo.tm_year >= (2024 - 1900)) {  // Überprüfungsbedingung: Jahr muss mindestens 2024 sein
      return true;
    }
  }
  return false;
}


/* Synchronisation der Zeit vom externen RTC zum internen RTC (diese Operation kann nicht direkt erfolgen, sie erfordert die Nutzung der Bibliothek ESP32Time)
Diese Funktion wird aufgerufen, wenn keine WiFi-Verbindung besteht oder keine NTP-Dienste verfügbar sind ------------------------------------------------------------------*/
void syncInRTCWithOutRTC() {
  DateTime now = outRTC.now();
  inRTC.setTime(now.unixtime());
  oled.fillRect(52, 112, 6, 6, YELLOW);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
}


/* Synchronisieren der Zeit vom NTP-Server zum internen RTC und dann zum externen RTC ------------------------------------------------------------------*/
void syncTimeWithNTP() {
  uint8_t currentNtpServerIndex = 0;
  uint8_t ntpRetryCount = 0;

  while (currentNtpServerIndex < NTP_SERVER_COUNT) {
    Serial.printf("Synchronizing InRTC time with NTP server %s ", NTP_SERVER[currentNtpServerIndex]);
    bool serverFailed = true;

    for (ntpRetryCount = 0; ntpRetryCount < NTP_MAX_RETRY; ntpRetryCount++) {
      configTime(UTC_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER[currentNtpServerIndex]);
      vTaskDelay(pdMS_TO_TICKS(1000));  // Warten von 1000 ms, um sicherzustellen, dass die Zeit synchronisiert wird
      if (checkTimeSync()) {
        Serial.println("  [OK]");
        serverFailed = false;
        syncOutRTCWithInRTC();  // Innen-RTC wurde mit NTP-Server synchronisiert, jetzt Zeit vom Innen-RTC zum Außen-RTC synchronisieren
        return;
      } else {
        Serial.print(".");
      }
    }

    if (serverFailed) {
      Serial.println("  [Failed]");
    }

    currentNtpServerIndex++;
  }

  // Wenn alle Synchronisierungsversuche fehlschlagen, wird die Zeit vom externen RTC als Grundlage verwendet
  if (currentNtpServerIndex == NTP_SERVER_COUNT && ntpRetryCount == NTP_MAX_RETRY) {
    Serial.println("[ERROR] Unable to connect to any NTP server, please check Internet connectivity or try different NTP server addresses!");
    Serial.println("[WARN] Using unsynchronized OutRTC time");
    // Zeit vom externen RTC zum internen RTC synchronisieren (diese Operation kann nicht direkt erfolgen, benötigt die Bibliothek ESP32Time)
    syncInRTCWithOutRTC();  // Zeit vom externen RTC zum internen RTC synchronisieren
  }
}


/* Synchronisieren der Zeit vom internen RTC zum externen RTC ------------------------------------------------------------------*/
void syncOutRTCWithInRTC() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[ERROR] Failed to obtain time from InRTC!");
    return;
  }
  outRTC.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
  oled.fillRect(52, 112, 6, 6, GREEN);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
  isTimeSynced = true;                  // Globale Variable - Zeit-Synchronisationsstatus auf true setzen
  Serial.println("[INFO] OutRTC time synchronized with InRTC");
}


/* Aktuelle Zeit und Temperatur ausgeben ------------------------------------------------------------------*/
void printTimeInfo() {

  DateTime now = outRTC.now();  // Aktuelle Zeitinformationen in 'now' speichern

  Serial.println();
  Serial.println("---------- OutRTC Time Info ----------");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(DAYS_OF_THE_WEEK[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  Serial.print("Temperature: ");
  Serial.print(outRTC.getTemperature());
  Serial.println(" ℃");
  Serial.println("--------------------------------------");
  Serial.println();
}



/* Initialisierung der SD-Kartenprotokollierung ------------------------------------------------------------------*/
void setupSDLogger() {
  SPI.begin(21, 20, 19, SD_CS_PIN);  // SCK, MISO, MOSI, SS (für SD)

  Serial.print("Connecting to MicroSD Module ");
  int retryCount = 0;
  while (!SD.begin(SD_CS_PIN) && retryCount < 5) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.print(".");
    retryCount++;
  }
  // Überprüfen, ob das SD-Modul verbunden ist
  if (SD.begin(SD_CS_PIN)) {
    oled.fillRect(110, 112, 6, 6, YELLOW);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
    Serial.println("  [OK]");
    Serial.println();
  } else {
    oled.fillRect(110, 112, 6, 6, RED);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
    Serial.println("  [Failed]");
    Serial.println("[ERROR] Card Mount Failed!");
    Serial.println();
    // Funktion verlassen, wenn die Verbindung zum SD-Modul fehlschlägt
    return;
  }

  // Versuchen, den SD-Kartentyp abzurufen, um zu überprüfen, ob die SD-Karte richtig angeschlossen ist
  uint8_t cardType = SD.cardType();
  if (cardType != CARD_NONE) {
    isSdConnected = true;
    oled.fillRect(110, 112, 6, 6, GREEN);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
    Serial.println("[INFO] SD card attached");
    Serial.println();
  } else {
    oled.fillRect(110, 112, 6, 6, RED);  // Aktualisieren des Hardwarestatus auf dem Ladebildschirm
    Serial.println("[ERROR] No SD card attached!");
    Serial.println();
    // Funktion verlassen, wenn die SD-Karte nicht erkannt wird
    return;
  }

  // SD-Karteninformationen ausgeben
  Serial.println();
  Serial.println("---------- MicroSD Info ------------");
  Serial.print("[INFO] SD-Kartentyp: ");
  switch (cardType) {
    case CARD_MMC: Serial.println("MMC"); break;
    case CARD_SD: Serial.println("SDSC"); break;
    case CARD_SDHC: Serial.println("SDHC"); break;
    default: Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("[INFO] SD Card Size: %lluMB\n", cardSize);
  Serial.println("------------------------------------");
  Serial.println();

  // Konfigurationsdatei von der SD-Karte lesen
  loadConfig();


  //// Aufgaben erstellen sowie zugehörige Timer
  // Aufgabe erstellen: Regelmäßige Überprüfung und Verwaltung des freien Speicherplatzes auf der SD-Karte
  xTaskCreate(
    ManageSdSpaceTask_func,    // Aufgabenfunktion
    "Check SD Free Space",     // Aufgabenname
    4096,                      // Stapelgröße (Bytes), bei Bedarf anpassen
    NULL,                      // Aufgabenparameter
    1,                         // Priorität (höhere Zahlen bedeuten höhere Priorität)
    &ManageSdSpaceTask_handle  // Aufgaben-Handle
  );
  // Timer erstellen
  ManageSdSpaceTimer_handle = xTimerCreate("SDCardTimer", pdMS_TO_TICKS(SD_CHECK_PERIOD_MS), pdTRUE, 0, sdCardTimerCallback);
  // Timer starten
  xTimerStart(ManageSdSpaceTimer_handle, 0);


  // Neue Protokolldatei erstellen
  char logFileName[40];
  getNewLogFileName(logFileName);
  logFile = SD.open(logFileName, FILE_WRITE);
  if (!logFile) {
    Serial.println("[ERROR] Failed to create log file!");
    return;
  }

  // Wenn das RTC-Modul verbunden ist, Zeitinformationen am Anfang der neuen Protokolldatei schreiben
  if (outRTC.begin()) {
    // Zeitinformationen vom externen RTC abrufen
    DateTime now = outRTC.now();  // Aktuelle Zeitinformationen in 'now' speichern

    // Zeit- und Datumsinformationen beim Systemstart in die Protokolldatei schreiben
    logFile.println("System gestartet um:");
    logFile.print(now.day(), DEC);
    logFile.print('.');
    logFile.print(now.month(), DEC);
    logFile.print('.');
    logFile.print(now.year(), DEC);
    logFile.print(" (");
    logFile.print(DAYS_OF_THE_WEEK[now.dayOfTheWeek()]);
    logFile.print(") ");
    logFile.print(outRtcTime);
    logFile.println();
    // Temperatur beim Systemstart in die Protokolldatei schreiben
    logFile.print("Temperatur: ");
    logFile.print(outRtcTemperature);
    logFile.println(" ℃");
  } else {
    // Behandlung, falls das RTC-Modul nicht verbunden ist
    logFile.println("RTC-Modul NICHT verbunden!");
  }

  // Kopfzeile für die neue Protokolldatei schreiben
  logFile.println();
  logFile.print("Betriebszeit | Echtzeit | Temperatur ||");
  for (int i = 0; i < 6; i++) {
    logFile.print(" ");
    logFile.print(GAS_NAME[i]);
    logFile.print(" |");
  }
  logFile.println();  // Zeilenumbruch nach der Kopfzeile

  // Serielle Ausgabe der Protokollinformationen
  Serial.printf("[INFO] Logging to: %s\n", logFileName);
  Serial.println();
}


/* Generieren eines neuen Logdateinamens ------------------------------------------------------------------*/
void getNewLogFileName(char *fileName) {
  if (outRTC.begin()) {
    DateTime now = outRTC.now();
    sprintf(fileName, "/%04d-%02d-%02d_%02d-%02d-%02d%s",
            now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), LOG_FILE_EXTENSION);
  } else {
    static int logFileIndex = 1;
    do {
      sprintf(fileName, "/UnknownStartTime_%d%s", logFileIndex++, LOG_FILE_EXTENSION);
    } while (SD.exists(fileName));
  }
}


/* Einstellungen von SD-Karte laden ------------------------------------------------------------------*/
void loadConfig() {
  // Überprüfen, ob die Konfigurationsdatei existiert
  if (!SD.exists(CONFIG_FILE_NAME)) {
    Serial.println("[WARN] Config file does not exist, creating with default settings");
    saveConfigToSD();  // Standardkonfigurationsdatei erstellen
    return;
  }

  File configFile = SD.open(CONFIG_FILE_NAME, FILE_READ);
  if (!configFile) {
    Serial.println("[ERROR] Failed to open config file for reading");
    return;
  }

  // JSON-Dokument erstellen
  StaticJsonDocument<1024> doc;

  // JSON aus Datei in doc deserialisieren
  DeserializationError error = deserializeJson(doc, configFile);
  if (error) {
    Serial.println("[ERROR] Failed to read file, using default settings");
    configFile.close();
    return;
  }

  JsonArray configArray = doc["config"];

  for (int i = 0; i < 6; i++) {
    JsonObject configObject = configArray[i];
    // settings[i].name = configObject["name"];  // Name des Gases lesen
    settings[i].minValue = configObject["minValue"];
    settings[i].maxValue = configObject["maxValue"];
    settings[i].threshold = configObject["threshold"];
  }

  configFile.close();
  Serial.println("[INFO] Config loaded from SD");
  Serial.println();
}




/* ========================================================== Bereich für Funktionen für wiederholende Aufgaben ========================================================== */


/* Initialisierung des Gaspuffers für die Filterung ------------------------------------------------------------------*/
void initGasBuffers() {
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < WMA_WINDOW_SIZE; ++j) {
      gasBuffer[i][j] = 0.0;
    }
    bufferIndex[i] = 0;
  }
}


/* Anwendung des WMA-Filters ------------------------------------------------------------------*/
float applyWMA(float newValue, int gasType) {
  // Aktualisierung des Puffers
  gasBuffer[gasType][bufferIndex[gasType]] = newValue;
  bufferIndex[gasType] = (bufferIndex[gasType] + 1) % WMA_WINDOW_SIZE;

  // Berechnung des gewichteten Durchschnitts
  float sum = 0;
  for (int i = 0; i < WMA_WINDOW_SIZE; ++i) {
    int index = (bufferIndex[gasType] - i + WMA_WINDOW_SIZE) % WMA_WINDOW_SIZE;
    sum += gasBuffer[gasType][index] * WMA_WEIGHTS[i];
  }

  return sum;
}


/* Gasdaten vom Sensor erfassen, filtern und in das Array schreiben. Mit I2C-Mutex ------------------------------------------------------------------*/
void getGasData() {
#if USE_MUTEX
  if (xSemaphoreTake(i2cMutex_handle, portMAX_DELAY) == pdTRUE) {  // I2C-Mutex übernehmen
#endif

    for (int i = 0; i < 6; ++i) {
      float rawValue = mics.getGasData(GAS_TYPE[i]);
      gasConcentrations[i] = applyWMA(rawValue, i);
    }

#if USE_MUTEX
    xSemaphoreGive(i2cMutex_handle);  // I2C-Mutex freigeben
  }
#endif
}




/* Überprüfen, ob die Gaskonzentration die Schwellenwerte erreicht ------------------------------------------------------------------*/
void checkGasConcentration() {
  // Initialisiere den allgemeinen Alarmstatus auf false
  isTotalAlarmState = false;

  // Prüfe, ob die Konzentration jedes Gases den festgelegten Schwellenwert erreicht oder überschreitet
  for (int i = 0; i < 6; ++i) {
    if (gasConcentrations[i] >= settings[i].threshold) {
      gasAboveThresholdCount[i]++;
      gasBelowThresholdCount[i] = 0;  // Zähler für Werte unterhalb des Schwellenwerts zurücksetzen

      if (gasAboveThresholdCount[i] >= GAS_CHECK_time) {
        isAlarmState[i] = true;
      }
    } else {
      gasBelowThresholdCount[i]++;
      gasAboveThresholdCount[i] = 0;  // Zähler für Werte oberhalb des Schwellenwerts zurücksetzen

      if (gasBelowThresholdCount[i] >= GAS_CHECK_time) {
        isAlarmState[i] = false;
      }
    }

    // Wenn der Alarmstatus irgendeines Gases true ist, dann ist der allgemeine Alarmstatus ebenfalls true
    if (isAlarmState[i]) {
      isTotalAlarmState = true;
    }
  }

  // Überprüfe, ob der Alarmstatus aller Gase false ist
  if (!isTotalAlarmState) {
    bool allFalse = true;
    for (int i = 0; i < 6; ++i) {
      if (isAlarmState[i]) {
        allFalse = false;
        break;
      }
    }

    isTotalAlarmState = !allFalse;
  }
}




/* Akustisch-optischer Alarm ------------------------------------------------------------------*/
void checkTotalAlarmState() {
  alarmCurrentMillis = millis();

  if (isTotalAlarmState) {
    // Wenn der Gesamtalarmstatus wahr ist, beginne sofort mit dem Alarm
    alarmEndMillis = 0;  // Alarmende-Zeit zurücksetzen
    // Alle 500 Millisekunden den Status umschalten
    if (alarmCurrentMillis - alarmPreviousMillis >= ALARM_interval) {
      alarmPreviousMillis = alarmCurrentMillis;
      digitalWrite(REDLED_PIN, !digitalRead(REDLED_PIN));  // Rote LED umschalten
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));  // Summer umschalten
    }
    digitalWrite(GREENLED_PIN, LOW);  // Grüne LED ausschalten
  } else {
    // Wenn der Gesamtalarmstatus falsch wird, zeichne die Zeit auf, zu der der Alarm endete
    if (isLastTotalAlarmState && alarmEndMillis == 0) {
      alarmEndMillis = alarmCurrentMillis;
    }

    // Alarm deaktivieren, wenn die Zeit seit dem Alarmende mehr als 2 Sekunden beträgt
    if (alarmEndMillis > 0 && alarmCurrentMillis - alarmEndMillis >= ALARM_delay) {
      digitalWrite(GREENLED_PIN, HIGH);  // Grüne LED dauerhaft einschalten
      digitalWrite(REDLED_PIN, LOW);     // Rote LED ausschalten
      digitalWrite(BUZZER_PIN, LOW);     // Summer ausschalten
      alarmEndMillis = 0;                // Alarmende-Zeit zurücksetzen
    } else if (alarmEndMillis > 0) {
      // Alarmzustand aufrechterhalten
      if (alarmCurrentMillis - alarmPreviousMillis >= ALARM_interval) {
        alarmPreviousMillis = alarmCurrentMillis;
        digitalWrite(REDLED_PIN, !digitalRead(REDLED_PIN));  // Rote LED umschalten
        digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));  // Summer umschalten
      }
    }
  }

  isLastTotalAlarmState = isTotalAlarmState;  // Den letzten Alarmstatus aktualisieren
}




/* Tastenverarbeitung - Hauptfunktion ------------------------------------------------------------------*/
void handleButtons() {
  uint32_t currentTime = millis();

  for (int i = 0; i < 3; i++) {
    int reading = digitalRead(buttons[i].pin);

    switch (buttons[i].state) {
      case IDLE:
        if (reading == LOW) {
          // Tastenstatus von IDLE zu GEDRÜCKT ändern
          buttons[i].state = PRESSED;
          buttons[i].lastPressTime = currentTime;
        }
        break;

      case PRESSED:
        if (reading == HIGH) {
          // Kurzer Druck freigegeben
          handleShortPress(i);
          buttons[i].state = IDLE;
        } else if (currentTime - buttons[i].lastPressTime > LONG_PRESS_DURATION) {
          // Wechsel in den Langdruckzustand
          buttons[i].state = LONG_PRESSED;
          handleLongPress(i);
        }
        break;

      case LONG_PRESSED:
        if (reading == HIGH) {
          // Langdruck freigegeben
          buttons[i].state = IDLE;
        } else if (currentTime - buttons[i].lastHoldTime > HOLD_REPEAT_INTERVAL) {
          // Wechsel in den Zustand GEHALTEN
          buttons[i].state = HELD;
          buttons[i].lastHoldTime = currentTime;
          handleHold(i);
        }
        break;

      case HELD:
        if (reading == HIGH) {
          // Durchgehendes Halten freigegeben
          buttons[i].state = IDLE;
        } else if (currentTime - buttons[i].lastHoldTime > HOLD_REPEAT_INTERVAL) {
          // Wiederholtes Auslösen im Zustand GEHALTEN
          buttons[i].lastHoldTime = currentTime;
          handleHold(i);
        }
        break;
    }
  }
}


/* Tastenverarbeitung - Kurzdruckereignis ------------------------------------------------------------------*/
void handleShortPress(int buttonIndex) {
  switch (buttonIndex) {
    case 0:  // SELECT-Taste
      if (currentUI != SETTINGS_UI) {
        // Wechsel zwischen Haupt-UI und Neben-UI
        currentUI = (currentUI == MAIN_UI) ? SECONDARY_UI : MAIN_UI;
      } else {
        // Wechsel zum nächsten Einstellungselement in der Einstellungs-UI
        currentSettingItem = (currentSettingItem + 1) % NUM_SETTING_ITEM;  // Inkrement des aktuellen Einstellungsindex und zyklische Begrenzung
        forceUpdateGasThresholds();                                        // Erzwinge die Anzeige von Gasschwellenwerten, um Darstellungsprobleme nach UI-Wechsel zu lösen
      }
      break;
    case 1:  // LINKE Taste
    case 2:  // RECHTE Taste
      if (currentUI == SETTINGS_UI) {
        if (currentSettingItem == 0) {                         // Bei Auswahl der Alarmoption
          isTempAlarmTaskEnabled = !isTempAlarmTaskEnabled;    // Umschalten des Alarmstatus
        } else if (currentSettingItem == 1) {                  // Bei Auswahl der TinyML-Option
          isTempTinyMLTaskEnabled = !isTempTinyMLTaskEnabled;  // Umschalten des TinyML-Status
        } else {
          // Schwellenwert anpassen
          int direction = (buttonIndex == 1) ? -1 : 1;                    // Richtung der Anpassung basierend auf dem Tastendruck
          changeThreshold(direction, currentSettingItem == 7 ? 0.1 : 1);  // Schrittweite der Anpassung abhängig von der aktuellen Einstellung
        }
      }
      break;
  }
}


/* Tastenverarbeitung - Langdruckereignis ------------------------------------------------------------------*/
void handleLongPress(int buttonIndex) {
  if (buttonIndex == 0) {  // SELECT-Taste
    if (currentUI != SETTINGS_UI) {
      // Wechsel zur Einstellungs-UI
      currentUI = SETTINGS_UI;
    } else {
      // Speichern der Einstellungen und Verlassen der Einstellungs-UI
      applyTempSettingsToGlobal();  // Anwenden temporärer Einstellungen auf globale Variablen
      if (isSdConnected) {
        saveConfigToSD();  // Speichern der Konfiguration auf SD-Karte
      }
      notifyConfigSaved();  // Anzeige der Speichernachricht auf dem Bildschirm
      currentUI = MAIN_UI;
    }
  }
}


/* Tastenverarbeitung - Durchgehendes Halten ------------------------------------------------------------------*/
void handleHold(int buttonIndex) {
  if (currentUI == SETTINGS_UI && (buttonIndex == 1 || buttonIndex == 2)) {
    // Schnelle Anpassung der Schwellenwerte
    int direction = (buttonIndex == 1) ? -10 : 10;                  // Anpassungsrichtung basierend auf dem Tastendruck
    changeThreshold(direction, currentSettingItem == 7 ? 0.1 : 1);  // Schrittweite der Anpassung abhängig von der aktuellen Einstellung
  }
}


/* Hilfsfunktion zur Anpassung der Schwellenwerte ------------------------------------------------------------------*/
void changeThreshold(int delta, float step) {
  int index = currentSettingItem - NUM_EXTRA_SETTING_ITEM;
  tempThresholds[index] += delta * step;
  // Sicherstellen, dass der Schwellenwert innerhalb gültiger Grenzen bleibt
  tempThresholds[index] = constrain(tempThresholds[index],
                                    settings[index].minValue,
                                    settings[index].maxValue);
}


/* Anwenden von temporären Variablen aus den Einstellungen auf globale Variablen ------------------------------------------------------------------*/
void applyTempSettingsToGlobal() {
  for (int i = 0; i < 6; ++i) {
    settings[i].threshold = tempThresholds[i];
  }
  isAlarmTaskEnabled = isTempAlarmTaskEnabled;
  isTinyMLTaskEnabled = isTempTinyMLTaskEnabled;
}


/* Aktualisierung der Bildschirminhalte ------------------------------------------------------------------*/
void updateUI() {
  // Bildschirm löschen, wenn zwischen unterschiedlichen UIs gewechselt wird
  if (currentUI != lastUI) {
    oled.clearScreen();
  }

  // Bildschirminhalte aktualisieren basierend auf der ausgewählten UI
  switch (currentUI) {
    case MAIN_UI:
      mainUI();
      break;
    case SECONDARY_UI:
      secondaryUI();
      break;
    case SETTINGS_UI:
      settingsUI();
  }

  lastUI = currentUI;  // Aktuellen Bildschirmstatus für den nächsten Durchlauf speichern
}


/* Haupt-Bildschirm ------------------------------------------------------------------*/
void mainUI() {

  // Verarbeitung der TinyML-Benutzeroberfläche
  if (isTinyMLTaskEnabled && TinyMLTask_handle != NULL) {
    // Variablen zurücksetzen, wenn von einem anderen Bildschirm gewechselt wird
    if (currentUI != lastUI) {
      // Statusleistenfarbe zurücksetzen, um erneutes Zeichnen zu erzwingen
      lastColorStatusBar = BLACK;
      // Textzeilen zurücksetzen, um erneutes Zeichnen zu erzwingen
      lastLine1[0] = '\0';
      lastLine2[0] = '\0';
      lastLine3[0] = '\0';
      lastLine4[0] = '\0';
    }
    updateTinyMLUI();
    updateStatusBar();

  } else {  // Standard-Benutzeroberfläche verarbeiten
    // Haupt-UI-Basis zeichnen, wenn von einem anderen Bildschirm gewechselt wird
    if (currentUI != lastUI) {
      // Statusleistenfarbe zurücksetzen, um erneutes Zeichnen zu erzwingen
      lastColorStatusBar = BLACK;
      // Alle vorherigen Gaskonzentrationen und Dreieckstatus zurücksetzen
      for (int i = 0; i < 6; ++i) {
        lastGasConcentrations[i] = -1;
        isTriangleDrawn[i] = false;
      }
      oledDrawMainUI();  // Haupt-UI-Basis zeichnen
    }
    updateMainUI();  // Hauptbildschirmelemente aktualisieren
    updateStatusBar();
  }
}


/* Neben-Bildschirm ------------------------------------------------------------------*/
void secondaryUI() {

  // Sechs Koordinatensysteme zeichnen, wenn von einem anderen Bildschirm gewechselt wird
  if (currentUI != lastUI) {
    initGasHistories();  // Historische Konzentrationsdaten zurücksetzen
    oledDrawCoordUI();
  }
  updateGasCurve();  // Gas-Konzentrationskurven aktualisieren
}


/* Einstellungsbildschirm ------------------------------------------------------------------*/
void settingsUI() {
  // Einstellungs-UI-Basis zeichnen, wenn von einem anderen Bildschirm gewechselt wird
  if (currentUI != lastUI) {
    currentSettingItem = 0;  // Aktuelle Auswahl auf das erste Element zurücksetzen
    initTempSettings();      // Temporäre Variablen für die Einstellungs-UI initialisieren
    oledDrawSettingsUI();    // Einstellungs-UI-Basis zeichnen
  }
  updateSettingsValues();  // Werte der Einstellungsoptionen aktualisieren
}


/* Temporäre Variablen für die Einstellungs-UI initialisieren ------------------------------------------------------------------*/
void initTempSettings() {
  // Globale Variablen zur Initialisierung der temporären Variablen verwenden
  for (int i = 0; i < 6; ++i) {
    tempThresholds[i] = settings[i].threshold;
  }
  isTempAlarmTaskEnabled = isAlarmTaskEnabled;
  isTempTinyMLTaskEnabled = isTinyMLTaskEnabled;
}


/* Zeichnet die Grundlagen der Einstellungsoberfläche auf dem OLED ------------------------------------------------------------------*/
void oledDrawSettingsUI() {
  // Einstellungstitel zeichnen
  oled.setTextScale(2);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.print("Einstellungen");

  // Menüpunkt für den Alarmstatus hinzufügen
  int alarmY = 25;  // Start-Y-Koordinate
  oled.setTextScale(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, alarmY);
  oled.print("Alarm ");
  oled.setTextColor(BLUE);
  oled.print("(Ein/Aus)");

  // Menüpunkt für den TinyML-Status hinzufügen
  int tinymlY = alarmY + 12;  // 37
  oled.setTextScale(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, tinymlY);
  oled.print("TinyML ");
  oled.setTextColor(BLUE);
  oled.print("(Ein/Aus)");

  oled.drawLine(0, tinymlY + 12, 180, tinymlY + 12, WHITE);  // Trennlinie zeichnen, y == 49

  // Sechs Gaseinstellungen zeichnen
  oled.setTextScale(1);
  oled.setTextColor(WHITE);
  int y = tinymlY + 18;  // Start-Y-Koordinate, 55
  for (int i = 0; i < 6; ++i) {

    // Bei der Zeichnung anderer Gase keine Dezimalstellen beibehalten
    float tempMinValueFloat;
    float tempMaxValueFloat;
    int tempMinValueInt;
    int tempMaxValueInt;

    if (i != 5) {
      oled.setTextColor(WHITE);
      oled.setCursor(0, y);
      // Gasname drucken
      oled.print(GAS_NAME[i]);
      oled.print(" ");
      // Mindest- bis Höchstwertbereich drucken
      oled.setTextColor(CYAN);
      oled.print("(");
      // Überprüfung, ob der Mindestwert über 1000 liegt, falls ja, als 'k' darstellen
      if (settings[i].minValue >= 1000) {
        tempMinValueFloat = settings[i].minValue / 1000;  // Mindestwert in 'k' umwandeln
        tempMinValueInt = (int)tempMinValueFloat;         // Cast zum Integer
        oled.print(tempMinValueInt);
        oled.print("k..");
      } else {
        tempMinValueInt = (int)settings[i].minValue;  // Cast zum Integer
        oled.print(tempMinValueInt);
        oled.print("..");
      }
      // Überprüfung, ob der Höchstwert über 10000 liegt, falls ja, als 'k' darstellen
      if (settings[i].maxValue >= 10000) {
        tempMaxValueFloat = settings[i].maxValue / 1000;  // Höchstwert in 'k' umwandeln
        tempMaxValueInt = (int)tempMaxValueFloat;         // Cast zum Integer
        oled.print(tempMaxValueInt);
        oled.print("k)");
      } else {
        tempMaxValueInt = (int)settings[i].maxValue;  // Cast zum Integer
        oled.print(tempMaxValueInt);
        oled.print(")");
      }

    } else {
      // Beim Zeichnen des Bereichs für NO2 eine Dezimalstelle beibehalten
      oled.setTextColor(WHITE);
      oled.setCursor(0, y);
      // Gasname drucken
      oled.print(GAS_NAME[i]);
      oled.print(" ");

      String tempMinValueStr = String(settings[i].minValue, 1);  // Mindestwert als String mit einer Dezimalstelle
      String tempMaxValueStr = String(settings[i].maxValue, 1);  // Höchstwert als String mit einer Dezimalstelle

      // Mindest- bis Höchstwertbereich drucken
      oled.setTextColor(CYAN);
      oled.print("(");
      oled.print(tempMinValueStr);
      oled.print("...");
      oled.print(tempMaxValueStr);
      oled.print(")");
    }
    y += 12;
  }
}


/* Aktualisieren der Werte der Einstellungsoptionen ------------------------------------------------------------------*/
void updateSettingsValues() {
  oled.setTextScale(1);  // Schriftgröße auf 1 setzen
  oled.setTextColor(WHITE);

  // Status des akustisch-optischen Alarms aktualisieren
  int alarmY = 25;  // Start-Y-Koordinate
  if (isLastAlarmTaskEnabled != isTempAlarmTaskEnabled || lastUI != SETTINGS_UI) {
    oled.fillRect(83, alarmY, 42, 7, BLACK);  // Alte Daten löschen
    oled.setCursor(83, alarmY);
    // Farbe basierend auf dem Wert von isTempAlarmTaskEnabled wählen
    if (isTempAlarmTaskEnabled) {
      oled.setTextColor(ORANGE);  // Aktiv in Orange anzeigen
      oled.print("Ein");
    } else {
      oled.setTextColor(DARK_GREY);  // Inaktiv in Grau anzeigen
      oled.print("Aus");
    }
    isLastAlarmTaskEnabled = isTempAlarmTaskEnabled;
  }

  // Status von TinyML aktualisieren
  int tinymlY = alarmY + 12;  // 37
  if (isLastTinyMLTaskEnabled != isTempTinyMLTaskEnabled || lastUI != SETTINGS_UI) {
    oled.fillRect(83, tinymlY, 42, 7, BLACK);
    oled.setCursor(83, tinymlY);
    if (isTempTinyMLTaskEnabled) {
      oled.setTextColor(ORANGE);  // Aktiv in Orange anzeigen
      oled.print("Ein");
    } else {
      oled.setTextColor(DARK_GREY);  // Inaktiv in Grau anzeigen
      oled.print("Aus");
    }
    isLastTinyMLTaskEnabled = isTempTinyMLTaskEnabled;
  }

  // Gasgrenzwerte aktualisieren
  int y = tinymlY + 18;  // 55
  oled.setTextColor(WHITE);
  for (int i = 0; i < 6; ++i) {
    // Aktualisierungsstrategie: bei Änderungen oder beim ersten Laden der Einstellungs-UI
    if (tempThresholds[i] != lastThresholds[i] || lastUI != SETTINGS_UI) {
      // Alte Daten löschen
      oled.fillRect(83, y, 42, 7, BLACK);
      // Neue Daten zeichnen
      oled.setCursor(83, y);
      oled.print(tempThresholds[i]);
      // lastThresholds aktualisieren
      lastThresholds[i] = tempThresholds[i];
    }
    y += 12;
  }

  // Rahmen um die aktuell ausgewählte Einstellung zeichnen
  int currentY;
  if (currentSettingItem > 1) {
    currentY = 55 - 2 + (currentSettingItem - 2) * 12;  // Y-Koordinate basierend auf dem Gasgrenzwert-Index berechnen
  } else if (currentSettingItem == 0) {
    currentY = alarmY - 2;  // Y-Koordinate für den Alarm-Status
  } else if (currentSettingItem == 1) {
    currentY = tinymlY - 2;  // Y-Koordinate für den TinyML-Status
  }

  // Weißen Rahmen zeichnen, wenn ein Menüelement gewechselt wurde oder von einer anderen UI gekommen wurde
  if (lastSettingItem != currentSettingItem || lastUI != SETTINGS_UI) {
    int lastY;
    if (lastSettingItem > 1) {
      lastY = 55 - 2 + (lastSettingItem - 2) * 12;  // Y-Koordinate des letzten Gasgrenzwert-Indexes berechnen
    } else if (lastSettingItem == 0) {
      lastY = alarmY - 2;  // Letzte Position des Alarm-Status
    } else if (lastSettingItem == 1) {
      lastY = tinymlY - 2;  // Letzte Position des TinyML-Status
    }
    oled.drawRect(80, lastY - 1, 47, 13, BLACK);
    oled.drawRect(80, currentY - 1, 47, 13, WHITE);
    blinkState = true;
    updateSettingsUiCallCounter = 0;
    lastSettingItem = currentSettingItem;
  } else if (blinkState && !isNotifyConfigSaved) {  // Blinkzustand abhängig von blinkState, solange keine Benachrichtigung gespeichert wird
    oled.drawRect(80, currentY - 1, 47, 13, WHITE);
  } else if (!isNotifyConfigSaved) {
    oled.drawRect(80, currentY - 1, 47, 13, BLACK);
  }

  updateSettingsUiCallCounter++;  // Aufrufzähler erhöhen

  // Blinkzustand wechseln, wenn der Zähler einen bestimmten Wert erreicht
  if (updateSettingsUiCallCounter >= 2) {
    blinkState = !blinkState;
    updateSettingsUiCallCounter = 0;  // Zähler zurücksetzen
  }
}


/* Erzwinge die Aktualisierung der Gasschwellenwerte in der Einstellungs-UI ------------------------------------------------------------------*/
void forceUpdateGasThresholds() {
  oled.setTextScale(1);  // Schriftgröße auf 1 setzen
  oled.setTextColor(WHITE);
  int y = 55;  // Start-Y-Koordinate
  for (int i = 0; i < 6; ++i) {
    // Alte Daten löschen
    oled.fillRect(83, y, 42, 7, BLACK);
    oled.setCursor(83, y);
    oled.print(tempThresholds[i]);
    y += 12;
  }
}


/* Beim Speichern der Einstellungen wird eine Benachrichtigung auf dem Bildschirm angezeigt ------------------------------------------------------------------*/
void notifyConfigSaved() {
  isNotifyConfigSaved = true;
  oled.fillRect(2, 40, 124, 50, BLACK);
  oled.drawRect(2, 40, 124, 50, WHITE);
  oled.setTextColor(WHITE);
  oled.setCursor(10, 57);
  oled.setTextScale(2);
  oled.print("Gespeichert!");
  vTaskDelay(pdMS_TO_TICKS(500));
  isNotifyConfigSaved = false;
  lastUI = SETTINGS_UI;
}


/* Zeichnen der Grund-Benutzeroberfläche auf dem Hauptbildschirm ------------------------------------------------------------------*/
void oledDrawMainUI() {

  // Sechs Gase mit der Einheit ppm zeichnen
  oled.setTextScale(1);
  oled.setTextColor(WHITE);
  int y = 35;  // Start-Y-Koordinate
  for (int i = 0; i < 6; ++i) {
    oled.setCursor(3, y);
    oled.print(GAS_NAME[i]);
    oled.setCursor(106, y);
    oled.println("ppm");
    y += 16;
  }
}


/* Aktualisieren der Hauptbildschirmelemente ------------------------------------------------------------------*/
void updateMainUI() {
  uint32_t currentTimestamp = millis();

  // Überprüfen, ob sich die Gaskonzentration geändert hat
  bool anyGasChange = false;
  for (int i = 0; i < 6; ++i) {
    if (gasConcentrations[i] != lastGasConcentrations[i]) {
      anyGasChange = true;
      break;
    }
  }

  // Bei Änderungen den Timer zurücksetzen und das Flag für erzwungenes Aktualisieren setzen
  if (anyGasChange) {
    lastUpdateTimestamp = currentTimestamp;
    isForceRefresh = true;
  }

  // Überprüfen, ob innerhalb der erzwungenen Aktualisierungszeit
  if (isForceRefresh) {

    if (currentTimestamp - lastUpdateTimestamp > KEEP_REFRESH_TIME) {
      isForceRefresh = false;  // Beendet die erzwungene Aktualisierung nach der eingestellten Timerzeit
    }

    int y = 35;

    for (int i = 0; i < 6; ++i) {
      // Alte Daten löschen
      oled.fillRect(58, y, 45, 7, BLACK);

      // Textfarbe abhängig von der Gaskonzentration im Verhältnis zum Schwellenwert bestimmen
      if (isAlarmState[i]) {
        colorGasData = RED;  // Im Alarmzustand, rot
        isStatusBarRed = true;

      } else if (gasConcentrations[i] >= (YELLOW_percentage / 100.0 * settings[i].threshold)) {
        colorGasData = YELLOW;  // Zwischen YELLOW_percentage% und 100% des eingestellten Schwellenwerts, gelb
        isStatusBarYellow = true;

      } else {
        colorGasData = GREEN;  // Im Normalzustand, grün
        isStatusBarGreen = true;
      }

      oled.setTextColor(colorGasData);  // Textfarbe für die Gaskonzentration setzen

      // Neue Daten zeichnen
      oled.setCursor(58, y);
      oled.print(gasConcentrations[i]);

      lastGasConcentrations[i] = gasConcentrations[i];  // Aktualisieren der letzten Gaskonzentrationen

      // Entscheidung, ob ein rotes Dreieck gezeichnet werden soll, abhängig vom Alarmstatus
      if (isAlarmState[i] && !isTriangleDrawn[i]) {
        // Wenn noch kein Dreieck vorhanden, rotes Dreieck zeichnen
        oled.fillTriangle(43, y + 8, 49, y - 2, 55, y + 8, RED);
        // Ein Ausrufezeichen im Dreieck zeichnen
        oled.drawLine(49, y + 1, 49, y + 5, BLACK);
        oled.drawPixel(49, y + 7, BLACK);
        // Markieren, dass ein rotes Dreieck gezeichnet wurde
        isTriangleDrawn[i] = true;
      } else if (!isAlarmState[i] && isTriangleDrawn[i]) {
        // Wenn vorhanden, rotes Dreieck entfernen
        oled.fillTriangle(43, y + 8, 49, y - 2, 55, y + 8, BLACK);
        // Markieren, dass das rote Dreieck entfernt wurde
        isTriangleDrawn[i] = false;
      }
      y += 16;
    }

    // Statusbarfarbe basierend auf der oben bestimmten Textfarbe festlegen
    if (isStatusBarRed) {
      colorStatusBar = MAROON;
    } else if (isStatusBarYellow) {
      colorStatusBar = ORANGE;
    } else if (isStatusBarGreen) {
      colorStatusBar = DARKGREEN;
    }

    // Nach der Entscheidung die Statusflags zurücksetzen
    isStatusBarGreen = false;
    isStatusBarYellow = false;
    isStatusBarRed = false;
  }
}


/* Aktualisierung der Statusleiste ------------------------------------------------------------------*/
void updateStatusBar() {

  // Aktualisieren der Hintergrundfarbe der Statusleiste
  if (colorStatusBar != lastColorStatusBar) {
    // Neue Hintergrundfarbe der Statusleiste zeichnen
    oled.fillRect(0, 0, 128, 26, colorStatusBar);
    lastColorStatusBar = colorStatusBar;

    // Teilvariablen zurücksetzen, um anschließend die Statusleiste mit Zeit-, Temperatur- und Symbolinformationen neu zu zeichnen
    lastOutRtcTemperature = 1000;
    lastOutRtcHour[0] = '\0';
    lastOutRtcMinute[0] = '\0';
    lastOutRtcSecond[0] = '\0';
    isNoRtcConnectionDrawn = false;
    isMLIconDrawn = false;
    isAlarmIconDrawn = false;
    isWifiIconDrawn = false;
    isSdIconDrawn = false;
    currentPowerIcon = POWER_NONE;
    lastFillHeight = -1;
  }


  // Zeit und Temperatur auf der Statusleiste aktualisieren
  if (isOutRtcConnected) {

    // Doppelpunkte zwischen der Uhrzeit zeichnen
    oled.setTextColor(WHITE);
    oled.setTextScale(1);
    oled.setCursor(15, 4);
    oled.print(":");
    oled.setCursor(29, 4);
    oled.print(":");

    // Uhrzeitinformationen aktualisieren
    // Sekunden aktualisieren
    if (strcmp(outRtcSecond, lastOutRtcSecond) != 0) {
      // Alte Zeit löschen
      oled.fillRect(31, 4, 11, 7, colorStatusBar);
      // Neue Zeit zeichnen
      oled.setCursor(31, 4);
      oled.print(outRtcSecond);

      // Aktuelle Zeit in globale Variable speichern
      strcpy(lastOutRtcSecond, outRtcSecond);


      // Minuten aktualisieren
      if (strcmp(outRtcMinute, lastOutRtcMinute) != 0) {
        oled.fillRect(17, 4, 11, 7, colorStatusBar);
        // Neue Zeit zeichnen
        oled.setCursor(17, 4);
        oled.print(outRtcMinute);

        // Aktuelle Zeit in globale Variable speichern
        strcpy(lastOutRtcMinute, outRtcMinute);


        // Stunden aktualisieren
        if (strcmp(outRtcHour, lastOutRtcHour) != 0) {
          oled.fillRect(3, 4, 11, 7, colorStatusBar);
          // Neue Zeit zeichnen
          oled.setCursor(3, 4);
          oled.print(outRtcHour);

          // Aktuelle Zeit in globale Variable speichern
          strcpy(lastOutRtcHour, outRtcHour);
        }
      }
    }

    // Temperaturinformationen aktualisieren
    if (outRtcTemperature != lastOutRtcTemperature) {
      // Alte Temperaturinformationen löschen
      oled.fillRect(3, 15, 40, 7, colorStatusBar);
      // OLED zeichnet Temperaturinformationen
      oled.setCursor(3, 15);
      oled.setTextColor(WHITE);
      oled.setTextScale(1);
      oled.print(outRtcTemperature);
      oled.print(" °C");

      // Aktuelle Temperaturinformationen in globale Variable speichern
      lastOutRtcTemperature = outRtcTemperature;
    }

    // Behandlung, wenn keine RTC-Verbindung besteht
  } else if (!isNoRtcConnectionDrawn) {
    oled.setTextColor(WHITE);
    oled.setTextScale(1);
    oled.setCursor(3, 4);
    oled.print("Keine");
    oled.setCursor(3, 15);
    oled.print("RTC");
    isNoRtcConnectionDrawn = true;
  }


  //// Aktualisieren der fünf Statussymbole in der Statusleiste
  // TinyML-Symbol
  if (!isMLIconDrawn) {

    // Zustandstext basierend auf dem Schalterstatus bestimmen
    if (isTinyMLTaskEnabled) {
      oled.setTextColor(CYAN);
      oled.setTextScale(1);
      oled.setCursor(46, 6);
      oled.print("ML");
      oled.setTextColor(ORANGE);
      oled.setCursor(46, 14);
      oled.print("Ein");
    } else {
      oled.setTextColor(LIGHT_GREY);
      oled.setTextScale(1);
      oled.setCursor(46, 6);
      oled.print("ML");
      oled.setTextColor(DARK_GREY);
      oled.setCursor(45, 14);
      oled.print("Aus");
    }

    // Wenn das Symbol bereits gezeichnet wurde, aber TinyMLTask unerwartet beendet wurde
  } else if (isTinyMLTaskEnabled && TinyMLTask_handle == NULL) {
    // Zustandstext zeichnen
    oled.fillRect(45, 14, 15, 7, colorStatusBar);  // Alten Status text löschen
    oled.setTextColor(RED);
    oled.setCursor(46, 14);
    oled.print("Err");
  }


  // Alarm-Symbol
  if (!isAlarmIconDrawn) {
    drawSpeakerIcon(61, 5, isAlarmTaskEnabled, false);  // Alarm-Symbol zeichnen und basierend auf dem Wert der temporären Variable isTempAlarmTaskEnabled den Symbolstatus auswählen
    isAlarmIconDrawn = true;

    // Wenn das Symbol bereits gezeichnet wurde, aber AlarmTask unerwartet beendet wurde
  } else if (isAlarmTaskEnabled && AlarmTask_handle == NULL) {
    oled.fillRect(59, 4, 18, 18, colorStatusBar);  // Altes Symbol löschen
    drawSpeakerIcon(60, 5, false, true);           // Alarm-Symbol zeichnen
  }


  // WiFi-Symbol
  if (isWifiEnabled) {           //Wenn die WiFi-Funktion aktiviert ist, den aktuellen WiFi-Status abrufen
    wifiStatus = WiFi.status();  // Aktuellen WiFi-Verbindungsstatus abrufen
  }

  if (!isWifiIconDrawn) {

    drawWiFiIcon(78, 5);  // WiFi-Symbol zeichnen

    // Überprüfen, ob die WiFi-Funktion aktiviert ist
    if (!isWifiEnabled) {  // Wenn die WiFi-Funktion deaktiviert ist, rotes Diagonalkreuz zeichnen
      // Dickes rotes Diagonalkreuz zeichnen
      oled.drawLine(79, 7, 90, 18, RED);
      oled.drawLine(80, 7, 90, 17, RED);
      oled.drawLine(80, 6, 91, 17, RED);
    } else if (wifiStatus != WL_CONNECTED) {  // Wenn WiFi aktiviert, aber nicht verbunden ist, gelbes Diagonalkreuz zeichnen
      // Dickes gelbes Diagonalkreuz zeichnen
      oled.drawLine(79, 7, 90, 18, YELLOW);
      oled.drawLine(80, 7, 90, 17, YELLOW);
      oled.drawLine(80, 6, 91, 17, YELLOW);
    }

    isWifiIconDrawn = true;

    // Überprüfen, ob sich der WiFi-Verbindungsstatus geändert hat
  } else if (isWifiEnabled && wifiStatus != lastWiFiStatus) {

    oled.fillRect(77, 4, 18, 18, colorStatusBar);  // Altes Symbol löschen
    drawWiFiIcon(78, 5);                           // WiFi-Symbol erneut zeichnen

    // Überprüfen, ob WiFi verbunden ist
    if (wifiStatus != WL_CONNECTED) {
      // Dickes gelbes Diagonalkreuz zeichnen
      oled.drawLine(79, 7, 90, 18, YELLOW);
      oled.drawLine(80, 7, 90, 17, YELLOW);
      oled.drawLine(80, 6, 91, 17, YELLOW);
    }

    lastWiFiStatus = wifiStatus;
  }


  // SD-Karten-Symbol
  if (!isSdIconDrawn) {
    drawSDCardIcon(95, 5);  // SD-Karten-Symbol zeichnen

    if (!isSdConnected) {
      // Dickes gelbes Diagonalkreuz zeichnen
      oled.drawLine(95, 7, 108, 20, YELLOW);
      oled.drawLine(96, 7, 108, 19, YELLOW);
      oled.drawLine(96, 6, 109, 19, YELLOW);
    }

    isSdIconDrawn = true;
  }


  // Stromversorgungssymbol zeichnen
  // Wechsel des Symbols je nach Stromversorgungsstatus
  if (isBatteryConnected && currentPowerIcon != POWER_BATTERY) {
    drawBatteryIcon(111, 5, capacityPercentage);  // Batteriesymbol zeichnen
    currentPowerIcon = POWER_BATTERY;
  } else if (!isBatteryConnected && currentPowerIcon != POWER_USB) {
    drawUsbIcon(110, 5);  // USB-Stromversorgungssymbol zeichnen
    currentPowerIcon = POWER_USB;
  } else if (currentPowerIcon == POWER_BATTERY) {
    drawBatteryIcon(111, 5, capacityPercentage);  // Batteriesymbol aktualisieren
  }
}


/* Zeichnen des Lautsprechersymbols 16px ------------------------------------------------------------------*/
void drawSpeakerIcon(int x, int y, bool enable, bool error) {
  // Lautsprechergehäuse (hohles Rechteck)
  oled.drawRect(x + 1, y + 5, 4, 6, WHITE);

  // Lautsprecherkonus (Umriss)
  oled.drawLine(x + 5, y + 4, x + 8, y + 1, WHITE);
  oled.drawLine(x + 5, y + 11, x + 8, y + 14, WHITE);
  oled.drawLine(x + 8, y + 1, x + 8, y + 14, WHITE);

  if (error) {
    // Rotes 'E' zeichnen
    oled.setTextColor(RED);
    oled.setCursor(x + 11, y + 5);
    oled.print("E");

  } else {

    if (enable) {
      //// Schallwellen zeichnen
      // Kleiner Bogen
      oled.drawPixel(x + 10, y + 5, WHITE);
      oled.drawPixel(x + 11, y + 6, WHITE);
      oled.drawPixel(x + 11, y + 7, WHITE);
      oled.drawPixel(x + 11, y + 8, WHITE);
      oled.drawPixel(x + 11, y + 9, WHITE);
      oled.drawPixel(x + 10, y + 10, WHITE);

      // Großer Bogen
      oled.drawPixel(x + 11, y + 2, WHITE);
      oled.drawPixel(x + 12, y + 3, WHITE);
      oled.drawPixel(x + 13, y + 4, WHITE);
      oled.drawPixel(x + 13, y + 5, WHITE);
      oled.drawPixel(x + 14, y + 6, WHITE);
      oled.drawPixel(x + 14, y + 7, WHITE);
      oled.drawPixel(x + 14, y + 8, WHITE);
      oled.drawPixel(x + 14, y + 9, WHITE);
      oled.drawPixel(x + 13, y + 10, WHITE);
      oled.drawPixel(x + 13, y + 11, WHITE);
      oled.drawPixel(x + 12, y + 12, WHITE);
      oled.drawPixel(x + 11, y + 13, WHITE);

    } else {
      // Rotes Kreuz zeichnen
      oled.drawLine(x + 10, y + 3, x + 14, y + 12, RED);
      oled.drawLine(x + 10, y + 12, x + 14, y + 3, RED);
      // Dickes rotes Kreuz (um einen Pixel versetzt, erneut zeichnen)
      oled.drawLine(x + 11, y + 3, x + 15, y + 12, RED);
      oled.drawLine(x + 11, y + 12, x + 15, y + 3, RED);
    }
  }
}


/* Zeichnen des WiFi-Symbols 16px ------------------------------------------------------------------*/
void drawWiFiIcon(int x, int y) {
  // Basispunkt des WiFi-Symbols
  oled.drawPixel(x + 8, y + 13, WHITE);
  oled.drawPixel(x + 7, y + 12, WHITE);
  oled.drawPixel(x + 8, y + 12, WHITE);
  oled.drawPixel(x + 9, y + 12, WHITE);

  // Erster (innerster) Bogen
  oled.drawPixel(x + 5, y + 10, WHITE);
  oled.drawPixel(x + 6, y + 9, WHITE);
  oled.drawPixel(x + 7, y + 9, WHITE);
  oled.drawPixel(x + 8, y + 9, WHITE);
  oled.drawPixel(x + 9, y + 9, WHITE);
  oled.drawPixel(x + 10, y + 9, WHITE);
  oled.drawPixel(x + 11, y + 10, WHITE);

  // Zweiter Bogen
  oled.drawPixel(x + 3, y + 8, WHITE);
  oled.drawPixel(x + 4, y + 7, WHITE);
  oled.drawPixel(x + 5, y + 7, WHITE);
  oled.drawPixel(x + 6, y + 6, WHITE);
  oled.drawPixel(x + 7, y + 6, WHITE);
  oled.drawPixel(x + 8, y + 6, WHITE);
  oled.drawPixel(x + 9, y + 6, WHITE);
  oled.drawPixel(x + 10, y + 6, WHITE);
  oled.drawPixel(x + 11, y + 7, WHITE);
  oled.drawPixel(x + 12, y + 7, WHITE);
  oled.drawPixel(x + 13, y + 8, WHITE);

  // Dritter Bogen
  oled.drawPixel(x + 1, y + 6, WHITE);
  oled.drawPixel(x + 2, y + 5, WHITE);
  oled.drawPixel(x + 3, y + 4, WHITE);
  oled.drawPixel(x + 4, y + 4, WHITE);
  oled.drawPixel(x + 5, y + 3, WHITE);
  oled.drawPixel(x + 6, y + 3, WHITE);
  oled.drawPixel(x + 7, y + 3, WHITE);
  oled.drawPixel(x + 8, y + 3, WHITE);
  oled.drawPixel(x + 9, y + 3, WHITE);
  oled.drawPixel(x + 10, y + 3, WHITE);
  oled.drawPixel(x + 11, y + 3, WHITE);
  oled.drawPixel(x + 12, y + 4, WHITE);
  oled.drawPixel(x + 13, y + 4, WHITE);
  oled.drawPixel(x + 14, y + 5, WHITE);
  oled.drawPixel(x + 15, y + 6, WHITE);
}


/* Zeichnen des SD-Karten-Symbols 16px ------------------------------------------------------------------*/
void drawSDCardIcon(int x, int y) {
  // Hauptkörper der SD-Karte
  oled.drawLine(x + 5, y + 1, x + 13, y + 1, WHITE);
  oled.drawLine(x + 2, y + 14, x + 13, y + 14, WHITE);
  oled.drawLine(x + 2, y + 4, x + 2, y + 13, WHITE);
  oled.drawLine(x + 13, y + 2, x + 13, y + 13, WHITE);
  oled.drawPixel(x + 3, y + 3, WHITE);
  oled.drawPixel(x + 4, y + 2, WHITE);

  // SD-Kontaktpunkte
  oled.drawLine(x + 5, y + 4, x + 5, y + 5, WHITE);
  oled.drawLine(x + 7, y + 3, x + 7, y + 5, WHITE);
  oled.drawLine(x + 9, y + 3, x + 9, y + 5, WHITE);
  oled.drawLine(x + 11, y + 3, x + 11, y + 5, WHITE);
}


/* Zeichnen des Batteriesymbols 16px ------------------------------------------------------------------*/
void drawBatteryIcon(int x, int y, float capacity) {

  // Sicherstellen, dass die Kapazität zwischen 0 und 100 liegt
  capacity = constrain(capacity, 0, 100);

  // Füllhöhe berechnen
  int fillHeight = map(capacity, 0, 100, 0, 10);

  // Wenn die Füllhöhe gleich der letzten Füllhöhe ist, sofort beenden
  if (fillHeight == lastFillHeight) {
    return;
  }

  lastFillHeight = fillHeight;  // lastFillHeight aktualisieren

  // Farben initialisieren
  uint16_t fillColor;            // Füllfarbe
  uint16_t borderColor = WHITE;  // Rahmenfarbe
  // Ausrufezeichen-Flag initialisieren
  bool exclamationMark = false;

  // Wählen der Füllfarbe und Rahmenfarbe des Batterieinneren basierend auf dem verbleibenden Kapazitätsprozentsatz
  if (fillHeight > 0) {  // Entsprechend 10%-100% Ladung

    if (capacity <= 20) {  // Entsprechend 10%-20% Ladung
      borderColor = RED;
      fillColor = RED;
    } else if (capacity <= 50) {  // Entsprechend 30.01%-50% Ladung
      fillColor = YELLOW;
    } else {  // Entsprechend 50.01%-100% Ladung
      fillColor = GREEN;
    }

  } else {  // Entsprechend 0%-9.99% Ladung
    borderColor = RED;
    exclamationMark = true;
  }

  oled.fillRect(x, y - 1, 16, 19, colorStatusBar);                      // Bereich des Symbols löschen (4px mehr auf der Y-Achse)
  oled.drawRect(x + 3, y + 1, 10, 14, borderColor);                     // Hauptkörper der Batterie zeichnen
  oled.drawLine(x + 6, y, x + 9, y, borderColor);                       // Positiven Pol der Batterie zeichnen
  oled.fillRect(x + 4, y + 2, 8, 12, BLACK);                            // Inneren Bereich der Batterie löschen
  oled.fillRect(x + 5, y + 13 - fillHeight, 6, fillHeight, fillColor);  // Inneren Bereich der Batterie füllen

  if (exclamationMark) {
    oled.drawRect(x + 7, y + 4, 2, 5, RED);
    oled.drawRect(x + 7, y + 10, 2, 2, RED);
  }
}


/* Zeichnen des USB-Symbols 16px ------------------------------------------------------------------*/
void drawUsbIcon(int x, int y) {

  oled.fillRect(x, y - 1, 16, 19, colorStatusBar);  // Bereich des Symbols löschen (4px mehr auf der Y-Achse)

  // USB-Anschluss
  oled.drawRect(x + 4, y, 9, 6, WHITE);
  oled.fillRect(x + 6, y + 2, 2, 2, WHITE);
  oled.fillRect(x + 9, y + 2, 2, 2, WHITE);
  // Hauptkörper
  oled.drawRect(x + 3, y + 5, 11, 9, WHITE);
  // Kleines Blitzsymbol
  oled.drawLine(x + 9, y + 6, x + 6, y + 9, WHITE);
  oled.drawLine(x + 7, y + 9, x + 10, y + 9, WHITE);
  oled.drawLine(x + 9, y + 10, x + 7, y + 12, WHITE);
  // Datenkabel
  oled.drawLine(x + 6, y + 14, x + 10, y + 14, WHITE);
  oled.drawPixel(x + 8, y + 15, WHITE);
  // Datenkabel verlängern, führt zu einer Verlängerung von 2px nach unten
  oled.drawLine(x + 8, y + 16, x + 8, y + 17, WHITE);
}



/* Aktualisieren der TinyML-Benutzeroberfläche ------------------------------------------------------------------*/
void updateTinyMLUI() {
  // Aktualisiere die Oberfläche, wenn sich irgendwelche anzuzeigenden Werte geändert haben
  if (line1 != lastLine1 || line2 != lastLine2 || line3 != lastLine3 || line4 != lastLine4) {
    oled.clearScreen();

    // Vier Zeilen Text zeichnen
    oled.setTextScale(2);
    oled.setTextColor(colorline1);
    oled.setCursor(0, 40);
    oled.print(line1);
    oled.setTextColor(colorline2);
    oled.setCursor(0, 60);
    oled.print(line2);
    oled.setTextColor(colorline3);
    oled.setCursor(0, 90);
    oled.print(line3);
    oled.setTextColor(colorline4);
    oled.setCursor(0, 110);
    oled.print(line4);
    // Speichere die aktuellen Textzeilen
    lastLine1 = line1;
    lastLine2 = line2;
    lastLine3 = line3;
    lastLine4 = line4;
  }
}



/* Zeichnen eines einzelnen Koordinatensystems ------------------------------------------------------------------*/
void drawCoordinateSystem(int x, int y, int width, int height) {
  oled.drawLine(x, y, x, y + height, WHITE);                   // Zeichnen der Y-Achse
  oled.drawLine(x, y + height, x + width, y + height, WHITE);  // Zeichnen der X-Achse
}


/* Zeichnen eines einzelnen Etiketts in der linken unteren Ecke des Koordinatensystems ------------------------------------------------------------------*/
void drawCoordinateLabel(int x, int y, int width, int height, const char *label) {
  oled.setTextScale(1);
  oled.setTextColor(WHITE);
  oled.setCursor(x + 2, y + height - 10);
  oled.print(label);
}


/* Zeichnen von sechs Koordinatensystemen auf der sekundären Oberfläche ------------------------------------------------------------------*/
void oledDrawCoordUI() {
  // Berechnen der Größe und Position der Koordinatensysteme und diese zeichnen
  drawCoordinateSystem(X_OFFSET, Y_OFFSET, COORD_WIDTH, COORD_HEIGHT);
  drawCoordinateSystem(X_OFFSET + COORD_WIDTH + X_GAP, Y_OFFSET, COORD_WIDTH, COORD_HEIGHT);

  drawCoordinateSystem(X_OFFSET, Y_OFFSET + COORD_HEIGHT + Y_GAP, COORD_WIDTH, COORD_HEIGHT);
  drawCoordinateSystem(X_OFFSET + COORD_WIDTH + X_GAP, Y_OFFSET + COORD_HEIGHT + Y_GAP, COORD_WIDTH, COORD_HEIGHT);

  drawCoordinateSystem(X_OFFSET, Y_OFFSET + 2 * (COORD_HEIGHT + Y_GAP), COORD_WIDTH, COORD_HEIGHT);
  drawCoordinateSystem(X_OFFSET + COORD_WIDTH + X_GAP, Y_OFFSET + 2 * (COORD_HEIGHT + Y_GAP), COORD_WIDTH, COORD_HEIGHT);
}


/* Zeichnen der Etiketten für die sechs Koordinatensysteme der sekundären Oberfläche ------------------------------------------------------------------*/
void oledDrawCoordLabel() {
  // Berechnen der Positionen der Etiketten und diese zeichnen
  drawCoordinateLabel(X_OFFSET, Y_OFFSET, COORD_WIDTH, COORD_HEIGHT, "CH4");
  drawCoordinateLabel(X_OFFSET + COORD_WIDTH + X_GAP, Y_OFFSET, COORD_WIDTH, COORD_HEIGHT, "C2H5OH");

  drawCoordinateLabel(X_OFFSET, Y_OFFSET + COORD_HEIGHT + Y_GAP, COORD_WIDTH, COORD_HEIGHT, "H2");
  drawCoordinateLabel(X_OFFSET + COORD_WIDTH + X_GAP, Y_OFFSET + COORD_HEIGHT + Y_GAP, COORD_WIDTH, COORD_HEIGHT, "NH3");

  drawCoordinateLabel(X_OFFSET, Y_OFFSET + 2 * (COORD_HEIGHT + Y_GAP), COORD_WIDTH, COORD_HEIGHT, "CO");
  drawCoordinateLabel(X_OFFSET + COORD_WIDTH + X_GAP, Y_OFFSET + 2 * (COORD_HEIGHT + Y_GAP), COORD_WIDTH, COORD_HEIGHT, "NO2");
}


/* Initialisieren der Historie der Gaskonzentrationen ------------------------------------------------------------------*/
void initGasHistories() {
  for (int i = 0; i < NUM_COORD_SYSTEM; ++i) {
    gasHistories[i].concentrations.clear();  // Konzentrationsarray leeren
    gasHistories[i].minConcentration = FLT_MAX;
    gasHistories[i].maxConcentration = -FLT_MAX;
  }
}


/* Aktualisieren der Gaskonzentrationshistorie und Zeichnen des Liniendiagramms ------------------------------------------------------------------*/
void updateGasCurve() {
  // Aktualisieren der Historie jeder Gaskonzentration
  for (int i = 0; i < NUM_COORD_SYSTEM; ++i) {
    gasHistories[i].concentrations.push_back(gasConcentrations[i]);
    gasHistories[i].minConcentration = std::min(gasHistories[i].minConcentration, gasConcentrations[i]);
    gasHistories[i].maxConcentration = std::max(gasHistories[i].maxConcentration, gasConcentrations[i]);
  }

  // Zeichnen der Liniendiagramme
  for (int i = 0; i < NUM_COORD_SYSTEM; ++i) {
    int x = X_OFFSET_CURVE + i % 2 * (COORD_WIDTH + X_GAP);
    int y = Y_OFFSET_CURVE + i / 2 * (COORD_HEIGHT + Y_GAP);

    // Berechnen des Y-Achsen-Bereichs
    float minConc = gasHistories[i].minConcentration;
    float maxConc = gasHistories[i].maxConcentration;
    float range = maxConc - minConc;
    if (range == 0) range = 1;  // Vermeidung der Division durch null

    // Vor dem Zeichnen neuer Linien die alten Linien schwarz übermalen
    oled.fillRect(x, y, COORD_WIDTH, COORD_HEIGHT + 1, BLACK);
    // Zeichnen der Etiketten für sechs Koordinatensysteme
    oledDrawCoordLabel();

    // Zeichnen neuer Linien
    for (size_t j = 1; j < gasHistories[i].concentrations.size(); ++j) {
      int x1 = x + (j - 1) * COORD_WIDTH / gasHistories[i].concentrations.size();
      int y1 = y + COORD_HEIGHT - (gasHistories[i].concentrations[j - 1] - minConc) / range * COORD_HEIGHT;
      int x2 = x + j * COORD_WIDTH / gasHistories[i].concentrations.size();
      int y2 = y + COORD_HEIGHT - (gasHistories[i].concentrations[j] - minConc) / range * COORD_HEIGHT;
      oled.drawLine(x1, y1, x2, y2, YELLOW);
    }

    // Wenn das Liniendiagramm die volle Breite des Koordinatensystems ausfüllt, wird der älteste Datenpunkt entfernt
    if (gasHistories[i].concentrations.size() > COORD_WIDTH) {
      gasHistories[i].concentrations.erase(gasHistories[i].concentrations.begin());
      // Aktualisieren der minimalen und maximalen Konzentration
      gasHistories[i].minConcentration = *std::min_element(gasHistories[i].concentrations.begin(), gasHistories[i].concentrations.end());
      gasHistories[i].maxConcentration = *std::max_element(gasHistories[i].concentrations.begin(), gasHistories[i].concentrations.end());
    }
  }
}




/* Überprüfen des verbleibenden Speicherplatzes auf der SD-Karte und Verwalten der Logdateien ------------------------------------------------------------------*/
void checkFreeSpaceAndManageLogs() {
  uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);                // Gesamte Byteanzahl der SD-Karte (MB)
  uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);                  // Bereits verwendete Byteanzahl der SD-Karte (MB)
  uint64_t freeBytes = totalBytes - usedBytes;                          // Verbleibende Byteanzahl (MB)
  float freePercentage = ((float)freeBytes / (float)totalBytes) * 100;  // Verbleibender Speicherplatz in Prozent

  Serial.println();
  Serial.println("---------- Periodic Space Check Task ----------");

  // Gesamt-, verwendetes und freies Speichervolumen der SD-Karte ausgeben
  Serial.printf("[INFO] Total Space: %llu MB\n", totalBytes);
  Serial.printf("[INFO] Used Space: %llu MB\n", usedBytes);
  Serial.printf("[INFO] Free Space: %llu MB\n", freeBytes);
  Serial.printf("[INFO] Free Percentage: %.2f%%\n", freePercentage);

  // Schleife zur Entscheidung, ob die älteste Logdatei gelöscht werden muss, um ausreichend Speicherplatz freizugeben
  while (freePercentage < MIN_FREE_SPACE_percentage) {
    // Löschen der ältesten Logdatei
    deleteOldestLogFile();
    // Aktualisieren des Prozentanteils des freien Speichers
    totalBytes = SD.totalBytes() / (1024 * 1024);                   // Gesamte Byteanzahl der SD-Karte (MB)
    usedBytes = SD.usedBytes() / (1024 * 1024);                     // Bereits verwendete Byteanzahl der SD-Karte (MB)
    freeBytes = totalBytes - usedBytes;                             // Verbleibende Byteanzahl (MB)
    freePercentage = ((float)freeBytes / (float)totalBytes) * 100;  // Verbleibender Speicherplatz in Prozent

    if (freePercentage < MIN_FREE_SPACE_percentage) {
      Serial.println("[INFO] MicroSD has freed up enough space");
      Serial.printf("[INFO] Free Percentage: %.2f%%\n", freePercentage);
    }
  }

  Serial.println("-----------------------------------------------");
  Serial.println();
}


/* Löschen der ältesten Logdatei ------------------------------------------------------------------*/
void deleteOldestLogFile() {
  File root = SD.open("/");
  File file = root.openNextFile();
  char oldestFileName[40];
  uint32_t oldestFileTime = UINT32_MAX;

  // Entfernen des Präfix '/' aus der Konstanten CONFIG_FILE_NAME und Speichern in einer neuen temporären Variablen configFileNameWithoutPrefix
  char configFileNameWithoutPrefix[sizeof(CONFIG_FILE_NAME) - 1];
  strcpy(configFileNameWithoutPrefix, CONFIG_FILE_NAME + 1);

  // Durchlaufen aller Dateien im Wurzelverzeichnis der SD-Karte, um die Datei mit dem frühesten Änderungsdatum zu finden, wobei der Dateiname über die Funktion file.name() ohne Präfix '/' erhalten wird
  while (file) {
    if (!file.isDirectory()) {
      if (strcmp(file.name(), configFileNameWithoutPrefix) != 0 && strcmp(file.name(), "System Volume Information") != 0) {  // Ausschluss der Konfigurationsdatei im Wurzelverzeichnis und des Verzeichnisses 'System Volume Information'
        uint32_t fileTime = file.getLastWrite();
        if (fileTime < oldestFileTime) {
          oldestFileTime = fileTime;
          // Hinzufügen des Präfix '/' zum erhaltenen Dateinamen
          snprintf(oldestFileName, sizeof(oldestFileName), "/%s", file.name());
        }
      }
    }
    file = root.openNextFile();
  }

  if (strlen(oldestFileName) > 0) {
    SD.remove(oldestFileName);
    Serial.printf("[INFO] Deleted old log file: %s\n", oldestFileName);
  }
}


/* Schreiben der aktuellen Gaskonzentrationen in die SD-Kartenprotokolldatei ------------------------------------------------------------------*/
void logSensorData(float sensorData[]) {

  // Systembetriebszeit berechnen und schreiben
  float currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000;  // In Sekunden umrechnen
  logFile.print(elapsedTime);
  logFile.print("s | ");

  // Wenn das externe RTC-Modul verbunden ist, aktuelle Zeit- und Temperaturinformationen schreiben
  if (isOutRtcConnected) {
    // Zeitinformationen schreiben
    logFile.print(outRtcTime);
    logFile.print(" | ");
    // Temperaturinformationen schreiben
    logFile.print(outRtcTemperature);
    logFile.print("℃ || ");
  } else {
    logFile.print("KEINE RTC! || ");
  }

  // Gassensor-Daten schreiben
  for (int i = 0; i < 6; i++) {
    logFile.print(sensorData[i]);
    logFile.print(" | ");
  }

  // Zeilenumbruch in der Protokolldatei und Puffer leeren
  logFile.println();
  logFile.flush();  // Sicherstellen, dass Daten geschrieben werden
}


/* Einstellungen auf SD-Karte speichern ------------------------------------------------------------------*/
void saveConfigToSD() {
  File configFile = SD.open(CONFIG_FILE_NAME, FILE_WRITE);
  if (!configFile) {
    Serial.println("[ERROR] Failed to open config file for writing");
    return;
  }

  // JSON-Dokument erstellen
  StaticJsonDocument<1024> doc;
  JsonArray configArray = doc.createNestedArray("config");

  for (int i = 0; i < 6; i++) {
    JsonObject configObject = configArray.createNestedObject();
    configObject["name"] = settings[i].name;  // Gaskonzentration speichern
    configObject["minValue"] = settings[i].minValue;
    configObject["maxValue"] = settings[i].maxValue;
    configObject["threshold"] = settings[i].threshold;
  }

  // JSON in Datei serialisieren, formatierte Ausgabe verwenden
  if (serializeJsonPretty(doc, configFile) == 0) {
    Serial.println("[ERROR] Failed to write to config file");
    return;
  }

  configFile.close();

  Serial.println("[INFO] Config saved to SD");
  Serial.println();
}




/* Serielle Ausgabe der Gaskonzentrationsdaten ------------------------------------------------------------------*/
void serialPrintf() {
  Serial.printf("CH4: %7.1f PPM    |    C2H5OH: %5.1f PPM    |    H2: %6.1f PPM    |    NH3: %5.1f PPM    |    CO: %6.1f PPM    |    NO2: %4.1f PPM\n",
                gasConcentrations[0], gasConcentrations[1], gasConcentrations[2],
                gasConcentrations[3], gasConcentrations[4], gasConcentrations[5]);
}



/* Gas-Schwellenwert-Aktualisierungsfunktion ------------------------------------------------------------------*/
void updateGasThreshold(void *value, JsonVariantConst jsonValue) {
  SettingItem *item = static_cast<SettingItem *>(value);
  float newValue = jsonValue.as<float>();
  if (newValue >= item->minValue && newValue <= item->maxValue) {
    item->threshold = newValue;
    Serial.printf("[INFO] Thingsboard: Updated %s threshold to %.2f\n", item->name, newValue);
  } else {
    Serial.printf("[WARN] Thingsboard: Update %s threshold unsuccessful, value out of range\n", item->name);
  }
  // Wenn beim Initialisieren die SD-Karte verbunden war, werden die Einstellungen auf der SD-Karte gespeichert
  if (isSdConnected) {
    saveConfigToSD();
  }
}

/* Boolean-Wert-Aktualisierungsfunktion ------------------------------------------------------------------*/
void updateBooleanValue(void *value, JsonVariantConst jsonValue) {
  bool *boolValue = static_cast<bool *>(value);
  *boolValue = jsonValue.as<bool>();
  Serial.printf("[INFO] Thingsboard: Updated boolean value to %s\n", *boolValue ? "true" : "false");
}

/* Integer-Wert-Aktualisierungsfunktion ------------------------------------------------------------------*/
void updateIntValue(void *value, JsonVariantConst jsonValue) {
  uint16_t *intValue = static_cast<uint16_t *>(value);
  *intValue = jsonValue.as<uint16_t>();
  Serial.printf("[INFO] Thingsboard: Updated integer value to %u\n", *intValue);
}

/* Float-Wert-Aktualisierungsfunktion ------------------------------------------------------------------*/
void updateFloatValue(void *value, JsonVariantConst jsonValue) {
  if (value == nullptr) {
    Serial.println("[ERROR] Thingsboard: Null pointer received in updateFloatValue");
    return;
  }

  float *floatValue = static_cast<float *>(value);
  float newValue = jsonValue.as<float>();

  // Überprüfen, ob der Wert eine gültige Fließkommazahl ist
  if (isnan(newValue) || isinf(newValue)) {
    Serial.println("[ERROR] Thingsboard: Invalid float value received");
    return;
  }

  *floatValue = newValue;

  // Verwenden der Funktion dtostrf zum Formatieren von Fließkommazahlen
  char floatStr[10];                     // Zeichenarray, das die meisten Fließkommazahlen aufnehmen kann
  dtostrf(*floatValue, 4, 2, floatStr);  // 4 ist die Mindestbreite, 2 die Anzahl der Dezimalstellen

  Serial.printf("[INFO] Thingsboard: Updated float value to %s\n", floatStr);
}


/* ThingsBoard Gemeinsame Attribute Einzeleinstellungen ----------------------------------------------------------------------------------------------*/

// Definition des Arrays gemeinsamer Attribute
std::array<SharedAttribute, ATTRIBUTE_COUNT> SHARED_ATTRIBUTES = {
  {
    { "CH4_threshold", &settings[0], updateGasThreshold },
    { "C2H5OH_threshold", &settings[1], updateGasThreshold },
    { "H2_threshold", &settings[2], updateGasThreshold },
    { "NH3_threshold", &settings[3], updateGasThreshold },
    { "CO_threshold", &settings[4], updateGasThreshold },
    { "NO2_threshold", &settings[5], updateGasThreshold },
    { "_isAlarmEnabled", &isAlarmTaskEnabled, updateBooleanValue },
    { "_isTinyMLEnabled", &isTinyMLTaskEnabled, updateBooleanValue },
    { "GAS_CHECK_time", &GAS_CHECK_time, updateIntValue },
    { "ALARM_interval", &ALARM_interval, updateIntValue },
    { "ALARM_delay", &ALARM_delay, updateIntValue },
    { "YELLOW_percentage", &YELLOW_percentage, updateIntValue },
    { "MIN_FREE_SPACE_percentage", &MIN_FREE_SPACE_percentage, updateIntValue },
    { "CONFIDENCE_threshold", &CONFIDENCE_threshold, updateFloatValue },
  }
};
// Erstellen eines Arrays, das nur die Attributnamen für die ThingsBoard-Bibliothek enthält
std::array<const char *, ATTRIBUTE_COUNT> SUBSCRIBED_SHARED_ATTRIBUTES = {
  { SHARED_ATTRIBUTES[0].name,
    SHARED_ATTRIBUTES[1].name,
    SHARED_ATTRIBUTES[2].name,
    SHARED_ATTRIBUTES[3].name,
    SHARED_ATTRIBUTES[4].name,
    SHARED_ATTRIBUTES[5].name,
    SHARED_ATTRIBUTES[6].name,
    SHARED_ATTRIBUTES[7].name,
    SHARED_ATTRIBUTES[8].name,
    SHARED_ATTRIBUTES[9].name,
    SHARED_ATTRIBUTES[10].name,
    SHARED_ATTRIBUTES[11].name,
    SHARED_ATTRIBUTES[12].name,
    SHARED_ATTRIBUTES[13].name }
};


/* Daten mit Thingsboard synchronisieren ------------------------------------------------------------------*/
void SyncWithThingsboard() {
  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    Serial.printf("[INFO] Thingsboard: Connecting to: %s with token %s\n", THINGSBOARD_SERVER, TOKEN);
    Serial.println();
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println();
      Serial.println("[WARN] Thingsboard: Failed to connect to Server");
      Serial.println();
      return;
    }
  }

  Serial.println("[INFO] Thingsboard: Sending data");
  Serial.println();

  // Senden der Gassensordaten (als Telemetriedaten)
  for (int i = 0; i < 6; i++) {
    tb.sendTelemetryData(GAS_NAME[i], gasConcentrations[i]);
  }

  // Senden der Temperaturdaten (als Telemetriedaten)
  if (isOutRtcConnected) {
    tb.sendTelemetryData("Temperature", outRtcTemperature);
  }

  // Gemeinsame Attribute an ThingsBoard senden
  sendAttributesToThingsBoard();

  // Abonnieren der serverseitig geteilten Attribute (Einstellungen: Gasschwellenwerte und zwei Schalter)
  if (!isSubscribed) {
    Serial.println("[INFO] Thingsboard: Subscribing for shared attribute updates");
    const Shared_Attribute_Callback<ATTRIBUTE_COUNT> callback(&processSharedAttributeUpdate, SUBSCRIBED_SHARED_ATTRIBUTES.cbegin(), SUBSCRIBED_SHARED_ATTRIBUTES.cend());
    if (!tb.Shared_Attributes_Subscribe(callback)) {
      Serial.println("[WARN] Thingsboard: Failed to subscribe for shared attribute updates!");
      Serial.println();
      return;
    }
    isSubscribed = true;
    Serial.println("[INFO] Thingsboard: Subscribed OK");
    Serial.println();
  }

  tb.loop();
}


/* Gemeinsame Attribute senden ------------------------------------------------------------------*/
void sendAttributesToThingsBoard() {
  // Senden der Schwellenwerte der Gassensoren
  for (int i = 0; i < 6; i++) {
    // Senden von Attributen an ThingsBoard, häufiges Aufrufen ist unbedenklich, da die ThingsBoard-Regelkette so eingestellt ist, dass sie nur bei Änderungen speichert
    tb.sendAttributeData(SUBSCRIBED_SHARED_ATTRIBUTES[i], settings[i].threshold);
  }

  // Senden der beiden Funktionsschalter aus den Einstellungen
  tb.sendAttributeData(SUBSCRIBED_SHARED_ATTRIBUTES[6], isAlarmTaskEnabled);
  tb.sendAttributeData(SUBSCRIBED_SHARED_ATTRIBUTES[7], isTinyMLTaskEnabled);

  // Senden der Überprüfungsanzahl, des Alarmintervalls und der Verzögerungszeit
  tb.sendAttributeData(SUBSCRIBED_SHARED_ATTRIBUTES[8], GAS_CHECK_time);
  tb.sendAttributeData(SUBSCRIBED_SHARED_ATTRIBUTES[9], ALARM_interval);
  tb.sendAttributeData(SUBSCRIBED_SHARED_ATTRIBUTES[10], ALARM_delay);

  // Senden des gelben Schwellenwertprozentsatzes und des minimalen verbleibenden SD-Kartenspeicherplatzes
  tb.sendAttributeData(SUBSCRIBED_SHARED_ATTRIBUTES[11], YELLOW_percentage);
  tb.sendAttributeData(SUBSCRIBED_SHARED_ATTRIBUTES[12], MIN_FREE_SPACE_percentage);

  // Senden des Konfidenzschwellenwerts für die TinyML-Erkennung
  tb.sendAttributeData(SUBSCRIBED_SHARED_ATTRIBUTES[13], CONFIDENCE_threshold);
}


/*  Callback-Funktion zur Verarbeitung der Aktualisierung gemeinsamer Attribute ------------------------------------------------------------------*/
void processSharedAttributeUpdate(const JsonObjectConst &data) {
  for (JsonPairConst kvp : data) {
    String key = kvp.key().c_str();
    for (const auto &attr : SHARED_ATTRIBUTES) {
      if (key == attr.name) {
        attr.updateFunc(attr.value, kvp.value());
        // Hier kann bei Bedarf Logik zum Speichern auf der SD-Karte hinzugefügt werden
        break;
      }
    }
  }

  // Besondere Verarbeitungslogik
  if (data.containsKey("_isAlarmEnabled")) {
    lastColorStatusBar = BLACK;  // Zurücksetzen der Statusleistenfarbe, um die Statusleiste zu aktualisieren
  }
  if (data.containsKey("_isTinyMLEnabled") || data.containsKey("YELLOW_percentage") || data.containsKey("CONFIDENCE_threshold")) {
    lastUI = SETTINGS_UI;
    currentUI = MAIN_UI;
  }
}



/* Überprüfen der WiFi-Verbindung und bei Bedarf erneut verbinden ------------------------------------------------------------------*/
bool reconnect() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  Serial.println("[INFO] Reconnecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  Serial.println("[INFO] WiFi Reconnected");
  return true;
}




/* Echtzeit und Temperatur vom externen RTC abrufen und in globale Variablen schreiben ------------------------------------------------------------------*/
void getRealTimeAndTempFromOutRTC() {
  // Globale Variablen auf Standardformate zurücksetzen
  strcpy(outRtcTime, TIME_FORMAT);
  strcpy(outRtcHour, HOUR_FORMAT);
  strcpy(outRtcMinute, MINUTE_FORMAT);
  strcpy(outRtcSecond, SECOND_FORMAT);

  // Aktuelle Zeit- und Temperaturinformationen abrufen und in globale Variablen schreiben
  outRTC.now().toString(outRtcTime);            // Aktuelle Zeit abrufen und in die globale Variable outRtcTime schreiben, Format hh:mm:ss
  outRTC.now().toString(outRtcHour);            // hh-Format der Stunde abrufen
  outRTC.now().toString(outRtcMinute);          // mm-Format der Minute abrufen
  outRTC.now().toString(outRtcSecond);          // ss-Format der Sekunde abrufen
  outRtcTemperature = outRTC.getTemperature();  // Temperatur abrufen
}




/* Funktion zur Berechnung der Batteriekapazität anhand der Spannung ------------------------------------------------------------------*/
float calculateCapacity(float voltage) {
  // Verwendung von Stückweise linearer Interpolation zur Schätzung der Batteriekapazität
  if (voltage >= V100) return 100.0;
  if (voltage >= V90) return 90.0 + (voltage - V90) * V90_100_RATIO;
  if (voltage >= V70) return 70.0 + (voltage - V70) * V70_90_RATIO;
  if (voltage >= V40) return 40.0 + (voltage - V40) * V40_70_RATIO;
  if (voltage >= V20) return 20.0 + (voltage - V20) * V20_40_RATIO;
  if (voltage >= V0) return (voltage - V0) * V0_20_RATIO;
  return 0.0;  // Rückgabe von 0% Kapazität, wenn die Spannung unter BATTERY_MIN_VOLTAGE liegt
}


/* Überprüfen des Batteriestatus ------------------------------------------------------------------*/
void CheckBattery() {
  // Starten der ADC-Konvertierung
  analogContinuousStart();

  // Warten auf Abschluss der Konvertierung (eventuell komplexere Synchronisationsmechanismen erforderlich)
  vTaskDelay(pdMS_TO_TICKS(100));

  Serial.println();
  Serial.println("--------- Periodic Battery Check Task ---------");

  if (analogContinuousRead(&adcResult, 0)) {
    float vout = adcResult[0].avg_read_mvolts / 1000.0 + ADC_OFFSET;
    float vin = vout / VOLTAGE_DIVISION_FACTOR + VOL_OFFSET;

#if TEMP_COMPENSATION
    // Wenn ein externes RTC angeschlossen ist, dann Temperaturkompensation anwenden
    if (isOutRtcConnected) {
      float tempCompensation = 1.0 + (outRtcTemperature - 25) * 0.002;  // Temperaturkompensationsfaktor (muss je nach tatsächlicher Situation angepasst werden)
      vin *= tempCompensation;                                          // Temperaturkompensation anwenden
    }
#endif

    capacityPercentage = calculateCapacity(vin);  // Berechnung des verbleibenden Kapazitätsprozentsatzes

    // Überprüfen des Batterieverbindungsstatus
    if (vin < BATTERY_CONNECTION_MIN_THRESHOLD || vin > BATTERY_CONNECTION_MAX_THRESHOLD) {
      isBatteryConnected = false;
      Serial.println("[INFO] The voltage sensor is not connected or is being powered by USB");

    } else {
      isBatteryConnected = true;

      // Ausgabe der Ergebnisse
      Serial.printf("[INFO] Battery Voltage: %.2fV\n", vin);
      Serial.printf("[INFO] Estimated Capacity: %.1f%%\n", capacityPercentage);

      if (capacityPercentage <= 10.0) {
        Serial.printf("[CRITICAL] Battery Nearly Depleted!\n");
      } else if (capacityPercentage <= 20.0) {
        Serial.printf("[WARN] Low Battery!\n");
      }
    }

  } else {
    printf("[ERROR] Error reading ADC data\n");
  }

  Serial.println("-----------------------------------------------");
  Serial.println();

  // Stoppen der ADC-Konvertierung
  analogContinuousStop();
}




/* ========================================================== TinyML-Spezialbereich ========================================================== */


/**
 * @brief Kopiert Daten aus dem features-Array an eine bestimmte Position und Länge in den Zielpuffer
 * 
 * @param sourceOffset Startposition der zu kopierenden Daten im features-Array
 * @param dataLength Länge der zu kopierenden Daten
 * @param destinationBuffer Zielpuffer, in den die Daten kopiert werden sollen
 * @return true Kopiervorgang erfolgreich
 * @return false Kopiervorgang fehlgeschlagen
 */
bool copyFeatureDataToBuffer(size_t sourceOffset, size_t dataLength, float *destinationBuffer) {
  if (destinationBuffer == nullptr) {
    return false;  // Ungültiger Zielpuffer
  }

  memcpy(destinationBuffer, features + sourceOffset, dataLength * sizeof(float));
  return true;
}


/* Inferenz mit TinyML ------------------------------------------------------------------*/
/*!
Diese Funktion bezieht sich auf die logische Struktur und Teile des Codes in der loop-Funktion der Datei 'bhopal4.ino' 
von bhopal84 (https://github.com/ronibandini/bhopal84) und überarbeitet den Originalcode erheblich.

Originalarbeit Copyright (c) 2022 Roni Bandini
Modifizierte Arbeit Copyright (c) 2024 Hang Yu

Siehe NOTICE-Datei für vollständige Urheberrechts- und Lizenzdetails.
*/
void InferenceUsingTinyML() {

  // Bei Erreichen der maximalen Anzahl von Messungen pro Gruppe werden Berechnungen und Inferenz durchgeführt
  if (sampleCounter == SAMPLE_NUMBER) {

    sampleCounter = 0;  // Zähler zurücksetzen

    // Durchschnittswerte berechnen
    for (int i = 0; i < 6; ++i) {
      mlGasData[i].avg = mlGasData[i].sum / SAMPLE_NUMBER;
    }

    // Minimale Werte normalisieren
    for (int i = 0; i < 6; ++i) {
      if (mlGasData[i].min == ML_DEFAULT_MIN) {
        mlGasData[i].min = 0;
      }
    }

#if SERIAL_PRINT_ROUND  // Wenn das Drucken der Enddaten jeder Runde aktiviert ist, auf die serielle Schnittstelle drucken

    Serial.println("----------------------- Sending to inference");
    // Durchschnittswerte der aktuellen Runde drucken
    for (int i = 0; i < 6; ++i) {
      Serial.print(mlGasData[i].avg);
      Serial.print(",");
    }
    Serial.println();

    // Minimale Werte der aktuellen Runde drucken
    for (int i = 0; i < 6; ++i) {
      Serial.print(mlGasData[i].min, 1);
      Serial.print(",");
    }
    Serial.println();

    // Maximale Werte der aktuellen Runde drucken
    for (int i = 0; i < 6; ++i) {
      Serial.print(mlGasData[i].max, 1);
      if (i < 5) {  // Wenn nicht das letzte Element, Komma ausgeben
        Serial.print(",");
      }
    }
    Serial.println();

#endif

    // Feature-Werte aktualisieren
    for (int i = 0; i < 6; ++i) {
      features[i] = mlGasData[i].avg;
    }

    for (int i = 6; i < 12; ++i) {
      features[i] = mlGasData[i].min;
    }

    for (int i = 12; i < 18; ++i) {
      features[i] = mlGasData[i].max;
    }

    // Zur Inferenz senden
    // Überprüfen, ob alle Durchschnittswerte 0 sind
    if (features[0] == 0 && features[1] == 0 && features[2] == 0 && features[3] == 0 && features[4] == 0 && features[5] == 0) {

      isTotalAlarmState = false;  // Alarmsystem nicht auslösen

      Serial.println("All gas readings 0");
      Serial.println();

      // Bildschirmaktualisierung
      line1 = "Alle Werte";
      line2 = "sind 0";
      line3 = "Keine Gase";
      line4 = "erkannt";
      colorline1 = GREEN;
      colorline2 = GREEN;
      colorline3 = GREEN;
      colorline4 = GREEN;
      // Statusleistenfarbe aktualisieren
      colorStatusBar = DARKGREEN;

    } else {
      Serial.println("Gas readings ok");

      if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
                  EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
        Serial.println();
        return;
      }

      ei_impulse_result_t result = { 0 };

      // Die Features sind im Flash gespeichert und wir wollen nicht alles in den RAM laden
      signal_t features_signal;
      features_signal.total_length = sizeof(features) / sizeof(features[0]);
      features_signal.get_data = &copyFeatureDataToBuffer;

      // Impuls aufrufen
      EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
      ei_printf("run_classifier returned: %d\n", res);

      if (res != 0) return;

      // Vorhersagen drucken
      ei_printf("Predictions ");
      ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);
      ei_printf(": \n");
      ei_printf("[");

      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("%.5f", result.classification[ix].value);

#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf(", ");
#else
        if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
          ei_printf(", ");
        }
#endif
      }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
      ei_printf("%.3f", result.anomaly);
#endif
      ei_printf("]\n");

      // Vertrauen zurücksetzen
      myScoreRegular = 0;
      myScoreHarmful = 0;

      // Menschlich lesbare Vorhersagen
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);

        if (result.classification[ix].label == "regular") {
          myScoreRegular = result.classification[ix].value;
        }  // regular

        if (result.classification[ix].label == "harmful") {
          myScoreHarmful = result.classification[ix].value;
        }  // harmful

      }  // Alle Labels durchlaufen

      // Wenn keine der Vorhersagen über dem Konfidenzlimit liegt
      if (myScoreHarmful < CONFIDENCE_threshold && myScoreRegular < CONFIDENCE_threshold) {

        isTotalAlarmState = false;  // Alarmsystem nicht auslösen

        // Bildschirmaktualisierung
        line1 = "Toxisches Gas";
        line2 = String(myScoreHarmful * 100) + "%";
        line3 = "Normales Gas";
        line4 = String(myScoreRegular * 100) + "%";
        colorline1 = DARKCYAN;
        colorline2 = WHITE;
        colorline3 = DARKCYAN;
        colorline4 = WHITE;
        // Statusleistenfarbe aktualisieren
        colorStatusBar = NAVY;
      }

      // Wenn reguläres Gas erkannt wird
      if (myScoreRegular > CONFIDENCE_threshold) {

        isTotalAlarmState = false;  // Alarmsystem nicht auslösen
        Serial.println("Regular gas detected!");

        // Bildschirmaktualisierung
        line1 = "Normales Gas";
        line2 = "erkannt!";
        line3 = String(myScoreRegular * 100) + "%";
        line4 = "";
        colorline1 = YELLOW;
        colorline2 = YELLOW;
        colorline3 = WHITE;
        colorline4 = YELLOW;
        // Statusleistenfarbe aktualisieren
        colorStatusBar = ORANGE;
      }

      // Wenn schädliches Gas erkannt wird
      if (myScoreHarmful > CONFIDENCE_threshold) {

        isTotalAlarmState = true;  // Alarmsystem auslösen
        Serial.println("Harmful gas detected!");

        // Bildschirmaktualisierung
        line1 = "Toxisches Gas";
        line2 = "erkannt!";
        line3 = String(myScoreHarmful * 100) + "%";
        line4 = "";
        colorline1 = RED;
        colorline2 = RED;
        colorline3 = WHITE;
        colorline4 = RED;
        // Statusleistenfarbe aktualisieren
        colorStatusBar = MAROON;
      }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
      ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

      Serial.println();

    }  // Ende der Inferenz

    // Gasdaten zurücksetzen, sum, min, max zurücksetzen; avg wird in jeder Runde der Datenberechnung überschrieben, daher kein Reset erforderlich
    for (int i = 0; i < 6; ++i) {
      mlGasData[i].sum = 0;
      mlGasData[i].min = ML_DEFAULT_MIN;
      mlGasData[i].max = 0;
    }

  } else {  // Wenn die maximale Anzahl der Proben nicht erreicht ist, eine Probe messen

    sampleCounter++;  // Sampling-Zähler erhöhen

    // Gasmessungen vom Sensor abrufen
    getGasData();

    // Daten summieren, um den Durchschnitt am Ende jeder Runde zu berechnen
    for (int i = 0; i < 6; ++i) {
      mlGasData[i].sum = mlGasData[i].sum + gasConcentrations[i];
    }

#if SERIAL_PRINT_SAMPLE  // Wenn das Drucken jeder Stichprobe aktiviert ist, auf die serielle Schnittstelle drucken

    Serial.println("----------------------- Sample " + String(sampleCounter));

    for (int i = 0; i < 6; ++i) {
      Serial.print(gasConcentrations[i], 1);
      if (i < 5) {  // Wenn nicht das letzte Element, Komma ausgeben
        Serial.print(",");
      }
    }
    Serial.println();

#endif

    // Maximale Werte der aktuellen Runde bestimmen
    for (int i = 0; i < 6; ++i) {
      if (gasConcentrations[i] > mlGasData[i].max) {  // Wenn nicht das letzte Element, Komma ausgeben
        mlGasData[i].max = gasConcentrations[i];
      }
    }

    // Minimale Werte der aktuellen Runde bestimmen
    for (int i = 0; i < 6; ++i) {
      if (gasConcentrations[i] < mlGasData[i].min) {  // Wenn nicht das letzte Element, Komma ausgeben
        mlGasData[i].min = gasConcentrations[i];
      }
    }
  }
}




/* ========================================================== Task und Timer -Spezialbereich ========================================================== */


/* Aufgabe: Gaskonzentration vom Sensor lesen (Ziel-Funktion enthält I2C-Mutex) ------------------------------------------------------------------*/
void ReadGasTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLING_INTERVAL);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  Serial.println("[INFO] Task: Read Gas - Started");
  Serial.println();

  initGasBuffers();  // Initialisierung der Gaspuffer (für die Filterung)

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  // Eine Aufgabe sollte niemals zurückkehren oder beenden
  for (;;) {

    // Ziel-Funktion aufrufen
    getGasData();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: Überprüfen, ob die Konzentration überschritten ist ------------------------------------------------------------------*/
void CheckGasTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLING_INTERVAL);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  Serial.println("[INFO] Task: Check Gas - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  for (;;) {

    checkGasConcentration();  // Ziel-Funktion aufrufen

    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: Akustischer und visueller Alarm ------------------------------------------------------------------*/
void AlarmTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(2 * SAMPLING_INTERVAL);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  Serial.println("[INFO] Task: Alarm - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  digitalWrite(GREENLED_PIN, HIGH);  // Löst das Problem, dass die grüne LED beim ersten Start der Funktion checkTotalAlarmState() nicht ständig leuchtet

  for (;;) {

    checkTotalAlarmState();  // Ziel-Funktion aufrufen

    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: Benutzeroberfläche aktualisieren (mit SPI-Mutex) ------------------------------------------------------------------*/
void UpdateUiTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(REFRESH_INTERVAL);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  Serial.println("[INFO] Task: Update UI - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  for (;;) {

    /*
#if USE_MUTEX
    if (xSemaphoreTake(spiMutex_handle, portMAX_DELAY) == pdTRUE) {  // SPI-Mutex holen
#endif
*/

    updateUI();  // Ziel-Funktion aufrufen

    /*
#if USE_MUTEX
      xSemaphoreGive(spiMutex_handle);  // SPI-Mutex freigeben
    }
#endif
*/

    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: Tasten verarbeiten ------------------------------------------------------------------*/
void HandleButtonsTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  // const TickType_t xFrequency = 200;  // 200 Ticks = 0,2 Sekunden
  const TickType_t xFrequency = pdMS_TO_TICKS(50);  // Konvertierung mit dem Makro pdMS_TO_TICKS, direkt in Millisekunden eingeben

  Serial.println("[INFO] Task: Handle Buttons - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  for (;;) {

    handleButtons();  // Ziel-Funktion aufrufen

    // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


/* Aufgabe: Zeit und Temperatur jede Sekunde abrufen (mit I2C-Mutex) ------------------------------------------------------------------*/
void GetTimeAndTempTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  Serial.println("[INFO] Task: Get Time and Temp - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  for (;;) {
#if USE_MUTEX
    if (xSemaphoreTake(spiMutex_handle, portMAX_DELAY) == pdTRUE) {  // SPI-Mutex holen
#endif

      getRealTimeAndTempFromOutRTC();  // Ziel-Funktion aufrufen

#if USE_MUTEX
      xSemaphoreGive(spiMutex_handle);  // SPI-Mutex freigeben
    }
#endif
    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: Protokoll auf SD-Karte aufzeichnen (mit SPI-Mutex) ------------------------------------------------------------------*/
void LogTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLING_INTERVAL);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  Serial.println("[INFO] Task: Log to SD - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  for (;;) {
#if USE_MUTEX
    if (xSemaphoreTake(spiMutex_handle, portMAX_DELAY) == pdTRUE) {  // SPI-Mutex holen
#endif

      logSensorData(gasConcentrations);  // Ziel-Funktion aufrufen

#if USE_MUTEX
      xSemaphoreGive(spiMutex_handle);  // SPI-Mutex freigeben
    }
#endif
    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: Protokoll über serielle Schnittstelle drucken ------------------------------------------------------------------*/
void SerialPrintTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(250);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  vTaskDelay(pdMS_TO_TICKS(500));  // Verzögerung von 500ms, damit andere Aufgaben zuerst abgeschlossen werden, bevor das Protokoll über die serielle Schnittstelle gedruckt wird

  Serial.println("[INFO] Task: Serial Print - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  for (;;) {

    serialPrintf();  // Ziel-Funktion aufrufen

    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: Mit Thingsboard synchronisieren ------------------------------------------------------------------*/
void ThingsboardTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  vTaskDelay(pdMS_TO_TICKS(250));  // Verzögerung von 250ms, damit andere Aufgaben zuerst abgeschlossen werden, bevor das Protokoll über die serielle Schnittstelle gedruckt wird

  Serial.println("[INFO] Task: Sync with Thingsboard - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  for (;;) {

    SyncWithThingsboard();  // Ziel-Funktion aufrufen

    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: TinyML Inferenz ------------------------------------------------------------------*/
void TinyMLTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_TIME_FRAME);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  Serial.println("[INFO] Task: TinyML Inference - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  sampleCounter = 0;  // Sampling-Zähler zurücksetzen

  // Eine Aufgabe sollte niemals zurückkehren oder beenden
  for (;;) {

    InferenceUsingTinyML();  // Ziel-Funktion aufrufen

    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: Regelmäßige Überprüfung, ob der freie Speicherplatz auf der SD-Karte knapp wird; Ziel-Funktion wird alle zehn Minuten aufgerufen (mit SPI-Mutex) ------------------------------------------------------------------*/
void ManageSdSpaceTask_func(void *parameter) {

  Serial.println("[INFO] Task: Log Management - Started");
  Serial.println();

// Aufgabe einmalig beim Start ausführen
#if USE_MUTEX
  if (xSemaphoreTake(spiMutex_handle, portMAX_DELAY) == pdTRUE) {  // SPI-Mutex holen
#endif

    checkFreeSpaceAndManageLogs();  // Ziel-Funktion aufrufen

#if USE_MUTEX
    xSemaphoreGive(spiMutex_handle);  // SPI-Mutex freigeben
  }
#endif

  // Nachfolgende Schleife
  for (;;) {
    // Warten auf Timer-Benachrichtigung (wenn keine Benachrichtigung eingeht, bleibt die Funktion blockiert, beansprucht jedoch keine CPU-Ressourcen und beeinträchtigt daher nicht den Ablauf anderer Aufgaben)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#if USE_MUTEX
    if (xSemaphoreTake(spiMutex_handle, portMAX_DELAY) == pdTRUE) {  // SPI-Mutex holen
#endif

      checkFreeSpaceAndManageLogs();  // Ziel-Funktion aufrufen

#if USE_MUTEX
      xSemaphoreGive(spiMutex_handle);  // SPI-Mutex freigeben
    }
#endif
  }
}


/* Timer: Callback-Funktion für die SD-Karten-Speicherverwaltungsaufgabe ------------------------------------------------------------------*/
void sdCardTimerCallback(TimerHandle_t xTimer) {
  xTaskNotifyGive(ManageSdSpaceTask_handle);
}



/* Aufgabe: Bildschirmaufwärmphase anzeigen, einmalig ausführen ------------------------------------------------------------------*/
void loadingScreenTask_func(void *parameter) {

  loadingScreen();  // Aufruf der Funktion loadingScreen

  Serial.println("[INFO] Task: Startup Loading Screen - Finished");  // Serielle Ausgabe bei Abschluss der Aufgabe
  Serial.println();

  vTaskDelete(LoadingScreenTask_handle);  // Aufgabe nach Abschluss löschen
  LoadingScreenTask_handle = NULL;        // Verhindern des Aufrufs von vTaskDelete bei nicht existierender Aufgabe
}


/* Aufgabe: Aufgabenüberwachung (Daemon-Aufgabe) ------------------------------------------------------------------*/
void DaemonTask_func(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500);  // Konvertierung mit dem Makro pdMS_TO_TICKS

  Serial.println("[INFO] Task: Daemon - Started");
  Serial.println();

  xLastWakeTime = xTaskGetTickCount();  // Aktuelle Systemzeit als Startzeit holen

  for (;;) {

    // 1. Überprüfen des AlarmTask-Status
    if (isAlarmTaskEnabled && AlarmTask_handle == NULL) {
      // Wenn Flag true ist und Aufgabe nicht läuft, Aufgabe erstellen
      xTaskCreate(
        AlarmTask_func,       // Aufgabenfunktion
        "Check Alarm State",  // Aufgabenname
        2048,                 // Stapelgröße
        NULL,                 // Aufgabenparameter
        4,                    // Priorität
        &AlarmTask_handle     // Aufgabenhandle
      );

    } else if (!isAlarmTaskEnabled && AlarmTask_handle != NULL) {
      // Wenn Flag false ist und Aufgabe läuft, Aufgabe löschen
      vTaskDelete(AlarmTask_handle);
      AlarmTask_handle = NULL;
      // Akustisches und visuelles Alarmsystem vollständig deaktivieren
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(REDLED_PIN, LOW);
      digitalWrite(GREENLED_PIN, LOW);
      Serial.println("[INFO] Task: Alarm - Stopped");
      Serial.println();
    }


    // 2. Überprüfen des TinyML-Status
    if (isTinyMLTaskEnabled && TinyMLTask_handle == NULL) {
      // Wenn Flag true ist und Aufgabe nicht läuft, traditionelle Erkennungsmethoden-bezogene Aufgaben löschen und TinyML-Aufgabe starten

      // Traditionelle Erkennungsaufgaben in Reihenfolge löschen
      if (SerialPrintTask_handle != NULL) {
        vTaskDelete(SerialPrintTask_handle);
        SerialPrintTask_handle = NULL;
        Serial.println("[INFO] Task: Serial Print - Stopped");
        Serial.println();
      }

      if (CheckGasTask_handle != NULL) {
        vTaskDelete(CheckGasTask_handle);
        CheckGasTask_handle = NULL;
        Serial.println("[INFO] Task: Check Gas - Stopped");
        Serial.println();
      }

      if (ReadGasTask_handle != NULL) {
        vTaskDelete(ReadGasTask_handle);
        ReadGasTask_handle = NULL;
        Serial.println("[INFO] Task: Read Gas - Stopped");
        Serial.println();
      }

#if USE_MUTEX
      // Überprüfen, ob I2C-Mutex belegt ist, und freigeben, wenn belegt
      if (xSemaphoreTake(i2cMutex_handle, 0) == pdTRUE) {
        xSemaphoreGive(i2cMutex_handle);
      }

      // Überprüfen, ob SPI-Mutex belegt ist, und freigeben, wenn belegt
      if (xSemaphoreTake(spiMutex_handle, 0) == pdTRUE) {
        xSemaphoreGive(spiMutex_handle);
      }
#endif

      // // Alle alarmbezogenen Flags zurücksetzen
      // isTotalAlarmState = false;          // Gesamter Alarmstatus
      // isLastTotalAlarmState = false;  // Letzter Gesamter Alarmstatus
      // for (int i = 0; i < 6; ++i) {
      //   isAlarmState[i] = false;      // Alarmstatus für jedes Gas
      //   isTriangleDrawn[i] = false;   // Ob das rote Dreieck für jedes Gas bereits gezeichnet wurde
      // }

      // TinyML-Inferenzaufgabe erstellen
      xTaskCreate(
        TinyMLTask_func,     // Aufgabenfunktion
        "TinyML Inference",  // Aufgabenname
        8192,                // Stapelgröße
        NULL,                // Aufgabenparameter
        4,                   // Priorität
        &TinyMLTask_handle   // Aufgabenhandle
      );

    } else if (!isTinyMLTaskEnabled && TinyMLTask_handle != NULL) {
      // Wenn Flag false ist und Aufgabe läuft, TinyML-Aufgabe löschen und traditionelle Erkennungsmethoden-bezogene Aufgaben starten

      // TinyML-Aufgabe löschen
      vTaskDelete(TinyMLTask_handle);
      TinyMLTask_handle = NULL;

#if USE_MUTEX
      // Überprüfen, ob I2C-Mutex belegt ist, und freigeben, wenn belegt
      if (xSemaphoreTake(i2cMutex_handle, 0) == pdTRUE) {
        xSemaphoreGive(i2cMutex_handle);
      }

      // Überprüfen, ob SPI-Mutex belegt ist, und freigeben, wenn belegt
      if (xSemaphoreTake(spiMutex_handle, 0) == pdTRUE) {
        xSemaphoreGive(spiMutex_handle);
      }
#endif

      Serial.println("[INFO] Task: TinyML Inference - Stopped");
      Serial.println();

      // Traditionelle Erkennungsaufgaben in Reihenfolge neu erstellen
      // Aufgabe erstellen: Gaskonzentration vom Sensor lesen
      xTaskCreate(
        ReadGasTask_func,    // Aufgabenfunktion
        "Read Gas Sensor",   // Aufgabenname
        2048,                // Stapelgröße
        NULL,                // Aufgabenparameter
        4,                   // Priorität (je höher die Zahl, desto höher die Priorität)
        &ReadGasTask_handle  // Aufgabenhandle
      );

      // Aufgabe erstellen: Überprüfen, ob die Konzentration überschritten ist
      xTaskCreate(
        CheckGasTask_func,           // Aufgabenfunktion
        "Check Gas Concentrations",  // Aufgabenname
        2048,                        // Stapelgröße
        NULL,                        // Aufgabenparameter
        4,                           // Priorität
        &CheckGasTask_handle         // Aufgabenhandle
      );

      // Wenn serielle Verbindung bei Initialisierung vorhanden, Aufgabe erstellen: Serielle Protokollausgabe
      if (isSerialConnected) {
        xTaskCreate(
          SerialPrintTask_func,    // Aufgabenfunktion
          "Serial Print Output",   // Aufgabenname
          2048,                    // Stapelgröße
          NULL,                    // Aufgabenparameter
          1,                       // Priorität
          &SerialPrintTask_handle  // Aufgabenhandle
        );
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Warten, damit die Aufgabe mit der angegebenen Frequenz ausgeführt wird
  }
}


/* Aufgabe: Überprüfen des Batteriestatus ------------------------------------------------------------------*/
void CheckBatteryTask_func(void *parameter) {

  Serial.println("[INFO] Task: Check Battery - Started");
  Serial.println();

  // Initialisierung der ADC-Einstellungen
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);
  analogContinuous(ADC_PINS, ADC_PINS_COUNT, CONVERSIONS_PER_PIN, ADC_SAMPLING_FREQ, NULL);

  // Aufgabe einmalig beim Start ausführen
  CheckBattery();  // Ziel-Funktion aufrufen

  for (;;) {
    // Warten auf Timer-Benachrichtigung (wenn keine Benachrichtigung eingeht, bleibt die Funktion blockiert, beansprucht jedoch keine CPU-Ressourcen und beeinträchtigt daher nicht den Ablauf anderer Aufgaben)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Ziel-Funktion aufrufen
    CheckBattery();
  }
}


/* Timer: Callback-Funktion für die Aufgabe CheckBattery ------------------------------------------------------------------*/
void batteryTimerCallback(TimerHandle_t xTimer) {
  // Batterieüberwachungsaufgabe benachrichtigen
  xTaskNotifyGive(CheckBatteryTask_handle);
}




/* ========================================================== Arduino-Hauptinitialisierungs- und Hauptschleifenbereich ========================================================== */


/* Hauptinitialisierung ------------------------------------------------------------------*/
void setup() {

  // OLED-Bildschirm initialisieren
  setupOLED();
  // Logo anzeigen
  oled.drawImage(0, 0, &logo);

  // Versuche, serielle Verbindung herzustellen und Hinweise zu drucken
  if (connectSerialWithRetry()) {
    // Systeminitialisierungsinformationen drucken
    Serial.println();
    Serial.println(">>>>>>>>>> The system is initializing <<<<<<<<<<");
    Serial.println();
    vTaskDelay(pdMS_TO_TICKS(750));  // Verzögerung, um sicherzustellen, dass die Anzeigedauer des Logos mit der ohne serielle Verbindung übereinstimmt
  }

  // Initialisiere die Pin-Modi der LEDs und des Summers
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(REDLED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialisiere die Pin-Modi der drei Tasten
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);

  // Gassensor initialisieren
  setupMiCS();

  // Beende die Anzeige des Logos und lösche den Bildschirm
  oled.clearScreen();

#if USE_MUTEX
  // Erstellen von Mutex
  i2cMutex_handle = xSemaphoreCreateMutex();
  spiMutex_handle = xSemaphoreCreateMutex();
  // Überprüfen, ob Mutex erstellt wurde, andernfalls System neu starten
  if (i2cMutex_handle == NULL || spiMutex_handle == NULL) {
    Serial.println("[CRITICAL] I2C / SPI Mutex Creation Failed!");
    Serial.println();
    // ESP-System neu starten
    Serial.printf("[WARN] System Restarting!\n\n\n\n\n");
    ESP.restart();
  } else {
    Serial.println("[INFO] I2C & SPI Mutex Created");
    Serial.println();
  }
#endif

  // Aufgabe erstellen: Aufwärmbildschirm anzeigen
  xTaskCreate(
    loadingScreenTask_func,    // Aufgabenfunktion
    "Loading Screen Task",     // Aufgabenname
    2048,                      // Stapelgröße (in Bytes)
    NULL,                      // Aufgabenparameter
    1,                         // Priorität
    &LoadingScreenTask_handle  // Aufgabenhandle
  );

  // Überprüfen, ob Aufwärmbildschirmaufgabe erstellt wurde, andernfalls neu starten
  if (LoadingScreenTask_handle != NULL) {
    Serial.println("[INFO] Task: Startup Loading Screen - Started");
    Serial.println();
  } else {
    Serial.println("[CRITICAL] Task: Startup Loading Screen - Creation failed!");
    Serial.println();
    // ESP-System neu starten
    Serial.printf("[WARN] System Restarting!\n\n\n\n\n");
    ESP.restart();
  }

  vTaskDelay(pdMS_TO_TICKS(1000));

  // WiFi initialisieren und verbinden
  initWiFi();

  // RTC initialisieren
  setupOutRTC();

  if (isOutRtcConnected) {
    // Aufgabe erstellen: Jede Sekunde Echtzeit abrufen
    xTaskCreate(
      GetTimeAndTempTask_func,    // Aufgabenfunktion
      "Get Time And Temp",        // Aufgabenname
      1024,                       // Stapelgröße
      NULL,                       // Aufgabenparameter
      2,                          // Priorität, sollte höher sein als die zu kontrollierenden Aufgaben
      &GetTimeAndTempTask_handle  // Aufgabenhandle
    );
  }

  // SD-Karten-Logger initialisieren
  setupSDLogger();

  //// ADC-bezogene Initialisierung
  // pinMode(BATTERY_VOLTAGE_PIN, INPUT);  // Initialisiere die Pin-Modus des Spannungssensors (tatsächlich ist es nicht erforderlich, den Pin-Modus explizit als Analogeingang zu setzen, da die Analogeingangspins des ESP32-C6 standardmäßig in diesem Modus sind)
  // Aufgabe erstellen: Batteriestatus überprüfen
  xTaskCreate(
    CheckBatteryTask_func,    // Aufgabenfunktion
    "Check Battery Status",   // Aufgabenname
    4096,                     // Stapelgröße
    NULL,                     // Aufgabenparameter
    1,                        // Priorität
    &CheckBatteryTask_handle  // Aufgabenhandle
  );
  // Timer erstellen
  CheckBatteryTimer_handle = xTimerCreate("BatteryTimer", pdMS_TO_TICKS(BAT_CHECK_PERIOD_MS), pdTRUE, 0, batteryTimerCallback);
  xTimerStart(CheckBatteryTimer_handle, 0);  // Timer starten
}


/* Hauptschleifenfunktion ------------------------------------------------------------------*/
void loop() {
  taskYIELD();                    // CPU-Zeit aktiv anderen Aufgaben überlassen
  vTaskDelay(pdMS_TO_TICKS(10));  // Kurze Verzögerung möglich, hilft, den Stromverbrauch zu senken
}
