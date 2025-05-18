/************************************************************
*****               APRSweatherstation                  ****                        
*****                 by Don_Epel                       ****
*************************************************************/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Ticker.h>
#include <SPI.h>
#include <SD.h>

// =============== CHANGELOG ==============//
/*
 * version 0.2 - 18/05/2025
 * se cambia modulo de temperatura y humedad dht11 por uno mas preciso AHT10
 * Se agrega modulo de memoria SD para cache cuando no hay internet, al regresar se envia toda la informacion
 * Se agraga led destellante de estado
 * Se agrega watchdot por software ( no es lo mejor, pero es una mejora) 
 */

// ================= CONFIGURACIÓN DE PINES ================= //
/*
 * CONEXIONES:
 * 
 * MÓDULO SD:
 *   CS   → D8 (GPIO15)
 *   MOSI → D7 (GPIO13)
 *   MISO → D6 (GPIO12)
 *   SCK  → D5 (GPIO14)
 *   VCC  → 5V (Vin/VU) *Si el módulo tiene regulador LM1117*
 *   GND  → GND
 * 
 * SENSOR AHT10:
 *   VCC  → 3.3V
 *   GND  → GND
 *   SDA  → D2 (GPIO4)
 *   SCL  → D1 (GPIO5)
 * 
 * LED:
 *   LED_BUILTIN → D4 (GPIO2)
 */
// ================= CONFIGURACIÓN EDITABLE ================= //

// ----- CONEXIONES WIFI ----- //
const char* ssid = "wifi";       // Nombre de red WiFi
const char* password = "pass";         // Contraseña WiFi

// ----- CREDENCIALES APRS ----- //
const char* aprsServer = "rotate.aprs2.net";  // Servidor APRS
const int aprsPort = 14580;                   // Puerto estándar
String callsign = "distintica-13";                // Indicativo APRS
String passcode = "clave";                    // Clave APRS

// ----- UBICACIÓN GEOGRÁFICA ----- //
const float manualLat = -11.1111;  // Latitud (Sur negativo)
const float manualLon = -11.1111;  // Longitud (Oeste negativo)

// ----- HARDWARE ----- //
#define SD_CS    D8  // Pin ChipSelect para módulo MicroSD (GPIO15)
#define LED_PIN  D4  // Pin del LED interno (GPIO2)

// ----- INTERVALOS ----- //
const unsigned long TX_INTERVAL = 300000;  // 5 minutos entre transmisiones
const unsigned long WIFI_CHECK = 30000;    // 30 segundos entre chequeos WiFi
const unsigned long WDT_TIMEOUT = 60;      // Timeout del watchdog (segundos)

// ================= NO MODIFICAR ================= //
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ar.pool.ntp.org", -3 * 3600);
Ticker watchdog;
Adafruit_AHTX0 aht;

unsigned long lastTxTime = 0;
unsigned long lastWiFiCheck = 0;
bool sdReady = false;
bool ahtReady = false;

// ================= FUNCIONES ================= //

void initWatchdog() {
  watchdog.attach(WDT_TIMEOUT, []() { ESP.restart(); });
}

bool initSD() {
  SPI.begin();
  if(!SD.begin(SD_CS)) {
    Serial.println("[SD] Error de inicialización");
    return false;
  }
  return true;
}

bool initAHT() {
  Wire.begin();
  if(!aht.begin()) {
    Serial.println("[AHT] Sensor no detectado");
    return false;
  }
  return true;
}

String getTimestamp() {
  if(timeClient.update()) {
    String time = timeClient.getFormattedTime();
    return time.substring(0,2) + time.substring(3,5) + time.substring(6,8) + "z";
  }
  return "000000z";
}

String convertToAPRSCoord(float value, bool isLat) {
  char buffer[10];
  int degrees = int(abs(value));
  float minutes = (abs(value) - degrees) * 60;
  snprintf(buffer, sizeof(buffer), isLat ? "%02d%05.2f" : "%03d%05.2f", degrees, minutes);
  return String(buffer) + (isLat ? (value >= 0 ? "N" : "S") : (value >= 0 ? "E" : "W"));
}

String buildPacket(float temp, float hum) {
  int tempF = round(temp * 1.8 + 32);
  tempF = constrain(tempF, -99, 999);
  int humRounded = constrain(round(hum), 0, 100);

  String packet = callsign + ">APE32I,WIDE1-1,TCPIP,qAR," + callsign + ":";
  packet += "@" + getTimestamp();
  packet += convertToAPRSCoord(manualLat, true) + "/";
  packet += convertToAPRSCoord(manualLon, false);
  packet += "_000/000g000t";
  
  if(tempF < 100) packet += "0";
  if(tempF < 10) packet += "0";
  packet += String(tempF);
  
  packet += "r000p000P000h";
  if(humRounded < 10) packet += "0";
  packet += String(humRounded);
  packet += "b00000WX-distintiva-13";

  Serial.printf("[WX] Temp: %.1fC (t%03dF) Hum: %d%%\n", temp, tempF, humRounded);
  return packet;
}

void sendToAPRSIS(String packet) {
  WiFiClient client;
  if(client.connect(aprsServer, aprsPort)) {
    client.print("user " + callsign + " pass " + passcode + " vers APRSWX-1.0\n");
    delay(100);
    client.print(packet + "\n");
    client.stop();
    Serial.println("[APRS] Transmisión exitosa");
  } else {
    Serial.println("[APRS] Error de conexión");
  }
}

// ================= SETUP ================= //
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  initWatchdog();
  
  Serial.println("\nIniciando APRSweatherstation");
  Serial.println("Inicializando hardware...");

  sdReady = initSD();
  ahtReady = initAHT();

  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Conectado");

  timeClient.begin();
  timeClient.update();

  Serial.println("Sistema listo");
  digitalWrite(LED_PIN, LOW);
}

// ================= LOOP ================= //
void loop() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  if(millis() - lastWiFiCheck > WIFI_CHECK) {
    if(WiFi.status() != WL_CONNECTED) WiFi.reconnect();
    lastWiFiCheck = millis();
  }

  if(millis() - lastTxTime > TX_INTERVAL) {
    if(ahtReady) {
      sensors_event_t humidity, temp;
      if(aht.getEvent(&humidity, &temp)) {
        sendToAPRSIS(buildPacket(temp.temperature, humidity.relative_humidity));
      }
    }
    lastTxTime = millis();
  }

  delay(1000);
}s
