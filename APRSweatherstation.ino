/************************************************************
*****                  APRSwaterstation                  ****                        
*****                 by Don_Epel                        ****
*************************************************************/

/********************************
// ==== CHANGELOG ==== //
Version 0.1
  *Wifi 
  *DHT11
  *NTP
  Esta version envia los parametros solo internet 



*/

/* ***************************************************/
#include <ESP8266WiFi.h>
#include <DHT.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// ===== DEFINICIONES ===== //
// WiFi
const char* ssid = "nombredelawifi";
const char* password = "clave";

// APRS-IS
const char* aprsServer = "rotate.aprs2.net"; // Servidor más confiable
const int aprsPort = 14580; //puerto, no cambiar
String callsign = "callsing-13"; //su señal distintiva, use con responsabilidad
String passcode = "12322"; //clave aprs, lo obtienes de https://apps.magicbug.co.uk/passcode/


#define TX_DELAY 500

// Sensor DHT11
#define DHTPIN D2
#define DHTYPE DHT11
DHT dht(DHTPIN, DHTYPE);

// Posición fija
float manualLat = -11.1111;
float manualLon = -11.1111;

// NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ar.pool.ntp.org", -3 * 3600); // UTC-3 con servidor local

// Variables de tiempo
unsigned long lastPosTime = 0;
unsigned long lastWiFiCheck = 0;
String lastValidTime = "0000z";





// ===== FUNCIONES PRINCIPALES ===== //

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Conectando...");
  
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Conectado a: " + String(ssid));
    Serial.println("[WiFi] IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[ERROR] WiFi no conectado. Modo offline.");
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Reconectando...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n[WiFi] Reconexión exitosa");
      timeClient.forceUpdate(); // Resincronizar hora
    }
  }
}


void setupNTP() {
  Serial.println("[NTP] Iniciando cliente NTP...");
  timeClient.begin();
  timeClient.setTimeOffset(0); // UTC
  timeClient.setPoolServerName("pool.ntp.org");

  if (WiFi.status() == WL_CONNECTED) {
    for (int i = 0; i < 3; i++) {
      if (timeClient.update()) {
        Serial.println("[NTP] Sincronización exitosa.");
        return;
      }
      delay(500);
    }
    Serial.println("[NTP] Falló la sincronización NTP.");
  } else {
    Serial.println("[NTP] WiFi no conectado, no se puede iniciar NTP.");
  }
}


String getAPRSTimestamp() {
  static unsigned long lastUpdate = 0;
  
  // Cache de 1 minuto para evitar consultas frecuentes
  if (millis() - lastUpdate < 60000 && lastValidTime != "000000z") {
    return lastValidTime;
  }

  // Intenta obtener hora NTP
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nIntenta obtener hora NTP");
    for (int i = 0; i < 3; i++) { // 3 intentos
      if (timeClient.update()) {
        String timeStr = timeClient.getFormattedTime();
        lastValidTime = timeStr.substring(0,2) + timeStr.substring(3,5) + timeStr.substring(6, 8) + "z";
        lastUpdate = millis();
        Serial.println("[NTP] Hora actual: " + timeStr + " -> " + lastValidTime);
        Serial.println("\nHora: " + timeStr);

        return lastValidTime;
      }
      delay(500);
    }
  }
  
  // Fallback: hora aproximada basada en millis()
  Serial.println("\n[NTP] Usando hora local aproximada");
  unsigned long secs = millis() / 1000;
  int hh = (secs / 3600) % 24;
  int mm = (secs / 60) % 60;
  char buffer[6];
  snprintf(buffer, sizeof(buffer), "%02d%02dz", hh, mm);
  //Serial.print("\nHora local:);
  Serial.println("\nHoralocal: " + String(hh) + ":" + String(mm));

  return String(buffer);
}

String convertToAPRSCoord(float value, bool isLat) {
  char buffer[10];
  int degrees = int(abs(value));
  float minutes = (abs(value) - degrees) * 60;
  
  // Formato exacto requerido por APRS
  snprintf(buffer, sizeof(buffer), 
          isLat ? "%02d%05.2f" : "%03d%05.2f", 
          degrees, minutes);
  
  return String(buffer) + (isLat ? (value >= 0 ? "N" : "S") : (value >= 0 ? "E" : "W"));
}
// ==== APRS  GPS Y EStaCIoN==== //
void sendAPRSPacket() {
  String timestamp = getAPRSTimestamp();
  String  posPacket = callsign + ">APE32I,WIDE1-1,TCPIP,qAR," + callsign + ":";
          posPacket += "!"; 
          posPacket += convertToAPRSCoord(manualLat, true) + "/" + convertToAPRSCoord(manualLon, false);
          posPacket += timestamp;
          posPacket += "["; //icono de persona
          posPacket += "PHG2010"; //datos de la estacion como potencia y antena
          posPacket += "_000/000g000t072r000p000h50b10215"; // Datos meteorológicos de prueba
          posPacket += "/Estacion experimental movil, en pruebas";
  sendPacket(posPacket);
}

// ==== APRS  Clima ==== //
void sendAPRSWX() {
  String timestamp = getAPRSTimestamp();  // "hhmmssz"
  
   float tempC = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  if(isnan(tempC)) tempC = 0;
  if(isnan(humidity)) humidity = 0;

  // Conversión y redondeo
  int tempF = round(tempC * 1.8 + 32);
  int hum = round(humidity);
  
  String WXPacket = callsign + ">APE32I,WIDE1-1,TCPIP,qAR," + callsign + ":";

  WXPacket += "@" + timestamp ;
  WXPacket += convertToAPRSCoord(manualLat, true) + "/";
  WXPacket += convertToAPRSCoord(manualLon, false);
  
  //Datos metereologicos
  WXPacket += "_";  // Indicador de bloque de datos meteorológicos
  WXPacket += "000/000";  // direccion/velocidad
  WXPacket += "g000";  // rafaga maxima.
  WXPacket += "t";         // Temperatura
  if(tempF < 100) WXPacket += "0";  // Asegura 3 dígitos
  WXPacket += String(tempF);
  WXPacket += "r000p000P000"; // Lluvia
  WXPacket += "h";
  if(hum < 10) WXPacket += "0";  // Asegura 2 dígitos
  WXPacket += String(hum);
  WXPacket += "b00000";  // presion atmosferica
  
  // Comentario adicional (opcional)
  WXPacket += "WX node Buenos Aires";

  sendPacket(WXPacket);
}

void sendPacket(String packet) {
  Serial.println("[ENVIO] " + packet);
  
  // Limpieza del paquete
  packet.trim();
  
  // Envío por radio (opcional)
  String radioPacket = packet;
  radioPacket.replace(",TCPIP*", "");
  // sendToRadio(radioPacket);
  
  // Envío por internet
  sendToAPRSIS(packet);
}

void sendToAPRSIS(String packet) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[APRS-IS] WiFi no conectado");
    return;
  }

  WiFiClient client;
  if (client.connect(aprsServer, aprsPort)) {
    // Envía credenciales
    client.print("user ");
    client.print(callsign);
    client.print(" pass ");
    client.print(passcode);
    client.print(" vers ESP8266-APRS 1.0");
    client.print("\n");
    
    // Espera breve
    delay(100);
    
    // Envía el paquete
    client.print(packet);
    client.print("\n");
    
    // Cierra conexión
    delay(50);
    client.stop();
    Serial.println("[APRS-IS] Paquete enviado");
  } else {
    Serial.println("[APRS-IS] Error de conexión");
  }
}



// ===== SETUP ===== //
void setup() {
  Serial.begin(115200);
  while (!Serial); // Espera a que el monitor serial esté listo
  
  // Configuración hardware
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, HIGH);
  pinMode(AUDIO_PIN, OUTPUT);
  noTone(AUDIO_PIN);

  // Inicialización de componentes
  dht.begin();
  connectToWiFi();
  setupNTP();

  Serial.println("\n[INFO] Estación APRS iniciada (DHT11)");
  // Test inicial del sensor DHT
  Serial.println("[DHT] Probando sensor...");
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  
  if (isnan(t) || isnan(h)) {
    Serial.println("[ERROR] No se pudo leer el sensor DHT!");
  } else {
    Serial.printf("[DHT] Lectura inicial: %.1f°C, %.1f%% HR\n", t, h);
  }

}

// ===== LOOP PRINCIPAL ===== //
void loop() {
  // Verifica conexión WiFi periódicamente
  if (millis() - lastWiFiCheck > 30000) 
  {
    checkWiFiConnection();
    lastWiFiCheck = millis();
  }
  
  // Envía paquete APRS cada 60 segundos
  if (millis() - lastPosTime > 60000) {
    sendAPRSWX(); //Datos de clima
    lastPosTime = millis();
  }
  

  delay(1000);
}
