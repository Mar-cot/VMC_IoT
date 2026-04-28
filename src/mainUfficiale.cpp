#include <Arduino.h>
#include <Relay.h>
#include <MySensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include "secrets.h"


#define DEUM_VALVE_OPEN_PIN 12
#define DEUM_VALVE_CLOSE_PIN 13
#define EXTERN_VALVE_OPEN_PIN 14
#define EXTERN_VALVE_CLOSE_PIN 27
#define MOTOR_TIME 15000
#define SAMPLE_DELAY 210000

#define FAN_PIN 25
#define DEHUMIDIFIER_PIN 26

#define SENSORE_ESTERNO_PIN 33
#define SENSORE_INTERNO_PIN 32

#define SOGLIA_UMIDITA 59.00f
#define TEMP_MIN_ESTERNO 12.00f
#define TEMP_MAX_ESTERNO 27.00f


WebServer server(80);



RelayMotore* rValvDeumidificatore = NULL;
RelayMotore* rValvEsterno = NULL;

SingleRelay* rVentolaDeum = NULL;
SingleRelay* rDeumidificatore = NULL;

Sensor* sensoreEsterno = NULL;
Sensor* sensoreInterno = NULL;



typedef struct ValoriSensori
{
    float tempInterno;
    float tempEsterno;
    float umidInterno;
    float umidEsterno;
    int lastReadInterno;
    int lastReadEsterno;
} ValoriSensori;

ValoriSensori* vs = NULL;



void apriValvolaEsterno();
void chiudiValvolaEsterno();
void accendiDeumidificatore();
void spegniDeumidificatore();
void accendiVentola();
void spegniVentola();
void apriValvolaDeumidificatore();
void chiudiValvolaDeumidificatore();

void handleRoot();

void TaskSensori(void* param);
void TaskSensoriInterno(void* param);
void TaskSensoriEsterno(void* param);

void serverTask(void* parameter) {
    Serial.print("Handling server; Server Status:");
    Serial.println(WiFi.status());
  while (true) {
    if(!WiFi.isConnected()){
      Serial.println("Disconnesso :(\nRiconnessione...");
      while(WiFi.reconnect());
      Serial.println("Connessione ripristinata!");
    }

    server.handleClient();  // Gestisce le richieste HTTP
    delay(100);              // Breve ritardo per evitare uso eccessivo della CPU
  }
}
void ElegantOTATask(void* parameter) {
    Serial.println("Handling OTA");
  while (true) {
    ElegantOTA.loop();  // Chiamata ricorrente per gestire OTA
    delay(100);          // Breve ritardo per evitare utilizzo eccessivo della CPU
  }
}
//Caso Base Ricircolo no deum;
TaskHandle_t task_sensori_handle;
TaskHandle_t task_server_handle;
TaskHandle_t task_ota_handle;


void setup(){
    Serial.begin(112500);
    WiFi.setHostname("vmccotogni.org");
    WiFi.begin(ssid, password);
    Serial.print("Connessione a: ");
    Serial.println(ssid);
  
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connessione in corso...");
    }
  
    Serial.println("Connesso al WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    vs = (ValoriSensori*) calloc(1,sizeof(ValoriSensori));
    vs->tempEsterno = CANC_NUM;
    vs->umidEsterno = CANC_NUM;
    vs->tempInterno = CANC_NUM;
    vs->umidInterno = CANC_NUM;
    vs->lastReadEsterno = 0;
    vs->lastReadInterno = 0;

    // Configurazione delle route del server
    server.on("/", handleRoot);
  
    // Inizializzazione di ElegantOTA
    ElegantOTA.begin(&server);
    server.begin();

    sensoreEsterno = new Sensor(SENSORE_ESTERNO_PIN);
    sensoreInterno = new Sensor(SENSORE_INTERNO_PIN);

    auto ret = xTaskCreatePinnedToCore(TaskSensoriEsterno, "Task Sensori", 4096, vs, 1, &task_sensori_handle, 0);
    if(ret != pdPASS){
        Serial.println("Errore nella creazione della task sensori");
    }

    ret = xTaskCreatePinnedToCore(TaskSensoriInterno, "Task Sensori", 4096, vs, 1, NULL, 0);
    if(ret != pdPASS){
        Serial.println("Errore nella creazione della task sensori");
    }

    ret = xTaskCreatePinnedToCore(ElegantOTATask, "OTATask", 4096, NULL, 3, &task_ota_handle, 0);
    if(ret != pdPASS){
        Serial.println("Errore nella creazione della task OTA");
    }
    
    rVentolaDeum = new SingleRelay(FAN_PIN, 0);
    rDeumidificatore = new SingleRelay(DEHUMIDIFIER_PIN, 0);
    rValvDeumidificatore = new RelayMotore(DEUM_VALVE_OPEN_PIN,DEUM_VALVE_CLOSE_PIN,MOTOR_TIME,true);
    rValvEsterno = new RelayMotore(EXTERN_VALVE_OPEN_PIN,EXTERN_VALVE_CLOSE_PIN,MOTOR_TIME);

    ret = xTaskCreatePinnedToCore(serverTask, "ServerTask", 4096, NULL, 1, &task_server_handle, 0);
    if(ret != pdPASS){
        Serial.println("Errore nella creazione della task server");
    }

}

void loop(){ 

    float tempEsterno = vs->tempEsterno;
    float tempInterno = vs->tempInterno;
    float umiditaEsterno = vs->umidEsterno;
    float umiditaInterno = vs->umidInterno;

    //Gestione errore lettura sensori, se a causa del sensore esterno inverte due volte la valvola esterna, se a causa del sensore interno inverte due volte la valvola del deumidificatore

    Serial.printf("Sensore Interno || Temperatura %f°C, Umidità %f%\n", tempInterno, umiditaInterno);
    Serial.printf("Sensore Esterno || Temperatura %f°C, Umidità %f%\n", tempEsterno, umiditaEsterno);

    if(tempEsterno == CANC_NUM || tempInterno == CANC_NUM || umiditaEsterno == CANC_NUM || umiditaInterno == CANC_NUM){
      delay(sensoreEsterno->delayus/250);
      return;
    }


    if((umiditaEsterno <= SOGLIA_UMIDITA) && (TEMP_MAX_ESTERNO>=tempEsterno) && (tempEsterno>=TEMP_MIN_ESTERNO)){
        Serial.println("Prendo aria dall'esterno");
        apriValvolaEsterno();
        if(umiditaInterno >= SOGLIA_UMIDITA - 10 ){
            Serial.println("Deumidifico");
            apriValvolaDeumidificatore();
            accendiDeumidificatore();
        }
        else{
            Serial.println("Non Deumidifico");
            chiudiValvolaDeumidificatore();
        }
    }
    else {
        Serial.println("Non prendo aria dall'esterno");
        chiudiValvolaEsterno();
        if(umiditaInterno >= SOGLIA_UMIDITA -10 ){
            Serial.println("Deumidifico");
            accendiDeumidificatore();
        }
        else{
            Serial.println("Non Deumidifico");

            spegniDeumidificatore();

        }
    }

    delay(SAMPLE_DELAY);

    /*

    rValvDeumidificatore->invertValve();
    rValvEsterno->invertValve();

    //test sensori
    rVentolaDeum->turnOn();
    rDeumidificatore->turnOn();
    delayMicroseconds(sensoreEsterno->delayus);
    rVentolaDeum->turnOff();
    rDeumidificatore->turnOff();
    delayMicroseconds(sensoreEsterno->delayus);
    Serial.printf("Sensore esterno\n Temperatura: %f\tUmidita:%f\n", sensoreEsterno->getTemperature(), sensoreEsterno->getHumidity());
    Serial.printf("Sensore interno\n Temperatura: %f\tUmidita:%f\n", sensoreInterno->getTemperature(), sensoreInterno->getHumidity());
    if(sensoreEsterno->getHumidity() != CANC_NUM && sensoreEsterno->getTemperature()< 50) rVentolaDeum->turnOn();
    if(sensoreInterno->getHumidity() != CANC_NUM && sensoreInterno->getTemperature()< 50) rDeumidificatore->turnOn();
    delayMicroseconds(sensoreEsterno->delayus);
    */
}

void apriValvolaEsterno(){
    rValvEsterno->openValve();
}
void chiudiValvolaEsterno(){
    if(!rValvDeumidificatore->is_open) apriValvolaDeumidificatore();
    rValvEsterno->closeValve();
}


void accendiDeumidificatore(){
    if(!rValvDeumidificatore->is_open) apriValvolaDeumidificatore();
    rVentolaDeum->turnOn();
    rDeumidificatore->turnOn();
}

void spegniDeumidificatore(){
    rDeumidificatore->turnOff();
    //spegniVentola();
}


void apriValvolaDeumidificatore(){
    rValvDeumidificatore->openValve();
    rVentolaDeum->turnOn();
}

void chiudiValvolaDeumidificatore(){
    if(!rValvEsterno->is_open) apriValvolaEsterno();
    spegniDeumidificatore();
    rVentolaDeum->turnOff();
    rValvDeumidificatore->closeValve();
}

void spegniVentola(){
    rVentolaDeum->turnOff();
}


void TaskSensori(void* param){
    ValoriSensori* valori = (ValoriSensori*) param;
    while(true){
      float ret = sensoreEsterno->getHumidity();
        if(!(ret > 100 || ret < 0))
        {
          valori->tempEsterno = sensoreEsterno->getTemperature();
          valori->umidEsterno = sensoreEsterno->getHumidity();
        }
        
      ret = sensoreInterno->getHumidity();
        if(!(ret > 100 || ret < 0)){
          valori->tempInterno = sensoreInterno->getTemperature();
          valori->umidInterno = sensoreInterno->getHumidity();
        }

        Serial.printf("tempEsterno: %f°C\tumidEsterno: %f%\ttempInterno: %f°C\tumidInterno: %f%\n",
                    valori->tempEsterno, valori->umidEsterno, valori->tempInterno, valori->umidInterno);
        delay(sensoreEsterno->delayus/250);

    }

}


void TaskSensoriEsterno(void* param){
    ValoriSensori* valori = (ValoriSensori*) param;
    while(true){
      float ret = sensoreEsterno->getHumidity();
        if(!(ret > 100 || ret < 0))
        {
          valori->tempEsterno = sensoreEsterno->getTemperature();
          valori->umidEsterno = sensoreEsterno->getHumidity();
          valori->lastReadEsterno = 0;
        }
        else{
          valori->lastReadEsterno += 1;
        }
        

        Serial.printf("tempEsterno: %f°C\tumidEsterno: %f%\ttempInterno: %f°C\tumidInterno: %f%\n",
                    valori->tempEsterno, valori->umidEsterno, valori->tempInterno, valori->umidInterno);
        delay(sensoreEsterno->delayus/1000);

    }

}

void TaskSensoriInterno(void* param){
    ValoriSensori* valori = (ValoriSensori*) param;
    while(true){
      float ret;
      ret = sensoreInterno->getHumidity();
        if(!(ret > 100 || ret < 0)){
          valori->tempInterno = sensoreInterno->getTemperature();
          valori->umidInterno = sensoreInterno->getHumidity();
          valori->lastReadInterno = 0;
        }
        else{
          valori->lastReadInterno += 1;
        }

        Serial.printf("tempEsterno: %f°C\tumidEsterno: %f%\ttempInterno: %f°C\tumidInterno: %f%\n",
                    valori->tempEsterno, valori->umidEsterno, valori->tempInterno, valori->umidInterno);
        delay(sensoreInterno->delayus/1000);

    }

}

void handleRoot() {
/*
  if (vs->tempEsterno == CANC_NUM || vs->tempInterno == CANC_NUM) {
    server.send(500, "text/plain", "Errore nella lettura dei sensori");
    return;
  }
*/  
  Serial.println("Chiamata dal client ricevuta");
  String html= String("<HTML><body>");
  html += "<h1>VMC CASA - ESP32 Web Server</h1>";
  html +="<h2>Lettura Sensori</h2>";
  html += "<style>table {border-collapse:collapse}td, th {border:2px solid #ddd;padding:8px;}</style><table>";
  html += "<tr><td><b>Sensore</b> </td>";
  html += "<td><b>Temperatura</b></td>";
  html += "<td><b>Umidit&#224;</b></td>";
  html += "<td><b>Ultima Lettura</b></td></tr>";

  html +="<tr><td><FONT COLOR=\"blue\"><b>Esterno     </b></FONT></td>";
  html +="<td><FONT COLOR=\"blue\"><b>"+String(vs->tempEsterno)+"&#8451;</b></FONT></td>";
  html +="<td><FONT COLOR=\"blue\"><b>"+ String(vs->umidEsterno)+"%</b></FONT></td>";
  html +="<td><FONT COLOR=\"blue\"><b>"+ String((float)vs->lastReadEsterno * (sensoreEsterno->delayus/1000000))+"s</b></FONT></td></tr>";

  html +="<tr><td><FONT COLOR=\"green\"><b>Interno</b></FONT></td>";
  html +="<td><FONT COLOR=\"green\"><b>"+String(vs->tempInterno)+"&#8451;<FONT COLOR=\"green\"></b></FONT></td>";
  html +="<td><FONT COLOR=\"green\"><b>"+String(vs->umidInterno)+"%</b></FONT></td>";
  html +="<td><FONT COLOR=\"green\"><b>"+ String((float)vs->lastReadInterno * (sensoreEsterno->delayus/1000000))+"s</b></FONT></td></tr>";


  html +="</table>";


  html +="<h2>Stato del Sistema</h2>";

  html += "<table>";
  if(rValvEsterno->is_open)
    html += "<tr><td><b>Aspirazione Esterna</b></td><td><FONT COLOR=\"green\"><b>ON</b></FONT></td></tr>";
  else
    html += "<tr><td><b>Aspirazione Esterna</b></td><td><FONT COLOR=\"red\"><b>OFF</b></FONT></td></tr>";

  if(rValvDeumidificatore->is_open)
    html += "<tr><td><b>Aspirazione Deumidificatore</b></td><td><FONT COLOR=\"green\"><b>ON</b></FONT></td></tr>";
  else
    html += "<tr><td><b>Aspirazione Deumidificatore</b></td><td><FONT COLOR=\"red\"><b>OFF</b></FONT></td></tr>";

  if(rDeumidificatore->is_on)
    html += "<tr><td><b>Deumidificatore</b></td><td><FONT COLOR=\"green\"><b>ON</b></FONT></td></tr>";
  else
    html += "<tr><td><b>Deumidificatore</b></td><td><FONT COLOR=\"red\"><b>OFF</b></FONT></td></tr>";

  if(rVentolaDeum->is_on)
    html += "<tr><td><b>Ventola Deumidificatore</b></td><td><FONT COLOR=\"green\"><b>ON</b></FONT></td></tr>";
  else
    html += "<tr><td><b>Ventola Deumidificatore</b></td><td><FONT COLOR=\"red\"><b>OFF</b></FONT></td></tr>";

  html += "</table></body></html>";

  Serial.println("HTML GENERATO");
  server.send(200, "text/html", html);
  Serial.println("RISPOSTA INVIATA");
  
}
/*
void OldHtml(){
  /*
  if (vs->tempEsterno == CANC_NUM || vs->tempInterno == CANC_NUM) {
    server.send(500, "text/plain", "Errore nella lettura dei sensori");
    return;
  }  
  Serial.println("Chiamata dal client ricevuta");
  String html = "<html><head><title>ESP32 Web Server</title></head><body>";
  html += "<h1>ESP32 Web Server - Lettura Sensori</h1>";

    try
    {
    html += "<p><strong>Sensore Esterno:</strong><br>";

    html += "Temperatura: " + String(vs->tempEsterno) + " &#8451;<br>";
    html += "Umidità: " + String(vs->umidEsterno) + " %</br>";
    html += "Tempo dall'ultima lettura: " + String((float)vs->lastReadEsterno * (sensoreEsterno->delayus/1000000)) + "s</p>";
    
    html += "<p><strong>Sensore Interno:</strong><br>";
    html += "Temperatura: " + String(vs->tempInterno) + " &#8451;<br>";
    html += "Umidità: " + String(vs->umidInterno) + " %</br>";
    html += "Tempo dall'ultima lettura: " + String((float)vs->lastReadInterno * (sensoreEsterno->delayus/1000000)) + "s</p>";
    }
    catch(const std::exception& e)
    {
        Serial.print("Caught Exception: ");
        Serial.println(e.what());
        
        html += "<p><strong>Lettura Sensori Non Disponibile</strong><br></p>";
    }

  try{
    html += "<p><strong>Stato Valvola Esterna:</strong><br>";
    if(rValvEsterno->is_open){
      html += "Aperta</p>";
    }
    else{
      html += "Chiusa</p>";
    }

    html += "<p><strong>Stato Valvola Deumidificatore:</strong><br>";
    if(rValvDeumidificatore->is_open){
      html += "Aperta</p>";
    }
    else{
      html += "Chiusa</p>";
    }

    html += "<p><strong>Stato Deumidificazione:</strong><br>";
    if(rDeumidificatore->is_on){
      html += "Acceso</p>";
    }
    else{
      html += "Spento</p>";
    }

    html += "<p><strong>Stato Ventola:</strong><br>";
    if(rVentolaDeum->is_on){
      html += "Accesa</p>";
    }
    else{
      html += "Spenta</p>";
    }
  }
  catch(const std::exception& e){
      Serial.print("Caught Exception: ");
      Serial.println(e.what());
        
      html += "<p><strong>Lettura Valvole Non Disponibile</strong><br></p>";
  }

  
  html += "</body></html>";


}

void HandlerootOld(){

String html= String("<HTML><body>");
html += "<h1>VMC CASA - ESP32 Web Server</h1>";
html +="<h2>Lettura Sensori</h2>";
html += "<style>table {border-collapse:collapse}td, th {border:2px solid #ddd;padding:8px;}</style><table>";
html += "<tr><td><b>Sensore</b> </td>";
html += "<td><b>Temperatura</b></td>";
html += "<td><b>Umidità</b></td>";
html += "<td><b>Ultima Lettura</b></td></tr>";

html +="<tr><td><FONT COLOR=\"blue\"><b>Esterno     </b></FONT></td>";
html +="<td><FONT COLOR=\"blue\"><b>"+String(vs->tempEsterno)+ "°C</b></FONT></td>";
html +="<td><FONT COLOR=\"blue\"><b>"+ String(vs->umidEsterno)+"%</b></FONT></td>";
html +="<td><FONT COLOR=\"blue\"><b>"+ String((float)vs->lastReadEsterno * (sensoreEsterno->delayus/1000000))+"s</b></FONT></td></tr>";

html +="<tr><td><FONT COLOR=\"green\"><b>Interno</b></FONT></td>";
html +="<td><FONT COLOR=\"green\"><b>"+String(vs->tempInterno)+"<FONT COLOR=\"green\"></b></FONT></td>";
html +="<td><FONT COLOR=\"green\"><b>"+String(vs->umidInterno)+"%</b></FONT></td>";
html +="<td><FONT COLOR=\"green\"><b>"+ String((float)vs->lastReadInterno * (sensoreEsterno->delayus/1000000))+"s</b></FONT></td></tr>";


html +="</table>";


html +="<h2>Stato del Sistema</h2>";

html += "<table>";
if(rValvEsterno->is_open)
  html += "<tr><td><b>Aspirazione Esterna</b></td><td><FONT COLOR=\"green\"><b>ON</b></FONT></td></tr>";
else
  html += "<tr><td><b>Aspirazione Esterna</b></td><td><FONT COLOR=\"red\"><b>OFF</b></FONT></td></tr>";

if(rValvDeumidificatore->is_open)
  html += "<tr><td><b>Aspirazione Deumidificatore</b></td><td><FONT COLOR=\"green\"><b>ON</b></FONT></td></tr>";
else
  html += "<tr><td><b>Aspirazione Deumidificatore</b></td><td><FONT COLOR=\"red\"><b>OFF</b></FONT></td></tr>";

if(rDeumidificatore->is_on)
  html += "<tr><td><b>Deumidificatore</b></td><td><FONT COLOR=\"green\"><b>ON</b></FONT></td></tr>";
else
  html += "<tr><td><b>Deumidificatore</b></td><td><FONT COLOR=\"red\"><b>OFF</b></FONT></td></tr>";

if(rVentolaDeum->is_on)
  html += "<tr><td><b>Ventola Deumidificatore</b></td><td><FONT COLOR=\"green\"><b>ON</b></FONT></td></tr>";
else
  html += "<tr><td><b>Ventola Deumidificatore</b></td><td><FONT COLOR=\"red\"><b>OFF</b></FONT></td></tr>";

html += "</table></body></html>";

}
*/