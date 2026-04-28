#include <Arduino.h>
#include <Relay.h>
#include <MySensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <ESPmDNS.h>
#include <LittleFS.h> 
#include <time.h>     
#include <math.h>     
#include "secrets.h"

// --- Definizioni PIN e Tempi (Invariati) ---
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

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      
const int   daylightOffset_sec = 3600; 

WebServer server(80);

// Oggetti Globali
RelayMotore* rValvDeumidificatore = NULL;
RelayMotore* rValvEsterno = NULL;
SingleRelay* rVentolaDeum = NULL;
SingleRelay* rDeumidificatore = NULL;
Sensor* sensoreEsterno = NULL;
Sensor* sensoreInterno = NULL;

typedef struct ValoriSensori {
    float tempInterno;
    float tempEsterno;
    float umidInterno;
    float umidEsterno;
    int lastReadInterno;
    int lastReadEsterno;
} ValoriSensori;

ValoriSensori* vs = NULL;

// --- Prototipi ---
void handleRoot();
void handleData(); 
void handleStorico(); // Nuova rotta
void handleDownload(); // Per servire i file CSV
void TaskSensoriInterno(void* param);
void TaskSensoriEsterno(void* param);
void serverTask(void* parameter);
void ElegantOTATask(void* parameter);
void LoggingTask(void* parameter); 

TaskHandle_t task_server_handle;

// --- Utility Ora ---
String getTimeStamp() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)) return "Errore_Ora";
    char buffer[50];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buffer);
}

void setup(){
    Serial.begin(112500);

    // 1. HARDWARE INIT (Priorità per evitare crash)
    sensoreEsterno = new Sensor(SENSORE_ESTERNO_PIN);
    sensoreInterno = new Sensor(SENSORE_INTERNO_PIN);
    rVentolaDeum = new SingleRelay(FAN_PIN, 0);
    rDeumidificatore = new SingleRelay(DEHUMIDIFIER_PIN, 0);
    rValvDeumidificatore = new RelayMotore(DEUM_VALVE_OPEN_PIN,DEUM_VALVE_CLOSE_PIN,MOTOR_TIME,true);
    rValvEsterno = new RelayMotore(EXTERN_VALVE_OPEN_PIN,EXTERN_VALVE_CLOSE_PIN,MOTOR_TIME);

    vs = (ValoriSensori*) calloc(1,sizeof(ValoriSensori));
    vs->tempEsterno = CANC_NUM; vs->umidEsterno = CANC_NUM;
    vs->tempInterno = CANC_NUM; vs->umidInterno = CANC_NUM;

    // 2. CONNETTIVITÀ
    WiFi.setHostname("vmc-coto"); 
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(1000); Serial.print("."); }
    
    if (MDNS.begin("vmc-coto")) { MDNS.addService("http", "tcp", 80); }

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    LittleFS.begin(true);

    // 3. ROTTE SERVER
    server.on("/", handleRoot);
    server.on("/data", handleData); 
    server.on("/storico", handleStorico);
    server.on("/download", handleDownload); // Es: /download?file=sensore_esterno.csv
    
    ElegantOTA.begin(&server);
    server.begin();

    // 4. AVVIO TASK
    xTaskCreatePinnedToCore(TaskSensoriEsterno, "SensExt", 4096, vs, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskSensoriInterno, "SensInt", 4096, vs, 1, NULL, 0);
    xTaskCreatePinnedToCore(serverTask, "WebTask", 16384, NULL, 1, &task_server_handle, 0);
    xTaskCreatePinnedToCore(LoggingTask, "LogTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(ElegantOTATask, "OTA", 4096, NULL, 3, NULL, 0);
}

void loop(){ 
    if(isnan(vs->tempEsterno) || vs->tempEsterno == CANC_NUM) { delay(2000); return; }

    // Logica VMC (Semplificata per brevità, tieni la tua originale)
    if((vs->umidEsterno <= SOGLIA_UMIDITA) && (vs->tempEsterno <= TEMP_MAX_ESTERNO) && (vs->tempEsterno >= TEMP_MIN_ESTERNO)){
        rValvEsterno->openValve();
        if(vs->umidInterno >= SOGLIA_UMIDITA - 10) { rValvDeumidificatore->openValve(); rVentolaDeum->turnOn(); rDeumidificatore->turnOn(); }
        else { rValvDeumidificatore->closeValve(); }
    } else {
        rValvEsterno->closeValve();
        if(vs->umidInterno >= SOGLIA_UMIDITA - 10) { rVentolaDeum->turnOn(); rDeumidificatore->turnOn(); }
        else { rDeumidificatore->turnOff(); }
    }
    delay(SAMPLE_DELAY);
}

// --- GESTIONE DOWNLOAD FILE ---
void handleDownload() {
    if (!server.hasArg("file")) { server.send(400, "text/plain", "Manca parametro file"); return; }
    String path = "/" + server.arg("file");
    if (LittleFS.exists(path)) {
        File f = LittleFS.open(path, "r");
        server.streamFile(f, "text/csv");
        f.close();
    } else {
        server.send(404, "text/plain", "File non trovato");
    }
}

// --- PAGINA STORICO CON GRAFICO ---
void handleStorico() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>Storico VMC</title>";
    html += "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>"; // Libreria Grafici
    html += "<style>body{font-family:sans-serif; background:#f4f7f6; padding:20px; text-align:center;} .box{background:white; max-width:900px; margin:auto; padding:20px; border-radius:10px; box-shadow:0 2px 10px rgba(0,0,0,0.1);}</style>";
    html += "</head><body>";
    html += "<div class='box'><h1>Andamento Sensori (Ultime 24h)</h1>";
    html += "<canvas id='myChart'></canvas>";
    html += "<br><a href='/'>Torna alla Dashboard</a> | <a href='/download?file=sensore_esterno.csv'>Scarica CSV Esterno</a></div>";
    
    html += "<script>";
    html += "async function drawChart() {";
    html += "  const res = await fetch('/download?file=sensore_esterno.csv');";
    html += "  const data = await res.text();";
    html += "  const rows = data.trim().split('\\n').slice(-100);"; // Prendiamo le ultime 100 letture
    html += "  const labels = [], temps = [], hums = [];";
    html += "  rows.forEach(row => {";
    html += "    const cols = row.split(',');";
    html += "    labels.push(cols[0].split(' ')[1]);"; // Solo l'ora del timestamp
    html += "    temps.push(parseFloat(cols[1]));";
    html += "    hums.push(parseFloat(cols[2]));";
    html += "  });";
    html += "  new Chart(document.getElementById('myChart'), {";
    html += "    type: 'line',";
    html += "    data: { labels: labels, datasets: [";
    html += "      { label: 'Temp Esterna (°C)', data: temps, borderColor: 'red', fill: false },";
    html += "      { label: 'Umidità Esterna (%)', data: hums, borderColor: 'blue', fill: false }";
    html += "    ] }";
    html += "  });";
    html += "}";
    html += "window.onload = drawChart;";
    html += "</script></body></html>";
    
    server.send(200, "text/html", html);
}

// --- DASHBOARD (Aggiornata con Link) ---
void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1' charset='utf-8'>";
    html += "<title>VMC Dashboard</title>";
    html += "<style>";
    html += "body { font-family: 'Segoe UI', sans-serif; background: #f0f2f5; color: #333; text-align: center; padding: 20px; }";
    html += ".container { max-width: 800px; margin: auto; }";
    html += ".card { background: white; padding: 20px; margin: 10px; border-radius: 15px; box-shadow: 0 4px 12px rgba(0,0,0,0.08); display: inline-block; width: 45%; min-width: 280px; vertical-align: top; }";
    html += ".full-card { width: 95%; display: block; margin: 10px auto; }";
    html += "h1 { color: #1a73e8; margin-bottom: 25px; }";
    html += "h3 { border-bottom: 2px solid #f0f2f5; padding-bottom: 10px; margin-top: 0; color: #5f6368; }";
    html += ".val { font-size: 2.5em; font-weight: bold; color: #1a73e8; margin: 10px 0; }";
    html += ".sub-val { font-size: 1.2em; color: #70757a; }";
    html += ".status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; text-align: left; padding: 10px; }";
    html += ".status-item { padding: 12px; border-radius: 10px; background: #f8f9fa; display: flex; justify-content: space-between; align-items: center; font-weight: 500; }";
    html += ".on { color: #2ecc71; font-weight: bold; } .off { color: #e74c3c; font-weight: bold; }";
    html += ".btn { display: inline-block; padding: 15px 30px; background: #1a73e8; color: white; text-decoration: none; border-radius: 10px; margin-top: 20px; font-weight: bold; transition: 0.3s; }";
    html += ".btn:hover { background: #1557b0; transform: translateY(-2px); }";
    html += "</style></head><body>";
    
    html += "<div class='container'>";
    html += "<h1>Controllo VMC IoT</h1>";
    
    // Card Sensore Esterno
    html += "<div class='card'><h3>Aria Esterna</h3>";
    html += "<div class='val'><span id='te'>--</span>°C</div>";
    html += "<div class='sub-val'>Umidità: <span id='he'>--</span>%</div></div>";
    
    // Card Sensore Interno
    html += "<div class='card'><h3>Aria Interna</h3>";
    html += "<div class='val'><span id='ti'>--</span>°C</div>";
    html += "<div class='sub-val'>Umidità: <span id='hi'>--</span>%</div></div>";
    
    // Card Stato Attuatori (Relè e Valvole)
    html += "<div class='card full-card'><h3>Stato Attuatori</h3>";
    html += "<div class='status-grid'>";
    html += "  <div class='status-item'><span>Valvola Esterna</span> <span id='vExt'>--</span></div>";
    html += "  <div class='status-item'><span>Valvola Deum.</span> <span id='vDeum'>--</span></div>";
    html += "  <div class='status-item'><span>Deumidificatore</span> <span id='deum'>--</span></div>";
    html += "  <div class='status-item'><span>Ventola Ricircolo</span> <span id='vent'>--</span></div>";
    html += "</div></div>";

    html += "<a href='/storico' class='btn'>Visualizza Grafici Storici</a>";
    html += "</div>";

    // Script di aggiornamento AJAX
    html += "<script>";
    html += "function update() {";
    html += "  fetch('/data').then(r => r.json()).then(d => {";
    // Aggiornamento Numeri
    html += "    document.getElementById('te').innerText = d.tempExt.toFixed(1);";
    html += "    document.getElementById('he').innerText = d.humExt.toFixed(1);";
    html += "    document.getElementById('ti').innerText = d.tempInt.toFixed(1);";
    html += "    document.getElementById('hi').innerText = d.humInt.toFixed(1);";
    
    // Funzione helper per cambiare colore al testo degli stati
    html += "    const setStatus = (id, val) => {";
    html += "      const el = document.getElementById(id); if(!el) return;";
    html += "      el.innerText = val;";
    html += "      el.className = (val === 'ON' || val === 'APERTA') ? 'on' : 'off';";
    html += "    };";
    
    // Aggiornamento Stati Attuatori
    html += "    setStatus('vExt', d.vExt);";
    html += "    setStatus('vDeum', d.vDeum);";
    html += "    setStatus('deum', d.deum);";
    html += "    if(d.vent) setStatus('vent', d.vent);"; // Se presente nel JSON
    
    html += "  }).catch(e => console.error('Errore aggiornamento:', e));";
    html += "}";
    html += "setInterval(update, 3000); update();";
    html += "</script></body></html>";

    server.send(200, "text/html", html);
}

// --- ALTRE FUNZIONI (Identiche alle precedenti) ---
// --- ALTRE FUNZIONI E TASK COMPLETAMENTE RIPRISTINATI ---

void handleData() {
    String json = "{";
    // Dati Sensori (Numerici)
    json += "\"tempExt\":" + String(vs->tempEsterno) + ",";
    json += "\"humExt\":" + String(vs->umidEsterno) + ",";
    json += "\"tempInt\":" + String(vs->tempInterno) + ",";
    json += "\"humInt\":" + String(vs->umidInterno) + ",";
    
    // Stati Attuatori (Stringhe per il frontend)
    json += "\"vExt\":\""   + String(rValvEsterno->is_open ? "APERTA" : "CHIUSA") + "\",";
    json += "\"vDeum\":\""  + String(rValvDeumidificatore->is_open ? "APERTA" : "CHIUSA") + "\",";
    json += "\"deum\":\""   + String(rDeumidificatore->is_on ? "ON" : "OFF") + "\",";
    json += "\"vent\":\""   + String(rVentolaDeum->is_on ? "ON" : "OFF") + "\"";
    
    json += "}";
    
    server.send(200, "application/json", json);
}

void serverTask(void* parameter) {
    while (true) {
        if(!WiFi.isConnected()){ while(WiFi.reconnect()); }
        server.handleClient();
        delay(10);
    }
}

void ElegantOTATask(void* parameter) {
    while (true) { 
        ElegantOTA.loop(); 
        delay(100); 
    }
}

void LoggingTask(void* parameter) {
    // Aspettiamo 10 secondi all'avvio prima di fare il primo salvataggio
    vTaskDelay(10000 / portTICK_PERIOD_MS); 

    while (true) {
        if(WiFi.isConnected()) {
            String timestamp = getTimeStamp();
            
            if (timestamp != "Errore_Ora") {
                Serial.println("Salvataggio Dati CSV in corso: " + timestamp);
                
                // 1. Sensore Esterno
                File f = LittleFS.open("/sensore_esterno.csv", FILE_APPEND);
                if(f) { f.printf("%s,%.2f,%.2f\n", timestamp.c_str(), vs->tempEsterno, vs->umidEsterno); f.close(); }

                // 2. Sensore Interno
                f = LittleFS.open("/sensore_interno.csv", FILE_APPEND);
                if(f) { f.printf("%s,%.2f,%.2f\n", timestamp.c_str(), vs->tempInterno, vs->umidInterno); f.close(); }

                // 3. Valvola Esterna
                f = LittleFS.open("/valvola_esterna.csv", FILE_APPEND);
                if(f) { f.printf("%s,%s\n", timestamp.c_str(), rValvEsterno->is_open ? "APERTA" : "CHIUSA"); f.close(); }

                // 4. Valvola Deumidificatore
                f = LittleFS.open("/valvola_deum.csv", FILE_APPEND);
                if(f) { f.printf("%s,%s\n", timestamp.c_str(), rValvDeumidificatore->is_open ? "APERTA" : "CHIUSA"); f.close(); }

                // 5. Deumidificatore (Motore)
                f = LittleFS.open("/deumidificatore.csv", FILE_APPEND);
                if(f) { f.printf("%s,%s\n", timestamp.c_str(), rDeumidificatore->is_on ? "ON" : "OFF"); f.close(); }

                // 6. Ventola Deumidificatore
                f = LittleFS.open("/ventola_deum.csv", FILE_APPEND);
                if(f) { f.printf("%s,%s\n", timestamp.c_str(), rVentolaDeum->is_on ? "ON" : "OFF"); f.close(); }

                Serial.println("Salvataggio completato!");
            }
        }
        // Aspetta 15 minuti (900.000 millisecondi)
        vTaskDelay(900000 / portTICK_PERIOD_MS);
    }
}

void TaskSensoriEsterno(void* param){
    ValoriSensori* valori = (ValoriSensori*) param;
    while(true){
        float h = sensoreEsterno->getHumidity();
        // Controlliamo che il valore sia valido prima di aggiornare i dati
        if(!isnan(h) && h >= 0 && h <= 100) {
            valori->tempEsterno = sensoreEsterno->getTemperature();
            valori->umidEsterno = h;
            valori->lastReadEsterno = 0;
        } else { 
            valori->lastReadEsterno++; 
        }
        delay(sensoreEsterno->delayus/1000);
    }
}

void TaskSensoriInterno(void* param){
    ValoriSensori* valori = (ValoriSensori*) param;
    while(true){
        float h = sensoreInterno->getHumidity();
        // Controlliamo che il valore sia valido prima di aggiornare i dati
        if(!isnan(h) && h >= 0 && h <= 100){
            valori->tempInterno = sensoreInterno->getTemperature();
            valori->umidInterno = h;
            valori->lastReadInterno = 0;
        } else { 
            valori->lastReadInterno++; 
        }
        delay(sensoreInterno->delayus/1000);
    }
}