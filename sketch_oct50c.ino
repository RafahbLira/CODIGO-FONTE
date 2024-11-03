//==============================================BIBLIOTECAS===================================================
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EmonLib.h>
#include <WiFi.h>
#include <IOXhop_FirebaseESP32.h>                           
#include <ArduinoJson.h>    
//============================================DEFINIÇÃO DE PINOS==============================================
#define PINO_DS18B20 4   // Pino do sensor DS18B20
#define PINO_KY037 34    // Pino do sensor KY-037
#define PINO_ZMPT101B 32 // Pino do sensor ZMPT101B 
#define PINO_SCT013 33   // Pino do SCT013 
//=================================================OUTROS=====================================================
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
OneWire oneWire(PINO_DS18B20);
DallasTemperature sensors(&oneWire);
EnergyMonitor SCT013;
//===========================================ESTRUTURA DE DADOS===============================================
struct SensorData {

  float magnitude;   // valor da vibração em magnitude ADXL345
  int niveldesom;    // Valor lido do KY-037
  float decibeis;    // Valor calculado de dB
  float tensaorms;   // Tensão RMS calculada
  double corrente;   // Corrente calculada
  float potencia;    // Potência calculada
  float temperatura; // Temperatura calculada

};

SensorData dados;

//===========================================TEMPORIZADOR E FATORES================================================
unsigned long leitura = 0; // Temporizador de leituras
const unsigned long intervalo_de_leitura = 500; // Intervalo

const float fatorConversaoSom = 90.0 / 4095.0; // Mapeia diretamente o valor lido para a faixa desejada
const float fatorTensao = (3.3 / 4095.0) * (5.0 / 3.3); // Pré-calcula fator de tensão
//====================================DEFINIÇÃO DE WI-FI E FIREBASE==========================================
// Credenciais Wi-Fi
const char* ssid = "brisa-2568252_EXT";
const char* password = "txanoqsb";
// Credenciais de acesso ao FireBase
#define FIREBASE_HOST "https://smartmunitor21633-default-rtdb.firebaseio.com/"    
#define FIREBASE_AUTH "USmVL9xZeLMqaG7p3YXgYflmTW4WnRQ44oZGUSql"  

//==============================CHAMADA DE FUNÇÕES DE CONFIGURAÇÃO E LEITURA==================================
void setup() {

    Serial.begin(115200);
    analogReadResolution(12); // Resolução 12 bits
    Wire.begin(21, 22); // inicialização I2C
    configuracaosensores(); // Inicializa os SENSORES

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);  // Inicialização de conexão
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Conectado ao Wi-Fi!");

    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}
void loop() {

    unsigned long inicio = millis();
    if(inicio - leitura >= intervalo_de_leitura){
      leitura = inicio;

    // Leitura dos sensores
    ler_sensor_ADX345();
    ler_sensor_DS18B20();  
    ler_sensor_KY037();
    ler_sensor_ZMPT101B();  
    ler_sensor_SCT013();

    //Impresão de dados
    print_dados();
    //Envio dos dados para o Firebase
    envio_dados_firebase();
  }
}
//=======================================CONFIGURAÇÃO DOS SENSORES===========================================
void configuracaosensores() {

    //ADX345
    accel.begin(); // Inicializa o sensor
    accel.setRange(ADXL345_RANGE_16_G);  // Define o range de ±16g

    //DS18B20
    sensors.begin(); // Inicializando o sensor 

    //KY037:
    pinMode(PINO_KY037, INPUT);  // Define o pino do sensor KY-037 como entrada

    //SCT013
    SCT013.current(PINO_SCT013, 6.0606);  
}

//========================================LEITURAS DOS SENSORES==============================================
void ler_sensor_ADX345() {

    sensors_event_t event;
    accel.getEvent(&event); // Captura os dados de aceleração
    // Calcula a magnitude da aceleração
    dados.magnitude = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
}

void ler_sensor_DS18B20() {
    sensors.requestTemperatures(); 
    dados.temperatura = sensors.getTempCByIndex(0);
}

void ler_sensor_KY037() {
    dados.niveldesom = analogRead(PINO_KY037); 
    dados.decibeis = (dados.niveldesom * fatorConversaoSom) + 30; // Mapeamento direto
}

void ler_sensor_ZMPT101B() {
    dados.tensaorms = analogRead(PINO_ZMPT101B) * fatorTensao; // Ajuste de tensão RMS
}

void ler_sensor_SCT013() {
    dados.corrente = SCT013.calcIrms(1480);  // Calcula o valor da Corrente (em mA)
    dados.potencia = dados.corrente * dados.tensaorms; // Calcula a Potência Instantânea (em W)
}

void print_dados() {
    Serial.print("-| Magnetude: ");
    Serial.println(dados.magnitude);
    Serial.print("-| Temperatura: ");
    Serial.println(dados.temperatura);
    Serial.print("-| Ruído: ");
    Serial.println(dados.decibeis);
    Serial.print("-| Tensão: ");
    Serial.println(dados.tensaorms);
    Serial.print("-| Corrente: ");
    Serial.println(dados.corrente);
    Serial.print("-| Potência: ");
    Serial.println(dados.potencia);
}

void envio_dados_firebase() {

    Firebase.setFloat("monitoramento-motor-eletrico/corrente", dados.corrente);
    Firebase.setFloat("monitoramento-motor-eletrico/potencia", dados.potencia);
    Firebase.setFloat("monitoramento-motor-eletrico/ruido", dados.decibeis);
    Firebase.setFloat("monitoramento-motor-eletrico/temperatura", dados.temperatura);
    Firebase.setFloat("monitoramento-motor-eletrico/tensao", dados.tensaorms);
    Firebase.setFloat("monitoramento-motor-eletrico/vibracao", dados.magnitude);
}