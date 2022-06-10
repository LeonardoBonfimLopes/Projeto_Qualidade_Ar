//Bibliotecas a serem incluidas
#include <Wire.h> //comunicação I2C
#include <Adafruit_ADS1X15.h> //Conversor ADS1115
#include <WiFi.h> //Biblioteca para utilizar o Wi-Fi
#include <PubSubClient.h> //Biblioteca do MQTT
#include <DHT.h> // Bibliotecaq do sensor DHT22
#include <stdlib.h>

Adafruit_ADS1115 ads; //Definição do ADS a ser utilizado

// Definicoes do sensor DHT22
#define DHTPIN 4    //GPIO que está ligado o pino de dados do sensor
#define DHTTYPE DHT22   //sensor em utilização: DHT22
DHT dht(DHTPIN, DHTTYPE);

//Definição dos sensores
#define   ADC_16BIT_MAX   65536
float ads_InputRange = 6.144f;
const float VOLT_STEP = (ads_InputRange * 2) / (ADC_16BIT_MAX - 1);
#define RL_MQ8 10000 // Valor da resistência de carga para o sensor MQ8
#define R_STANDARD_MQ8 17900 // Valor de R0 para o sensor MQ8 padrão
#define RL_CO 560000 // Valor da resistência de carga para o sensor CO
#define R_STANDARD_CO 540000 // Valor de R0 para o sensor CO padrão
#define RL_NO2 10000 // Valor da resistência de carga para o sensor NO2
#define R_STANDARD_NO2 6500 // Valor de R0 para o sensor NO2 padrão
#define RL_NH3 470000 // Valor da resistência de carga para o sensor NH3
#define R_STANDARD_NH3 50000 // Valor de R0 para o sensor NH3 padrão

//Dados para conectar no Wi-Fi
const char* ssid = "Rafael";  //Nome do Wi-Fi
const char* password = "rafa2409"; //Senha do Wi-Fi

//Dados do MQTT Broker
const char* mqtt_server = "projetoleolopesifsp.duckdns.org"; //Localização do MQTT Broker
const char* mqtt_username = "admin"; //Usuario do Broker
const char* mqtt_password = "bababoe141516"; //Usuario do Broker
const int mqtt_port = 1883; //Porta de acesso do Broker

WiFiClient espClient;
PubSubClient client(espClient);

void init_wifi() { //Função para inicializar o Wi-Fi
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  reconnect_wifi();
}

void init_mqtt(){ //Função para inicializar o MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void reconnect_wifi(){ //Função para conectar o Wi-Fi
  if (WiFi.status() == WL_CONNECTED)
        return;
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {//Loop que mostra a tentativa de se conectar no Wi-Fi
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
}

void reconnect_mqtt() { //Função para conectar o mqtt
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(1000);
    }
  }
}

void verificaConexao(){
   reconnect_wifi();
   if (!client.connected()) {
    reconnect_mqtt();
  }
}

float faz_leitura_temperatura(){
    float t = dht.readTemperature();
    float result;
     
    if (! (isnan(t)) )
        result = t;
    else
        result = 0;

    Serial.print(result);
    Serial.println(" °C");
    return result;
}

float faz_leitura_umidade(){
    float h = dht.readHumidity();    
    float result;
     
    if (! (isnan(h)) ) //Caso o valor não é um numero colocará uma valor diferente no resultado
        result = h;
    else
        result = 0;

    Serial.print(result);
    Serial.println("%");
    return result;
}

int leitura_mq8(){
  float adc3 = ads.readADC_SingleEnded(3);
  float RS = RL_MQ8 * ((5 / (VOLT_STEP * adc3)) - 1);
  float ratio = Ajust_ratio_mq8((RS/R_STANDARD_MQ8));
  float PPM = pow(10, ((-log10(ratio) + 4.8965) / 1.59295));
  int result;

  if (! (isnan(PPM)) ) //Caso o valor não é um numero colocará uma valor diferente no resultado
        result = (int)PPM;
    else
        result = 0;

  Serial.print(result);
  Serial.println(" ppm H2");
  return result;
}

int leitura_CO(){
  float adc0 = ads.readADC_SingleEnded(0);
  float RS = RL_CO / ((5 / (VOLT_STEP * adc0)) - 1);
  float ratio = RS / R_STANDARD_CO;
  float PPM = pow(ratio, -1.177) * 4.4638;
  int result;

  if (! (isnan(PPM)) ) //Caso o valor não é um numero colocará uma valor diferente no resultado
        result = (int)PPM;
    else
        result = 0;

  Serial.print(result);
  Serial.println(" ppm CO");
  return result;
}

int leitura_NO2(){
  float adc1 = ads.readADC_SingleEnded(1);
  float RS = RL_NO2 / ((5 / (VOLT_STEP * adc1)) - 1);
  float ratio = RS / R_STANDARD_NO2;
  float PPM = pow(ratio, 0.9979) * 0.1516;
  int result;

  if (! (isnan(PPM)) ) //Caso o valor não é um numero colocará uma valor diferente no resultado
        result = (int)PPM;
    else
        result = 0;

  Serial.print(result);
  Serial.println(" ppm NO2");
  return result;
}

int leitura_NH3(){
  float adc2 = ads.readADC_SingleEnded(2);
  float RS = RL_NH3 / ((5 / (VOLT_STEP * adc2)) - 1);
  float ratio = RS / R_STANDARD_NH3;
  float PPM = pow(ratio, -1.903) * 0.6151;
  int result;

  if (! (isnan(PPM)) ) //Caso o valor não é um numero colocará uma valor diferente no resultado
        result = (int)PPM;
    else
        result = 0;

  Serial.print(result);
  Serial.println(" ppm NH3");
  return result;
}

float Ajust_ratio_mq8(float r0_rs_ratio){
  float ajusted_ratio = 0; //Valor da razao ajustada pela temperatura e humidade
  float max_value_ratio = 0; //Valor de correcao da razao a 33% de umidade
  float min_value_ratio = 0; //Valor de correcao da razao a 85% de umidade
  int humidity_ar = dht.readHumidity(); //Leitura da umidade pelo sensor
  int temperature_ar = dht.readTemperature(); //Leitura da temperatura pelo sensor
  if(humidity_ar>85 || humidity_ar<33 || temperature_ar<0 ||temperature_ar>50)
  {
  return NAN;
  }
  delay(100);

  max_value_ratio = 0.00012 * pow(temperature_ar, 2) - 0.00656 * temperature_ar + 0.98828; //Curva para 33% de umidade
  min_value_ratio = 0.00014 * pow(temperature_ar, 2) - 0.00753 * temperature_ar + 1.04125; //Curva para 85% de umidade
  ajusted_ratio = r0_rs_ratio * (max_value_ratio + (min_value_ratio - max_value_ratio) *(humidity_ar - 33) / 52); //Interpolacao entre os valores maximos e minimos e a umidade
  return ajusted_ratio;
}

void setup() {
  Serial.begin(9600); //Inicializa o serial
  ads.begin(); //Inicializa o ADS
  dht.begin(); //Inicializa o DHT
  init_wifi(); //Inicializa o Wi_Fi
  init_mqtt(); //Inicializa o MQTT
}

void loop() {
  //Variaveis locais
  char temperatura_str[10] = {0};
  char umidade_str[10]     = {0};
  char h2_str[10]          = {0};
  char co_str[10]          = {0};
  char no2_str[10]         = {0};
  char nh3_str[10]         = {0};
  double temp = faz_leitura_temperatura();
  double umi = faz_leitura_umidade();
  double h2 = leitura_mq8();
  double co = leitura_CO();
  double no2 = leitura_NO2();
  double nh3 = leitura_NH3();
  
  verificaConexao();
  client.loop();

  //Preenchimento da String
  //sprintf(temperatura_str,"%.2f", faz_leitura_temperatura());
  //sprintf(umidade_str,"%.2f", faz_leitura_umidade());
  //sprintf(h2_str,"%d", leitura_mq8());
  //sprintf(co_str,"%d", leitura_CO());
  //sprintf(no2_str,"%d", leitura_NO2());
  //sprintf(nh3_str,"%d", leitura_NH3());
  dtostrf(temp, 5, 2, temperatura_str);
  dtostrf(umi, 5, 2, umidade_str);
  dtostrf(h2, 5, 2, h2_str);
  dtostrf(co, 5, 2, co_str);
  dtostrf(no2, 5, 2, no2_str);
  dtostrf(nh3, 5, 2, nh3_str);

  //Envia os dados via MQTT
  client.publish("topico_sensor_temperatura", temperatura_str);
  client.publish("topico_sensor_umidade", umidade_str);
  client.publish("topico_sensor_h2", h2_str);
  client.publish("topico_sensor_co", co_str);
  client.publish("topico_sensor_nh3", no2_str);
  client.publish("topico_sensor_no2", nh3_str);

  delay(2000);
  } 
