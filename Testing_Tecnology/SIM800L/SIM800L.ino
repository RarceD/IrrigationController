#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SPIFlash.h>
#include <SPI.h>
#include "LowPower.h"
/******************************************************************* debug ********************************************************************************************/
#define RF_RST    27 
#define INT_RF    2
#define INT_RTC   14
#define CS_M      22       
#define CS_RF     23 
#define VREF_IN   24
#define VREF_EXT  29
#define PG_RXD    16 
#define PG_TXD    17 
#define SIM_PWR   26
#define SIM_AWK   21
#define LED_SETUP 3 
#define PCINT_PIN 18 
#define PCMSK *digitalPinToPCMSK(PCINT_PIN)
#define PCINT digitalPinToPCMSKbit(PCINT_PIN)
#define PCPIN *portInputRegister(digitalPinToPort(PCINT_PIN))
/******************************************************************* declarations  ************************************************************************************/ 
const char apn[]  = "gprs-service.com";
const char user[] = "";                 
const char pass[] = "";
SPIFlash flash(CS_M);
TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);

/******************************************************************* setup section ************************************************************************************/
void setup(){
 
  Serial.begin(115200);                                                                
  delay(250);
  pinMode(CS_RF, OUTPUT);  
  digitalWrite(CS_RF, HIGH);  
  pinMode(RF_RST, OUTPUT);
  digitalWrite(RF_RST, HIGH); 
  pinMode(SIM_PWR,OUTPUT);                                                           
  pinMode(SIM_AWK,OUTPUT);                                                               
  pinMode(PCINT_PIN,INPUT);                                                               
  pinMode(LED_SETUP,OUTPUT);                                                             
  digitalWrite(SIM_PWR,LOW);                                                              
  digitalWrite(SIM_AWK,HIGH);                                                             
  connectSIM();
  connectMqtt();
  delay(5);
}


/******************************************************************* main program  ************************************************************************************/
void loop(){
  
  LowPower.powerDown(SLEEP_500MS,ADC_OFF,BOD_OFF);
  mqttClient.loop();
  if(!mqttClient.connected()){
    Serial.println("Mqtt connection fail");
    connectMqtt();
    delay(5);
  }
}

/*******************************************************************   functions     ************************************************************************************/

/*
this function connects to internet through sim card
*/
void connectSIM(){

  delay(500);
  Serial.println("Oasis-Com starts");
  pinMode(SIM_PWR, OUTPUT);
  digitalWrite(SIM_PWR,LOW);
  delay(1200);
  Serial.println("Initializing modem..."); 
  digitalWrite(SIM_PWR,HIGH);
  delay(1200);
  Serial1.begin(57600);
  modem.restart();

  String modemInfo = modem.getModemInfo();    
  if(modemInfo.indexOf("SIM800") == -1) {
    Serial1.end();
    digitalWrite(SIM_PWR,LOW);
    delay(1200);
    digitalWrite(SIM_PWR,HIGH);
    delay(1200);
    Serial1.begin(57600);
    modem.restart();   
    modemInfo = modem.getModemInfo();    
  }

  Serial.print("Modem: ");
  Serial.println(modemInfo);
  Serial1.print("AT+IPR=115200");
  modem.init();
  Serial1.begin(115200);
  delay(2000);
  Serial1.print("AT+IPR=115200");
  modem.simUnlock("8724"); 
  //Serial.print("AT+CSCLK=2");

  
  Serial.print("Waiting for network...");                                                             
  if(!modem.waitForNetwork()) {
    Serial.println(" fail");
    while(true);
  }
  Serial.println(" succeed");
  
  Serial.print("Connecting to ");
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(" fail");
    while (true);
  }
  Serial.println(" succeed");
  int signalq = modem.getSignalQuality();
  Serial.println("Signal quality: " +  String(signalq));
}

/*
this function connects to jam-idea mqtt broker
*/
void connectMqtt(){

  String clientId = "Jam-Idea" + modem.getIMEI();
  mqttClient.setServer("mqtt.pre.hydro-plus.es", 1883);                                           
  mqttClient.setCallback(mqttCallback);                                                   
  while(!mqttClient.connected()){                                                         
    if(mqttClient.connect(clientId.c_str(),"hydroplus","vyrsa"))                               
      mqttClient.subscribe("oasis-gprs");                                            
    else{                                                                                  
      Serial.println("Trying to connect to MQTT...");
      delay(1000);
    }                                                                                                                              
  }
  Serial.println("Successfully connected to MQTT");  
}

/*
this callback function is called everytime a subscription topic message is received
*/
void mqttCallback(char* topic, byte* payload, unsigned int length){
  
}
