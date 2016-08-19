#include "DHT.h"
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include "SensorData.h"

#define NO_ALARM_ID 0
#define TEMP_ALARM_ID 1
#define HUM_ALARM_ID 2
#define GAS_ALARM_ID 3

#define TEMP_H_BOUND 35  
#define TEMP_L_BOUND 16
#define HUM_H_BOUND 80
#define HUM_L_BOUND 20
#define GAS_H_BOUND 5

#define DHTPIN 2
#define DHTTYPE DHT11

#define debug true

char * const NET_CONF[] PROGMEM = { 
         "192.168.1.10", //IP ADDRESS
         "192.168.1.1",  // GATEWAY
         "255.255.255.0", // NETMASK 
         "XXXXXXXXXXX", // things speak API KEY
         "SSID", //SSID
         "password" // PWD
         }; 

long lastConnectionRetry = 0;
int CONN_RETRY_DELAY = 1000;  // 1 minute

long lastSensorCheck = 0;
long SENSOR_CHECK_RATE = 300000; // 5 minutes

long lastConnectionCheck = 0;
long CONN_CHECK_RATE = 300000; // 5 minutes

long lastSensorDataUpdate = 0;
long SENSOR_DATA_UPDATE = 1200000; // 20 minutes

byte OUT_CONN_ID = 0;

float R0 = 0; //Gas sensor in clean air resistance

boolean connected = false;

SoftwareSerial swSerial(10, 11); // RX, TX

DHT dht(DHTPIN, DHTTYPE);

void setup() {
 
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // set data rate and timeout for the SoftwareSerial port
  swSerial.begin(9600);
  swSerial.setTimeout(2000);

  pinMode(13, OUTPUT);

  calibrateGasSensor();

  initWifiModule();
}

void loop() {

  while (swSerial.available()){
    
    if(swSerial.find("+IPD,")){
          
      delay(500); //wait for the buffer to fill-up
      
      int connectionId = swSerial.read()-48;
        
      swSerial.find("pin=");
      int pinNumber = swSerial.read()-48; 
        
      int secondNumber = swSerial.read()-48;
      if(secondNumber>=0 && secondNumber<=9){
        pinNumber*=10;
        pinNumber +=secondNumber;
      }
     
      digitalWrite(pinNumber, !digitalRead(pinNumber));

      String response;
      response = "Pin ";
      response += pinNumber;
      response += " is ";
      response += digitalRead(pinNumber) == HIGH ? "ON" : "OFF";

      sendHTTPResponse(response, connectionId);
     
      // make close command
      String closeCommand = "AT+CIPCLOSE="; 
      closeCommand+=connectionId; 

      sendESPCommand(closeCommand,1000); 
    }
  }
  
  long currentMillis = millis();
  
  if (!connected && (currentMillis - lastConnectionRetry >= CONN_RETRY_DELAY)) {
    lastConnectionRetry = currentMillis;
    joinNetwork();
  }

  if(connected && (currentMillis - lastSensorCheck >= SENSOR_CHECK_RATE)){
    lastSensorCheck = currentMillis;
    if(checkSensors()){
      updateChannels();
    }
  }

  if(connected && (currentMillis - lastSensorDataUpdate >= SENSOR_DATA_UPDATE)){
    lastSensorDataUpdate = currentMillis;
    updateChannels();
  }

  if(currentMillis - lastConnectionCheck >= CONN_CHECK_RATE){
    lastConnectionCheck = currentMillis;
    checkConnectionStatus();
  }

}

void initWifiModule() {

  digitalWrite(13, LOW);

  //send reset
  sendESPCommand("AT+RST",3000);

  //turn off autoconn
  sendESPCommand("AT+CWAUTOCONN=0",1000); // 0 off, 1 on

  //disable DHCP
  sendESPCommand("AT+CWDHCP_DEF=1,0",1000); // 1,0 disabled - 1,1 enabled (need reboot)

  //set net config
  String cmd = "AT+CIPSTA=\"";

  cmd += String((char *) pgm_read_word (&NET_CONF [0])) + "\",\"";
  cmd += String((char *) pgm_read_word (&NET_CONF [1])) + "\",\"";
  cmd += String((char *) pgm_read_word (&NET_CONF [2])) + "\"";
  
  sendESPCommand(cmd,1000);
 
  //set mode
  sendESPCommand("AT+CWMODE=1",1000);//1 CLI, 2 HOST, 3 both

  sendESPCommand("AT+CIPMUX=1",1000);//0 Single conn, 1 Multiple conn (4 max) 

  sendESPCommand("AT+CIPSERVER=1",1000);//Start webserver on port 333
}

void scanNetwork() {
  sendESPCommand("AT+CWLAP",3000);
}

void joinNetwork() {

  String cmd = "AT+CWJAP=\"";
  cmd += String((char *) pgm_read_word (&NET_CONF [4]));
  cmd += "\",\"";
  cmd += String((char *) pgm_read_word (&NET_CONF [5]));
  cmd += "\"";
 
  sendESPCommand(cmd,0);

  if (!swSerial.find("OK")) {
    connected = false;
    return;
  }

  sendESPCommand("AT+CIFSR",1000);
  connected = true;
  digitalWrite(13, HIGH);
}

void checkConnectionStatus() {

  if (!connected) {
    return;
  }

  sendESPCommand("AT+CIPSTATUS",0);
  
  if (swSerial.find("STATUS:")) {
    
    delay(500);//waiting for serial to fill-up
    byte connStatus = swSerial.read() - 48;

    switch (connStatus) {
      case 2:
        Serial.println("Got ip");
        break;
      case 3:
        Serial.println("Connected");
        break;
      case 4:
        Serial.println("Disconnected"); //bugged
        //resetModule();
        break;
      case 5:
        Serial.println("Connection fail");
        resetModule();
        break;
      default:
        Serial.println("Unknown");
    }
  }
}

void resetModule() {
  sendESPCommand("AT+CWQAP",0);
  connected = false;
  initWifiModule();
}

void clearSerialBuffer() {
  
  while (swSerial.available()) {
    swSerial.read();
  }
}

void updateChannels() {

  if (!connected) {
    return;
  }

  // TCP connection
  String cmd = "AT+CIPSTART=";
  cmd += OUT_CONN_ID;
  cmd += ",";
  cmd += "\"TCP\",\"";
  cmd += "184.106.153.149";
  cmd += "\",80";

  sendESPCommand(cmd,0);

  if (swSerial.find("ERROR")) {
    resetModule();
    return;
  }

  SensorData *data = getSensorsData(); 
  String query =  "field1=" + String(data->temperature,2) + 
                  "&field2=" + String(data->humidity,2) + 
                  "&field3=" + String(data->ch4Concentration) + 
                  "&field4=" + String(data->smokeConcentration) +
                  "&field5=" + String(data->coConcentration) +
                  "&field6=" + String(data->alarmFlag);    
  
  sendGetRequest(query);
  free(data);
  
  //close TCP connection
  sendESPCommand("AT+CIPCLOSE=" + OUT_CONN_ID,1000);

  OUT_CONN_ID = OUT_CONN_ID > 3 ? 0 : OUT_CONN_ID +1;
}

void sendGetRequest(String query) {

  // prepare GET string
  String request = "GET /update?api_key=";
  request += String((char *) pgm_read_word (&NET_CONF [3]));
  request += "&";
  request += query;
  request += "\n";

  sendCIPData(request,OUT_CONN_ID);
}

void sendHTTPResponse(String content, int connectionId){
     
  String httpResponse;
  httpResponse = "HTTP/1.1 200 OK\nContent-Type: text/html; charset=UTF-8\n"; 
  httpResponse += "Content-Length: ";
  httpResponse += content.length();
  httpResponse += "\n";
  httpResponse +="Connection: close\n\n";
  httpResponse += content;
  
  sendCIPData(httpResponse,connectionId);
}

void sendESPCommand(String command,int timeout){

  clearSerialBuffer();
   
  swSerial.println(command); 
  long int time = millis();
 
  if(debug){
    while(time + timeout > millis()){
     while(swSerial.available()){
      Serial.write(swSerial.read());    
     }  
    }
  }else{
    delay(timeout);
  } 
}

void sendCIPData(String data,int connectionId){

  String cmd = "AT+CIPSEND=";
  cmd += connectionId;
  cmd += ",";
  cmd += data.length();
  
  swSerial.println(cmd);

  if (swSerial.find(">")) {
    swSerial.println(data);
  }
}

void calibrateGasSensor(){

  byte RL_VALUE = 5; //5Kohm
  float count = 50;
  float sensorValue = 0;

  for(int i = 0 ; i < count ; i++){
    sensorValue += analogRead(A0);
    delay(500);
  }

  sensorValue = sensorValue/count;
  float val = (((float)RL_VALUE*(1023-sensorValue)/sensorValue));
  R0 = val/9.8;
}

int getGasConcentration(int gasAdcValue,double x, double y, double slope){
  
  byte RL_VALUE = 5;//5Kohm
  float val = (((float)RL_VALUE*(1023-gasAdcValue)/gasAdcValue));
  float ratio = val/R0;

  return (pow(10,(((log(ratio)-y)/slope) + x)));
}

boolean checkSensors(){
  
  SensorData *data = getSensorsData();
  int alarmFlag = data->alarmFlag;
  free(data);
  
  return alarmFlag > NO_ALARM_ID;
}

SensorData *getSensorsData(){

  SensorData *data = (SensorData*)malloc(sizeof(struct SensorData));
  
  data->humidity = dht.readHumidity();
  data->temperature = dht.readTemperature(); 
  
  data->ch4Concentration = getGasConcentration(analogRead(A0),2.3,0.47,-0.44);
  data->smokeConcentration = getGasConcentration(analogRead(A0),2.3,0.53,-0.44);
  data->coConcentration = getGasConcentration(analogRead(A0),2.3,0.72,-0.34);

  setAlarmFlag(data);
    
  return data;
}

void setAlarmFlag(SensorData *data){

  byte value = NO_ALARM_ID;

  if(data->temperature >= TEMP_H_BOUND || data->temperature <= TEMP_L_BOUND){
    value = TEMP_ALARM_ID;
  }

  if(data->humidity >= HUM_H_BOUND || data-> humidity <= HUM_L_BOUND ){
    value += HUM_ALARM_ID;
  }

  if(data->ch4Concentration >= GAS_H_BOUND || data->smokeConcentration >= GAS_H_BOUND || data->coConcentration >= GAS_H_BOUND){
    value += GAS_ALARM_ID;
  }

  data->alarmFlag = value;
}




