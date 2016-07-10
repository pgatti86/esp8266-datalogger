#include "DHT.h"
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include "SensorData.h"

#define DHTPIN 2
#define DHTTYPE DHT11

#define debug true

char * const NET_CONF[] PROGMEM = { 
         "192.168.1.3", //IP ADDRESS
         "192.168.1.1",  // GATEWAY
         "255.255.255.0", // NETMASK 
         "XXXXXXXXXXXXXXXX", // things speak API KEY
         "your_network_SSID", //SSID
         "your_password", // PWD
         }; 

long lastConnectionRetry = 0;
int CONN_RETRY_DELAY = 1000; // 1 minute

long lastSensorCheck = 0;
long SENSOR_CHECK_RATE = 300000; // 5 minutes

long lastConnectionCheck = 0;
long CONN_CHECK_RATE = 300000; // 5 minutes

long lastSensorDataUpdate = 0;
long SENSOR_DATA_UPDATE = 30000; //30 seconds

byte OUT_CONN_ID = 0;  //OUTPUT CONNECTION ID

float R0 = 0; //Gas sensor in clean air resistance (mq2 sensor)

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

      sendHTTPResponse(connectionId,response);
     
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

  //set multiple connection
  sendESPCommand("AT+CIPMUX=1",1000);//0 Single conn, 1 Multiple conn (4 max) 

  //start web server (toggle onboard led)
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
        Serial.println("Disconnected"); //bugged, reset not needed (module still connected)
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
  cmd += "184.106.153.149"; //thingspeak address
  cmd += "\",80";           //port

  sendESPCommand(cmd,0);

  if (swSerial.find("ERROR")) {
    resetModule();
    return;
  }

  SensorData *data = getSensorsData(); 
  String query =  "field1=" + String(data->temperature) + 
                  "&field2=" + String(data->humidity) + 
                  "&field3=" + String(data->rawGasValue) + 
                  "&field4=" + String(data->gasConcentration);     
  
  sendGetRequest(query);
  free(data);
  
  //close TCP connection
  sendESPCommand("AT+CIPCLOSE=" + OUT_CONN_ID,1000);
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

void sendHTTPResponse(int connectionId, String content){
     
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

  OUT_CONN_ID = OUT_CONN_ID > 3 ? 0 : OUT_CONN_ID +1;
 
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

int getGasConcentration(int gasAdcValue){
  
  byte RL_VALUE = 5;//5Kohm
  float val = (((float)RL_VALUE*(1023-gasAdcValue)/gasAdcValue));
  float ratio = val/R0;

  return (pow(10,(((log(ratio)-0.53)/-0.44) + 2.3)));
}

boolean checkSensors(){

  SensorData *data = getSensorsData();
  boolean needUpdate = false;

  if(data->temperature > 50)
    needUpdate = true;

  if(data->humidity > 80)
    needUpdate = true;

  if(data->rawGasValue > 500)  
    needUpdate = true;

  if(data->gasConcentration > 100)
    needUpdate = true; 

  free(data);
  return needUpdate;
}

SensorData *getSensorsData(){

  SensorData *data = (SensorData*)malloc(sizeof(struct SensorData));
  data->humidity = dht.readHumidity();
  data->temperature = dht.readTemperature(); 
  data->rawGasValue = analogRead(A0);
  data->gasConcentration = getGasConcentration(data->rawGasValue);
  return data;
}




