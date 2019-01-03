#define W_RADIO
#ifdef W_RADIO
#define CLOUD_UPLOAD //tell gateway to store data on cloud
#include <SPI.h>
#include <MKRWAN.h>
#include <LoRa.h>
#define W_ACK
#define N_MAX_TRIES 5 //max number of retry before getting ACK

#define gw_addr   1 //gateway address on LoRa network (typically 1)
#define node_addr 7 //node address of LoRa node (>6)

LoRaModem modem;
#endif

#define W_TEMP
#ifdef W_TEMP
#include <OneWire.h>             // library for Onewire
#include <DallasTemperature.h>   //library for handling temperature sensors
#define ONE_WIRE_BUS 5

// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

#define N_SENSORS_MAX 4
#define N_MEASUREMENTS 1

// Addresses of DS18b20 sensors connected to the 1-wire line. 
// Each sensor has a unique 64bit identifier
uint8_t sensors_address[N_SENSORS_MAX][8];
uint8_t nSensors;
#endif

#define W_HUM
#ifdef W_HUM
#define N_SENSORS_HUM 3
#define N_MEASUREMENTS_HUM 1
uint8_t hum_sensors_input[N_SENSORS_HUM] = { A2, A3, A4 };
#endif

char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

//#define DEBUG
#include "ArduinoLowPower.h"
#ifndef DEBUG
#define SLEEP_TIME 10*60*1000 //sleep time in ms
#else
#define SLEEP_TIME 10*1000 //sleep time in ms
#endif

#define VBAT_MON

#define ICHARGE_MON
#define ICHARGE_PIN A1

#define SENSOR_POWER_SWITCH 4 //pin used to control the NPN transistor to turn power to the sensors

int counter = 0;

void setup() {

  pinMode(SENSOR_POWER_SWITCH,OUTPUT);
  digitalWrite(SENSOR_POWER_SWITCH,HIGH);
   
  pinMode(LED_BUILTIN, OUTPUT);
  for (byte i=0; i<20;++i)
  {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500); //delay to allow re-programming when sleeping
  digitalWrite(LED_BUILTIN, LOW);
  delay(500); //delay to allow re-programming when sleeping
  }

  Serial.begin(115200);
#ifdef DEBUG    
  while (!Serial);
#endif

  Serial.println("LoRa MKR Sensor");

#ifdef W_TEMP
  sensors.begin();
  findTempSensor();
#endif  

#ifdef W_HUM
   analogReadResolution(12);
//   analogReference(AR_INTERNAL1V65);
   for (unsigned int isens=0;isens<N_SENSORS_HUM;++isens)   
      pinMode(hum_sensors_input[isens], INPUT);
#endif

#ifdef VBAT_MON
   analogReadResolution(12);
 //  analogReference(AR_INTERNAL1V65);
   pinMode(ADC_BATTERY, INPUT);
#endif

#ifdef ICHARGE_MON
 pinMode(ICHARGE_PIN, INPUT);
#endif
 
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, dummy, CHANGE);
}

void loop() {
 digitalWrite(SENSOR_POWER_SWITCH,HIGH);
 delay(100);
#ifdef W_TEMP  
  char ltemp[N_SENSORS_MAX*10] = "";
  getTemp(ltemp); 
#endif
#ifdef W_HUM  
  char lhum[N_SENSORS_HUM*10] = "";
  getHumidity(lhum); 
#endif
#ifdef VBAT_MON
  char lbat[20] = "";
  getVbat(lbat);
#endif
#ifdef ICHARGE_MON
  char icharge[20] = "";
  getIcharge(icharge);
#endif
#ifdef W_RADIO
  setupRadio();
  char message[100]="";
#ifdef CLOUD_UPLOAD
  strcat(message,"\\!##"); //store the data to cloud
#endif
#ifdef W_TEMP  
  strcat(message,ltemp);
#endif
#ifdef W_HUM  
  strcat(message,lhum);
#endif   
#ifdef VBAT_MON
  strcat(message,lbat);
#endif  
#ifdef ICHARGE_MON
  strcat(message,icharge);
#endif  
  sendMessage(message);
  sleepRadio();
#endif  
  counter++;
  //delay(10000);
  digitalWrite(SENSOR_POWER_SWITCH,LOW);
  LowPower.sleep(SLEEP_TIME);
}

#ifdef W_RADIO
void sendMessage(char* data)
{
  Serial.println(data);
#ifdef W_ACK
  bool ack=false;
  byte ntries=N_MAX_TRIES;
  while (!ack && ntries>0)
  {
#endif 
  // send packet
  LoRa.beginPacket(0);
  //Header is 4 bytes: DST, TYPE, FROM, SEQ NUMBER
  LoRa.write(gw_addr); //dst
#ifdef W_ACK
  LoRa.write(0x18); //data type with ACK
#else  
  LoRa.write(0x10); //data type
#endif  
  LoRa.write(node_addr); //from
  LoRa.write(counter); //sequence number
  LoRa.write((uint8_t *) data,strlen(data));
  LoRa.endPacket();

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);                       
  digitalWrite(LED_BUILTIN, LOW);


#ifdef W_ACK 
  //now wait for ACK
  unsigned long starttime = millis();
  while ((millis() - starttime) < 3000)
    if (LoRa.parsePacket())
      break;
      
  int packetSize = LoRa.parsePacket();
  if (packetSize >=4) {
     // read packet header bytes:
    int recipient = LoRa.read();  
    byte type = LoRa.read();            // sender address    
    byte from = LoRa.read();            // sender address    
    byte msgCount = LoRa.read();
    
    String incoming = "";

    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    if (recipient == node_addr)
    {
      Serial.print("ACK: ");
      Serial.print(msgCount);
      Serial.print(" Message: ");
      Serial.print(incoming);
      Serial.println(" RSSI: " + String(LoRa.packetRssi()));
      ack=true;
      for (int iblink=0; iblink<2;++iblink)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);                       // wait for a second
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
      break;
    }
  }

  ntries--;
  Serial.println("No ACK. Try again");
  delay(random(200)); //random delay to avoid continous conflict
  }
  if (!ack)
    Serial.println("Is LoRa server running?");    
#endif    
}
#endif

#ifdef W_TEMP
void findTempSensor()
{
  // Check sensor connections and count devices
  nSensors=sensors.getDeviceCount();

#ifdef DEBUG
   Serial.print("Found ");
   Serial.print(nSensors);
   Serial.println(" sensors");
#endif

  if (nSensors > N_SENSORS_MAX)
  {
    Serial.println("Too many DS18B20 sensors");
    return;
  }
  
  for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
  {      
       if(!sensors.getAddress(&sensors_address[iSensor][0],iSensor))
       {
          //Serial.println("Cannot find a sensor");
          return;
       }
       sensors.setResolution((uint8_t*)&sensors_address[iSensor][0],12);

#ifdef DEBUG      
       Serial.print("DS18B20 sensor #");
       Serial.print(iSensor);
       Serial.print(" ");
       String address;
       for (int iWord=7;iWord>=0;--iWord) //read address from MSB to LSB (DS18B20 address ends with 28)
          address+=String(sensors_address[iSensor][iWord],HEX);
       address.toUpperCase(); //nicely put everything in upper case
       Serial.println(address);
#endif
  }

}
void getTemp(char* logtemp){

  float tMeas[nSensors];
  for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
     tMeas[iSensor]=0.; 

  
  for (uint8_t nMeas=0;nMeas< N_MEASUREMENTS ;++nMeas)
  { 
    sensors.requestTemperatures(); //start temp conversion for all devices at the same time. 12bit conversion is 750ms
//      delay(1000);
         
    for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
    {
      //start temp conversion. 12bit conversion is 750ms
//      while(!sensors.requestTemperaturesByAddress( (uint8_t*)&sensors_address[iSensor][0] ))       
//        delay(100);
      //read data from sensor and increase average; 
      tMeas[iSensor]+=sensors.getTempC( (uint8_t*)&sensors_address[iSensor][0] );     
    }
  }

  for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
  {
     tMeas[iSensor]=tMeas[iSensor]/(float)N_MEASUREMENTS; //average the measurements
/*     
     for (int iWord=7;iWord>=0;--iWord) //read address from MSB to LSB (DS18B20 address ends with 28)
     {
        String address=String(sensors_address[iSensor][iWord],HEX);
        address.toUpperCase(); //nicely put everything in upper case
        strLogline+=address;
     }
     strLogline+=";";
 */
     char sens[3];
     sprintf(sens,"%d",iSensor+1); 
     strcat(logtemp,"T");
     strcat(logtemp,sens);
     strcat(logtemp,"/");
     char temp[7];
     dtostrf(tMeas[iSensor], 5, 2, temp);
     strcat(logtemp,temp);
     strcat(logtemp,"/");
  }
#ifdef DEBUG
#ifndef W_RADIO
  Serial.println(logtemp);
#endif  
#endif
}
#endif

#ifdef VBAT_MON
void getVbat(char* logtemp)
{
   int adc=analogRead(ADC_BATTERY);
   float vbat=(adc/4095.0)*3.3*(1.0/(33.0/(68.0+33.0)));
   strcat(logtemp,"BT/");
   char vmon[5] = "";
   dtostrf(vbat, 3, 2, vmon);
   strcat(logtemp,vmon);
   strcat(logtemp,"/");
}
#endif

#ifdef ICHARGE_MON
void getIcharge(char* logtemp)
{
   int adc=analogRead(ICHARGE_PIN);
   float vbat=(adc/4095.0)*3.3;
   strcat(logtemp,"IC/");
   char vmon[5] = "";
   dtostrf(vbat, 3, 2, vmon);
   strcat(logtemp,vmon);
   strcat(logtemp,"/");
}
#endif

#ifdef W_HUM
void getHumidity(char* loghum)
{
  float hMeas[nSensors];
  for (uint8_t iSensor=0;iSensor<N_SENSORS_HUM;++iSensor)
     hMeas[iSensor]=0.; 

  for (uint8_t nMeas=0;nMeas< N_MEASUREMENTS_HUM ;++nMeas)
  { 
    for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
    {
      int adc=analogRead(hum_sensors_input[iSensor]);
      hMeas[iSensor]+=(adc/4095.0)*3.3; //maybe apply some humidity calibration?    
    }
  }
  
  for (uint8_t iSensor=0;iSensor<N_SENSORS_HUM;++iSensor)
  {
     hMeas[iSensor]=hMeas[iSensor]/(float)N_MEASUREMENTS_HUM; //average the measurements
     char sens[3];
     sprintf(sens,"%d",iSensor+1); 
     strcat(loghum,"H");
     strcat(loghum,sens);
     strcat(loghum,"/");
     char hum[7];
     dtostrf(hMeas[iSensor], 4, 3, hum);
     strcat(loghum,hum);
     strcat(loghum,"/");   
  }
}   
#endif

#ifdef W_RADIO
void setupRadio()
{
  modem.dumb(); //use the SX1276 directly bypassing STM32 AT modem
  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(LORA_IRQ_DUMB, 6, 1); // set CS, reset, IRQ pin
  LoRa.setFrequency(868E6);
  LoRa.setSPIFrequency(100000);

  //Mode 11 for PM LoRa Network
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(7);
  //LoRa.setSyncWord(0x45); //private syncword for PM network
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);

  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
#ifdef DEBUG  
  LoRa.dumpRegisters(Serial);
#endif 
}

void sleepRadio()
{
  //Set CS HIGH to avoid dumb mode drawing 8mA
  digitalWrite(LORA_IRQ_DUMB, HIGH);
  // Hardware reset
  digitalWrite(LORA_BOOT0, LOW);
  digitalWrite(LORA_RESET, HIGH);
  delay(200);
  digitalWrite(LORA_RESET, LOW);
  delay(200);
  digitalWrite(LORA_RESET, HIGH);
  delay(50);
  //Consumption in sleep mode ~1.1mA. From schematic it looks like this can be mostly due to TCXO still powered in sleep mode. 
  //To reduce further will need to provide VDD_TCXO from a controlled PIN (either from STM32 or SAMD21)
}

#endif
void dummy() {
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}
