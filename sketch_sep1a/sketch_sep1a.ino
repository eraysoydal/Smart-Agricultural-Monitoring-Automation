#define BLYNK_TEMPLATE_ID           "TMPL6u8d_89Bw"
#define BLYNK_TEMPLATE_NAME         "Quickstart Device"
#define BLYNK_AUTH_TOKEN             //removed for security

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

char ssid[] = "CARDAK";
char pass[] = "OSMAN1450";

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
BlynkTimer timer;


// PINS
int pressurePin = 36;
int electricityPin = 39;
int alarmPin = 34;
int SondajPin = 35;

void sendSondajData(){
  bool sondaj = true;
  
  int rawData = analogRead(SondajPin);
  float sondajVoltage = rawData * (3.3 / 4095.0);

  if (sondajVoltage == 0){
    sondaj = false;
  }

  Serial.print("SONDAJ: ");
  Serial.println(sondaj);
}


void sendAlarmData(){
  bool alarm = false;
  
  int rawData = analogRead(alarmPin);
  float alarmVoltage = rawData * (3.3 / 4095.0);

  if (alarmVoltage == 0){
    alarm = true;
  }

  Serial.print("ALARM: ");
  Serial.println(alarm);
}

void sendElectrictyData(){
  bool electricity = true;
  
  int rawData = analogRead(electricityPin);
  float electricityVoltage = rawData * (3.3 / 4095.0);

  if (electricityVoltage == 0){
    electricity = false;
  }

  Serial.print("ELECTRICITY: ");
  Serial.println(electricity);

  Blynk.virtualWrite(V6, electricityVoltage);
}

// void sendPressureData(){
//   int num_readings = 100;
//   long raw_sum = 0;
//   float pressure = 0;


//   for (int i = 0; i < num_readings; i++){
//     raw_sum += analogRead(pressurePin);
//     delay(1);
//   }

//   int avgRawValue = raw_sum / num_readings;

//   float pressureVoltage = avgRawValue * (3.3 / 4095.0);

//   if (pressureVoltage <= 0.39){
//     pressure = 0;
//   }
//   else{
//     pressure = 10 * pressureVoltage - 3.7;
//   }

//   Serial.print("Pressure: ");
//   Serial.println(pressure);

// }




void setup() {
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);


  timer.setInterval(1000L, sendSondajData);
  timer.setInterval(1000L, sendElectrictyData);
  timer.setInterval(1000L, sendAlarmData);
}

void loop() {
  Blynk.run();
  timer.run();
}
