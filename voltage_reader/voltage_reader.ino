/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID           "TMPL6u8d_89Bw"
#define BLYNK_TEMPLATE_NAME         "Quickstart Device"
#define BLYNK_AUTH_TOKEN             //removed for security

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "CARDAK";
char pass[] = "OSMAN1450";

BlynkTimer timer;

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
}

// A counter variable to help generate smooth data
float counter = 0;
float pressure = 0;
// This function is called every second to send sensor data
void sendSensorData() {
  
  // --- SENSOR READING CODE IS REMOVED ---
  // int rawValue = analogRead(pressurePin);
  // float voltage = rawValue * (3.3 / 4095.0);
  // float pressure = (10.0 * voltage) - 2.0;

  // +++ DUMMY DATA GENERATION ADDED +++
  // We'll use a sine wave to create a smooth, oscillating value from 0 to 8.0
  // sin(counter) produces a value between -1 and 1.
  // (sin(counter) + 1) shifts this to be between 0 and 2.
  // Multiplying by 4 makes the range 0 to 8.
  float dummyPressure = (sin(counter) + 1) * 4;
  float dummyTemperature = (sin(counter) + 1) * 25;

  
  // Increment the counter for the next reading
  counter += 0.1;

  // Print the dummy value to the Serial Monitor for debugging
  Serial.print("Sending Dummy Pressure: ");
  Serial.print(dummyPressure);
  Serial.println(" Bar");


  // Send the dummy pressure value to Blynk on Virtual Pin V0
  Blynk.virtualWrite(V4, dummyPressure);
  Blynk.virtualWrite(V5, dummyTemperature);
}

const int pressurePin = 36;

void sendElData() {

// In your function that reads the sensor
int num_readings = 500;
long raw_sum = 0; // Use a long to prevent overflow

for (int i = 0; i < num_readings; i++) {
  raw_sum += analogRead(pressurePin);
  delay(1); // Small delay between readings
}

// Calculate the average raw value
int avgRawValue = raw_sum / num_readings;

// Now use the averaged value in your calculation
float voltage = avgRawValue * (3.3 / 4095.0);

Serial.print("Stable Voltage: ");
Serial.println(voltage);


if (voltage <= 0.39){
  pressure = 0;
}
else{
  pressure = 10 * voltage - 3.7;
}

Serial.print("Pressure: ");
Serial.println(pressure);


  // 4. Send the calculated pressure to Blynk on Virtual Pin V0
  // Blynk.virtualWrite(V6, voltage);
  
  // You would read your temperature sensor here and send it to V1
  // float temperature = readTemperature();
  // Blynk.virtualWrite(V1, temperature);
}



void setup()
{
  // Debug console
  Serial.begin(115200);

  // Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  // You can also specify server:
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  // timer.setInterval(1000L, myTimerEvent);
  // timer.setInterval(1000L, sendSensorData);
  timer.setInterval(2000L, sendElData);
}



void loop()
{
  // Blynk.run();
  timer.run();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
}

