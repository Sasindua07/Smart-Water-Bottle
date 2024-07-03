
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>

// Set these to run example.
#define FIREBASE_HOST "hydr8-e6e22-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "6hc7aeHMQoTgXYFsXfdDVCRUxEgFa3MKmzLTFO6F"
#define WIFI_SSID "Dialog 4G 280"
#define WIFI_PASSWORD "9EF87fbD"

String Data,values;
void setup() {
  Serial.begin(9600);
  delay(1000);

  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}


void loop() 
{
  bool Sr=false;
 
    while(Serial.available())
    {
      //get sensor data from serial put in sensor_data
      Data=Serial.readString(); 
      Sr=true;    
    }
  
  delay(1000);

    if(Sr==true)
    {  

      values=Data;

      int fristCommaIndex = values.indexOf(',');
      int secondCommaIndex = values.indexOf(',', fristCommaIndex+1);

      String horizontal = values.substring(0, fristCommaIndex);
      String batteryPercentage = values.substring(fristCommaIndex+1,secondCommaIndex);
      String voterVolume = values.substring(secondCommaIndex+1);



      Firebase.setString("horizontal",horizontal);
      delay(10);
      Firebase.setString("batteryPercentage",batteryPercentage);
      delay(10);
      Firebase.setString("waterVolume",batteryPercentage);
      
    
      delay(1000);

      if (Firebase.failed()) 
      {  
        return;
      }
}
}