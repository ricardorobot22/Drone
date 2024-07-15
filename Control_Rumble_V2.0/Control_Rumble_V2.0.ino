/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);  

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xD8, 0x13, 0x2A, 0x7D, 0xC3, 0xE4};
//0xC8, 0xF0, 0x9E, 0x79, 0x43, 0x7C

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float a;
  float b;
  float c;
  float d;
  float e;
} struct_message;

float datoa;
float datob;
float datoc;
float datod;
uint32_t tiempoAnterior ;


// Create a struct_message called myData
struct_message BMP180;
struct_message myData;

esp_now_peer_info_t peerInfo;

const int buttonPin = 18;
int lastbuttonstate = 0;
int currentbuttonstate =0;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&BMP180, incomingData, sizeof(BMP180));
 // Serial.print("Bytes received: ");
//  Serial.println(len);
 // Serial.println(String(myData.a)+" "+String(myData.b)+" "+ String(myData.c)+" "+String(myData.d));

}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  pinMode(buttonPin, INPUT);
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

   esp_now_register_recv_cb(OnDataRecv);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Inicializa pantalla, I2C con direccion 0x3C
  display.clearDisplay();       // Limpia pantalla OLED
  display.setTextColor(WHITE);  // Color de texto
  display.setRotation(0);  // Orientacion del texto 0, 1, 2 or 3
  display.setTextWrap(false);  
  display.dim(0);  //Brillo

  currentbuttonstate = digitalRead(buttonPin);
   myData.e = 0;
}
 
void loop() {

  lastbuttonstate =currentbuttonstate ;
  currentbuttonstate = digitalRead(buttonPin);
    if (currentbuttonstate == LOW && lastbuttonstate == HIGH) {
    // turn LED on
    if(myData.e==0){
      myData.e = 2;
      }
      else {
    
    myData.e = 0;
  }
    
  } 
 
  // Set values to send
  // datoa = analogRead(33);
  myData.a= map (analogRead(34),0,4096,2000,1000);
  
  datob = analogRead(33);
 // Serial.println(datob);
  if(datob < 2165 && datob > 1400){myData.b=1500;}
  if(datob <= 1400){myData.b = map (datob,1400,0,1499,1000);}
  if(datob >= 2165){myData.b = map (datob,2165,4095,1501,2000);}
  
 datoc = analogRead(35);
 //Serial.println(datoc);
 if(datoc < 2010 && datoc > 1300){myData.c=1500;}
 if(datoc <= 1300){myData.c = map (datoc,1300,0,1499,1000);}
 if(datoc >= 2010){myData.c = map (datoc,2010,4075,1501,2000);}

 datod = analogRead(32);
 //Serial.println(datod);
 if(datod < 2300 && datod > 1500){myData.d=1500;}
 if(datod <= 1500){myData.d = map (datod,1500,0,1501,2000);}
 if(datod >=2300){myData.d = map (datod,2300,4095,1499,1000);}
 
 
// myData.d = 0;
 // Serial.println(String(myData.a)+" "+String(myData.b)+" "+String(myData.c)+" "+String(myData.d)+" "+String(BMP180.a)+" "+String(myData.e));
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
   // Serial.println("Sent with success");
  }
  else {
   // Serial.println("Error sending the data");
  }
    while (micros() - tiempoAnterior < 2000);
  tiempoAnterior = micros();

   updateDisplay();
}

void updateDisplay(){
  // Display Readings on OLED Display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("INCOMING READINGS");
  display.setCursor(0, 17);
  display.print("T:");
  display.print( myData.a);
  display.setCursor(0, 27);
  display.print("R:");
  display.print(myData.b); 
  display.setCursor(0, 37);
  display.print("P:");
  display.print(myData.c);
  display.setCursor(0, 47);
  display.print("Y:");
  display.print(myData.d);
  display.setCursor(59, 17);
  display.print("Al:");
  display.print(BMP180.a);
  display.setCursor(59, 27);
  display.print("AH:");
  if( myData.e == 2){
  display.print("ON");
  }else{
  display.print("OF");
  }
  
 // display.setCursor(59, 27);
 // display.print("Pr:");
 // display.print(BMP180.b);
 // display.setCursor(59, 37);
 // display.print("Te:");
 // display.print(BMP180.c);
  display.display();
  

}
