#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// MAC Adresse des Empfängers
uint8_t broadcastAddress[] = {0x4C,0x11,0xAE,0xDF,0x9D,0x38};

// Messwerte
int haptic1;
int haptic2;
int haptic3;
int haptic4;
int haptic5;

// Incoming Werte
int finger1;
int finger2;
int finger3;
int finger4;
int finger5;

int gyrox;
int gyroy;
int gyroz;

int accx;
int accy;
int accz;

int pitch;
int roll;
int yaw;

// Wenn senden erfolgreich war
String success;

//Strutktur der Daten
typedef struct struct_message {
   int data1;
   int data2;
   int data3;
   int data4;
   int data5;
    
   int datagx;
   int datagy;
   int datagz;
    
   int dataax;
   int dataay;
   int dataaz;

   int dataroll;
   int datapitch;
   int datayaw;
} struct_message;

//Struktur für die Messungen die gesendet werden
struct_message Haptic;

// Stuktur für incoming werte initialisirung
struct_message incomingReadings;

// Callback DATAsent, wird überprüft ob die Daten richtig gesendet wurden
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success";
  }
  else{
    success = "Delivery Fail";
  }
}


// Callback DATArecieve, hier werden die incoming daten aus der Struktur in die Variablen gespeichert
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  finger1 = incomingReadings.data1;
  finger2 = incomingReadings.data2;
  finger3 = incomingReadings.data3;
  finger4 = incomingReadings.data3;
  finger5 = incomingReadings.data3;

  gyrox = incomingReadings.datagx;
  gyroy = incomingReadings.datagy;
  gyroz = incomingReadings.datagz;

  accx = incomingReadings.dataax;
  accy = incomingReadings.dataay;
  accz = incomingReadings.dataaz;

  roll = incomingReadings.dataroll;
  pitch = incomingReadings.datapitch;
  yaw = incomingReadings.datayaw;
}
 
void setup() {
  // Für I2c
  Wire.begin();
  Serial.begin(115200);

 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialisierung ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  //Funtkion fürs einlesen der Werte wird aufgerufen
  getMeasurement();
 
  // Werte die gesendet werden
  Haptic.data1 =  haptic1;
  Haptic.data2 =  haptic2;
  Haptic.data3 =  haptic3;
  Haptic.data4 =  haptic4;
  Haptic.data5 =  haptic5;

  
  // Mit esp-now werden die Daten aus der Struktur an die Mac adresse gesendet
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Haptic, sizeof(Haptic));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  //------------------------------------------------I2C-Funktion
  I2c();
  //--------------------------------------------- SerialMonitor-Funktion
  updateData();

  
  delay(1000);
}
// Einlesen von Messwerten
void getMeasurement(){
  //Hier sollten die Haptic sensoren eingelesen werden
  haptic1 = 0;
  haptic2 = 1;
  haptic3 = 2;
  haptic4 = 3;
  haptic5 = 4;
}
void I2c(){
   Wire.beginTransmission(0x40);  //slaveAddress
  // ------------------- Finger
  // Int hat 16-bit (2Byte), es muss ein byte nach dem anderen gesendet werden
  Wire.write(highByte(finger1)); //Höheres Byte wird gesendet
  Wire.write(lowByte(finger1));  // Niedrigeres Byte wird gesendet
  
  Wire.write(highByte(finger2)); 
  Wire.write(lowByte(finger2));
  
  Wire.write(highByte(finger3)); 
  Wire.write(lowByte(finger3));
  
  Wire.write(highByte(finger4)); 
  Wire.write(lowByte(finger4));
  
   Wire.write(highByte(finger5));
  Wire.write(lowByte(finger5));
  
 // --------------------------------------------- Gyro
/* Wire.write(highByte(gyrox)); 
 Wire.write(lowByte(gyrox));

 Wire.write(highByte(gyroy)); 
 Wire.write(lowByte(gyroy));

 Wire.write(highByte(gyroz)); 
 Wire.write(lowByte(gyroz));
*/
 //--------------------------------------------- ACC
/* Wire.write(highByte(accx)); 
 Wire.write(lowByte(accx));
  
 Wire.write(highByte(accy)); 
 Wire.write(lowByte(accy));
  
 Wire.write(highByte(accz)); 
 Wire.write(lowByte(accz));
*/
 //--------------------------------------------Pitch,Yaw,Roll
 /*Wire.write(highByte(roll)); 
 Wire.write(lowByte(roll));
 
 Wire.write(highByte(pitch)); 
 Wire.write(lowByte(pitch));
  
 Wire.write(highByte(yaw)); 
 Wire.write(lowByte(yaw));
 */
  Wire.endTransmission();
}
void updateData(){
 
  // Ausgabe in Serial Monitor
  // ------------------------------------------ Finger
  Serial.println("INCOMING READINGS");
  Serial.print("finger1: ");
  Serial.print(incomingReadings.data1);
 
  Serial.print("finger2: ");
  Serial.print(incomingReadings.data2);

  Serial.print("finger3: ");
  Serial.print(incomingReadings.data3);
  
  Serial.print("finger4: ");
  Serial.print(incomingReadings.data4);
  
  Serial.print("finger5: ");
  Serial.print(incomingReadings.data5);
  Serial.println();
// -------------------------------------------- Gyro
  Serial.print("GyroX: ");
  Serial.print(incomingReadings.datagx);

  Serial.print("GyroY: ");
  Serial.print(incomingReadings.datagy);
  
  Serial.print("GyroZ: ");
  Serial.print(incomingReadings.datagz);
  Serial.println();
  // ----------------------------------------- ACC
  Serial.print("ACCx: ");
  Serial.print(incomingReadings.dataax);

  Serial.print("ACCy: ");
  Serial.print(incomingReadings.dataay);
  
  Serial.print("ACCz: ");
  Serial.print(incomingReadings.dataaz);

  //----------------------------- yaw,roll,pitch
  Serial.print("Roll: ");
  Serial.print(incomingReadings.dataroll);

  Serial.print("Pitch: ");
  Serial.print(incomingReadings.datapitch);
  
  Serial.print("Yaw: ");
  Serial.print(incomingReadings.datayaw);
  
  
  Serial.println();
}
