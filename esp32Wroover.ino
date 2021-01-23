#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250_asukiaaa mySensor;

// MAC Adresse des Empfängers
uint8_t broadcastAddress[] = {0x24,0x6F,0x28,0xB4,0x93,0xC0};

// Messwerte
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

// Incoming Werte
int haptic1;
int haptic2;
int haptic3;
int haptic4;
int haptic5;


// Wenn senden erfolgreich war
String success;

//Struktur der Daten
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

//Struktur für die Messungen die gesendet werden initialisierung
struct_message Sensoren;

// Stuktur für incoming werte initialisierung
struct_message incomingReadings;

// Callback DATAsent , wird überprüft ob die Daten richtig gesendet wurden
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback DATArecieve, hier werden die incoming daten aus der Struktu in die Variablen gespeichert
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  haptic1 = incomingReadings.data1;
  haptic2 = incomingReadings.data2;
  haptic3 = incomingReadings.data3;
  haptic4 = incomingReadings.data4;
  haptic5 = incomingReadings.data5;
}
 
void setup() {
 
  Serial.begin(115200);

#ifdef _ESP32_HAL_I2C_H_
// for esp32
Wire.begin(SDA_PIN, SCL_PIN); //sda, scl
#else
Wire.begin();
#endif
 
mySensor.setWire(&Wire);
mySensor.beginGyro();
mySensor.beginAccel();

 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialisirung ESP-NOW
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
  Sensoren.data1 = finger1;
  Sensoren.data2 = finger2;
  Sensoren.data3 = finger3;
  Sensoren.data4 = finger4;
  Sensoren.data5 = finger5;

   Sensoren.datagx = gyrox;
   Sensoren.datagy = gyroy;
   Sensoren.datagz = gyroz;

   Sensoren.dataax  = accx;
   Sensoren.dataay  = accy;
   Sensoren.dataaz  = accz;

   Sensoren.dataroll  = roll;
   Sensoren.datapitch  = pitch;
   Sensoren.datayaw = yaw; 

  // Mit esp-now werden die Daten aus der Struktur an die Mac adresse gesendet
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Sensoren, sizeof(Sensoren));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  updateData();
  delay(10000);
}
// Einlesen von Messwerten
void getMeasurement(){
 
  finger1 =  analogRead(34);
  finger2 =  analogRead(35);
  finger3 =  analogRead(36);
  finger4 =  analogRead(37);
  finger5 =  analogRead(38);

  mySensor.accelUpdate();

accx=mySensor.accelX();;
accy=mySensor.accelY();
accz=mySensor.accelZ();

mySensor.gyroUpdate();

gyrox= mySensor.gyroX();
gyroy= mySensor.gyroY();
gyroz= mySensor.gyroZ();

 pitch = atan(accx / sqrt((accy * accy) + (accz * accz)));
 roll = atan(accy / sqrt((accx * accx) + (accz * accz)));
 yaw = atan(accz / sqrt((accx*accx) + (accz*accz)));
  //Radiant in Grad
 pitch = pitch * (180.0 / 3.14);
 roll = roll * (180.0 / 3.14);
 yaw = yaw * (180.0 /3.14);
  
delay(500);
}

void updateData(){
 
  //Ausgabe in Serial Monitor
  //---------------------------- Haptic sensoren
  Serial.println("INCOMING READINGS");
  Serial.print("Haptic1: ");
  Serial.print(incomingReadings.data1);
 
  Serial.print("Haptic2: ");
  Serial.print(incomingReadings.data2);

  Serial.print("Haptic3: ");
  Serial.print(incomingReadings.data3);

  Serial.print("Haptic4: ");
  Serial.print(incomingReadings.data4);
  
  Serial.print("Haptic5: ");
  Serial.print(incomingReadings.data5);
  
  Serial.println();
}
