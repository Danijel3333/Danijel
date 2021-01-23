#include<Wire.h> //fuer I2c
#include <ros.h> // rosserial/ros
#include <std_msgs/Int16.h> //Int message 16-bit

ros::NodeHandle  nh; // initialisierung von dem Node

std_msgs::Int16 myData1; //Finger messages
std_msgs::Int16 myData2;
std_msgs::Int16 myData3;
std_msgs::Int16 myData4;
std_msgs::Int16 myData5;

/*
std_msgs::Int16 gyrox; //gyro messages
std_msgs::Int16 gyroy;
std_msgs::Int16 gyroz;

std_msgs::Int16 accx; // acc messages
std_msgs::Int16 accy;
std_msgs::Int16 accz;

std_msgs::Int16 roll; //roll,pitch,yaw - messages
std_msgs::Int16 pitch;
std_msgs::Int16 yaw;
*/

// volatile byte g1,g2,g3,g4,g5,g6,a1,a2,a3,a4,a5,a6,r1,r2,p1,p2,j1,j2; // Variablen fuer bitShifting I2c
// volatile int gx,gy,gz,ax,ay,az,r,p,j;
volatile byte x1,x2,x3,x4,x5,x6,x7,x8,x9,x10;
volatile int y1,y2,y3,y4,y5;


ros::Publisher sendData1("sendData1", &myData1); // Publisher initialisierung
ros::Publisher sendData2("sendData2", &myData2);
ros::Publisher sendData3("sendData3", &myData3);
ros::Publisher sendData4("sendData4", &myData4);
ros::Publisher sendData5("sendData5", &myData5);
/*
ros::Publisher sendgyrox("sendgyrox", &gyrox);
ros::Publisher sendgyroy("sendgyroy", &gyroy);
ros::Publisher sendgyroz("sendgyroxz", &gyroz);

ros::Publisher sendaccx("sendaccx", &accx);
ros::Publisher sendaccy("sendaccy", &accy);
ros::Publisher sendaccz("sendaccz", &accz);

ros::Publisher sendroll("sendroll", &roll);
ros::Publisher sendpitch("sendpitch", &pitch);
ros::Publisher sendyaw("sendyaw", &yaw);
*/
void setup()
{
  Wire.begin(0x40); // I2c Adresse 0x40 Hex
  Serial.begin(9600);
  Wire.onReceive(receiveEvent); // Empfaenger Funktion fuer I2c mit receive Event
 
  nh.initNode();
  nh.advertise(sendData1);
  nh.advertise(sendData2);
  nh.advertise(sendData3);
  nh.advertise(sendData4);
  nh.advertise(sendData5);
/*
  nh.advertise(sendgyrox);
  nh.advertise(sendgyroy);
  nh.advertise(sendgyroz);

  nh.advertise(sendaccx);
  nh.advertise(sendaccy);
  nh.advertise(sendaccz);

  nh.advertise(sendroll);
  nh.advertise(sendpitch);
  nh.advertise(sendyaw);
*/

}

void loop()
{
  
  myData1.data = y1; // speichern von I2c Daten in messages 
  myData2.data = y2;
  myData3.data = y3;
  myData4.data = y4;
  myData5.data = y5;
/*
  gyrox.data = gx;
  gyroy.data = gy;
  gyroz.data = gz;

  accx.data = ax;
  accy.data = ay;
  accz.data = az;

  roll.data = r;
  pitch.data = p;
  yaw.data = j;
  */
  sendData1.publish( &myData1 ); //Messages publishen
  sendData2.publish( &myData2 );
  sendData3.publish( &myData3 );
  sendData4.publish( &myData4 );
  sendData5.publish( &myData5 );
 /*
  sendgyrox.publish( &gyrox );
  sendgyroy.publish( &gyroy);
  sendgyroz.publish( &gyroz );
  
  sendaccx.publish( &accx );
  sendaccy.publish( &accy );
  sendaccz.publish( &accz );
  
  sendroll.publish( &roll );
  sendpitch.publish( &pitch );
  sendyaw.publish( &yaw );
   */
   
  nh.spinOnce();
  delay(1000);
}

void receiveEvent(int howMany)
{
  //----------------------------------------------------------------------------------Finger
  x1 = Wire.read();  //x1 Hoeheres Byte
  x2 = Wire.read();  //x2 Niedrigeres Byte
  y1 = (int)x1 << 8 | (int)x2; //BitShifting und zusammenfuegen
  //------------------------
  x3 = Wire.read();  
  x4 = Wire.read();  
  y2 = (int)x3 << 8 | (int)x4;
  //------------------------
  x5 = Wire.read();  
  x6 = Wire.read();  
  y3 = (int)x5 << 8 | (int)x6;
  //---------------------------
  x7 = Wire.read(); 
  x8 = Wire.read();  
  y4 = (int)x7 << 8 | (int)x8;
  //---------------------------
  x9 = Wire.read();  
  x10 = Wire.read();  
  y5 = (int)x9 << 8 | (int)x10;
   /*
  //----------------------------------------------------------------------------------IMU
  g1 = Wire.read();  
  g2 = Wire.read();  
  gx = (int)g1 << 8 | (int)g2;
  //------------------------
  g3 = Wire.read();  
  g4 = Wire.read();  
  gy = (int)g3 << 8 | (int)g4;
  //------------------------
  g5 = Wire.read();  
  g6 = Wire.read();  
  gz = (int)g5 << 8 | (int)g6;
  //---------------------------------------------------------------------------Gyro
  
  a1 = Wire.read();  
  a2 = Wire.read();  
  ax = (int)a1 << 8 | (int)a2;
  //------------------------
  a3 = Wire.read();  
  a4 = Wire.read(); 
  ay = (int)a3 << 8 | (int)a4;
  //------------------------
  a5 = Wire.read();  
  a6 = Wire.read();
  az = (int)a5 << 8 | (int)a6;
  //---------------------------------------------------------------------------ACC
  
  r1 = Wire.read(); 
  r2 = Wire.read();  
  r = (int)r1 << 8 | (int)r2;
  //------------------------
  p1 = Wire.read();  
  p2 = Wire.read();  
  p = (int)p1 << 8 | (int)p2;
  //------------------------
  j1 = Wire.read();  
  j2 = Wire.read();  
  j = (int)j1 << 8 | (int)j2;
  //---------------------------------------------------------------------------Roll.pitch.yaw
   */
}
