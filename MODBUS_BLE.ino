#include <SimpleModbusMaster.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include "BLEDevice.h"
#include <std_msgs/String.h>
#include <ArduinoJson.h>

unsigned int Speed_FL_to_send_1 = 0;
unsigned int Speed_FL_to_send_2 = 0; 
unsigned int Speed_FR_to_send_1 = 0;
unsigned int Speed_FR_to_send_2 = 0;
unsigned int Speed_RL_to_send_1 = 0;
unsigned int Speed_RL_to_send_2 = 0;
unsigned int Speed_RR_to_send_1 = 0;
unsigned int Speed_RR_to_send_2 = 0;

//////////////////// Parametry komunikacji ///////////////////
#define baud 115200
#define timeout 4
#define polling 2 // the scan rate
#define retry_count 1000

#define RXD1 3
#define TXD1 1

//Wyjście logiczne odpowiedzialne za kierunek przepływu danych
#define TxEnablePin 5 

//Ilość wszystkich rejestrów na których będą wykonywane operacje
#define TOTAL_NO_OF_REGISTERS 24

#define LED_BUILTIN 2

/* Konfiguracja BLE */

#define BLE_SERVER_NAME "Xsens Dot"
static BLEAddress *pServerAddress;
static BLEUUID    serviceUUID("15172000-4947-11e9-8646-d663bd873d93");          //serwis, numer 0x2000
static BLEUUID    charMeasurementUUID("15172002-4947-11e9-8646-d663bd873d93");  // charakterystyka do subskrypcji, numer 0x2003
static BLEUUID    charStartUUID("15172001-4947-11e9-8646-d663bd873d93");        // charakterystyka do uruchomienia ciaglego przesyłu danych, numer 0x2001

const uint8_t notifyON[]={0x01, 0x01, 0x1A};                                    //0x010117 - kodu uruchomienia subskrypcji w Movella XSens. CUSTOM DATA -> TO BE CHANGED. 1A custom 5 - quaternion / acc / ang. vel
static boolean doConnect = false;
static boolean connectedBLE = false;
static boolean doScan = false;
static BLERemoteCharacteristic* charMeasurement;
static BLERemoteCharacteristic* charStart;
static BLEAdvertisedDevice* myDevice;

#define STACK_SIZE 10000
TaskHandle_t taskBleHandle        = NULL;
TaskHandle_t taskRosSerialHandle  = NULL;
void taskBLE(void *pvParameters);

int motorsON = 0;

enum
{
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  PACKET5,
  PACKET6,
  PACKET7,
  PACKET8,
  PACKET9,
  PACKET10,
  PACKET11,
  PACKET12,
  PACKET13,
  PACKET14,
  PACKET15,
  PACKET16,
  PACKET17,
  PACKET18,
  PACKET19,
  PACKET20,
  TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Masters register array
unsigned int regs[TOTAL_NO_OF_REGISTERS];

void connectionSpeed() {
    packets[0].connection = 0; packets[1].connection = 0; packets[2].connection = 0; packets[3].connection = 0;
    packets[4].connection = 0; packets[5].connection = 0; packets[6].connection = 0; packets[7].connection = 0;
    packets[8].connection = 0; packets[9].connection = 0;
    packets[10].connection = 0; packets[11].connection = 0; packets[12].connection = 1; packets[13].connection = 1; 
    packets[14].connection = 1; packets[15].connection = 1; packets[16].connection = 0; packets[17].connection = 0; 
    packets[18].connection = 0; packets[19].connection = 0; 
}




//BluetoothSerial SerialBT;


// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

double lastCmdVelReceived = 0;
void onTwist(const geometry_msgs::Twist &cmd_vel);
void handleMovement();
void setupRos();

// ROSserial - subskrybuj cmd_vel
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);
std_msgs::String str_msg;
ros::Publisher pub("/IMU_data",&str_msg);


bool connected = false;
bool movement = false;
bool updating = false;
float linearVelocityX= 0.0, linearVelocityY= 0.0, angularVelocityZ = 0.0;

/* DATA HANDLER FROM SUBSCRIPTION -> CHARACTERISTIC RECIVE MEASUREMENT. <-*/

// data parsed from ble to ros_serial
float quaternionW, quaternionX, quaternionY, quaternionZ;
float acceleratioX, acceleratioY, acceleratioZ;
float angularX, angularY, angularZ, angularXrad, angularYrad, angularZrad;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    
    memcpy(&quaternionW, pData +4, sizeof(float));
    memcpy(&quaternionX, pData +4 + sizeof(float), sizeof(float));
    memcpy(&quaternionY, pData +4 + sizeof(float) * 2, sizeof(float));
    memcpy(&quaternionZ, pData +4 + sizeof(float) * 3, sizeof(float));

    memcpy(&acceleratioX, pData +4 + sizeof(float) * 4, sizeof(float));
    memcpy(&acceleratioY, pData +4 + sizeof(float) * 5, sizeof(float));
    memcpy(&acceleratioZ, pData +4 + sizeof(float) * 6, sizeof(float));

    memcpy(&angularX, pData +4 + sizeof(float) * 7, sizeof(float));
    memcpy(&angularY, pData +4 + sizeof(float) * 8, sizeof(float));
    memcpy(&angularZ, pData +4 + sizeof(float) * 9, sizeof(float));

    angularXrad=angularX*0.0174533;
    angularYrad=angularY*0.0174533;
    angularZrad=angularZ*0.0174533;

    /*SHOULD BE COMMENTED!!! - dubugging*/

    // printf("Quaternion values:\n");
    // printf("X: %f\n", quaternionX);
    // printf("Y: %f\n", quaternionY);
    // printf("Z: %f\n", quaternionZ);
    // printf("W: %f\n", quaternionW);
    // Serial.println();
    // printf("Accelerations (LOCAL):\n");
    // printf("X: %f\n", acceleratioX);
    // printf("Y: %f\n", acceleratioY);
    // printf("Z: %f\n", acceleratioZ);
    // Serial.println();
    // printf("Angular velocities:\n");
    // printf("X: %f\n", angularXrad);
    // printf("Y: %f\n", angularYrad);
    // printf("Z: %f\n", angularZrad);
    // Serial.println();
    
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("onConnect");
  }

  void onDisconnect(BLEClient* pclient) {
    connectedBLE = false;
    Serial.println("onDisconnect");
  }
};

/*    SERVER CONNECTION   */
bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");
    pClient->setClientCallbacks(new MyClientCallback());
    // Connect to the remove BLE Server.
    pClient->connect(myDevice); 
    Serial.println(" - Connected to server");
    pClient->setMTU(517); 
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");
    charMeasurement = pRemoteService->getCharacteristic(charMeasurementUUID);
    charStart = pRemoteService->getCharacteristic(charStartUUID);
    if (charMeasurement == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charMeasurementUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if(charMeasurement->canNotify())
    {
      charMeasurement->registerForNotify(notifyCallback);
    }
    connectedBLE = true;
    return true;
}

/* SEEK FOR SERVER (OVER DEFINED SVR NAME)*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    if (advertisedDevice.getName()=="Xsens DOT")
    {
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void onTwist(const geometry_msgs::Twist &cmd_vel)
{
  if (connected)
  {
    linearVelocityX = cmd_vel.linear.x;
    linearVelocityY = cmd_vel.linear.y;
    angularVelocityZ = cmd_vel.angular.z;
  }
  else
    stop();
 }

void handleMovement()
{


  // Naped Mecanum - kinematyka odwrotna:
  // ------------------------
  
  float wheel_FL_rad = 5.56 * (linearVelocityX - linearVelocityY - angularVelocityZ * 1.04);
  float wheel_RL_rad = 5.56 * (linearVelocityX + linearVelocityY - angularVelocityZ * 1.04);
  float wheel_FR_rad = 5.56 * (linearVelocityX + linearVelocityY + angularVelocityZ * 1.04);
  float wheel_RR_rad = 5.56 * (linearVelocityX - linearVelocityY + angularVelocityZ * 1.04);
  
  //ZAMIANA RAD/S NA RPM
  float wheel_FL_rpm = wheel_FL_rad * 9.5493;
  float wheel_RL_rpm = wheel_RL_rad * 9.5493;
  float wheel_FR_rpm = wheel_FR_rad * 9.5493;
  float wheel_RR_rpm = wheel_RR_rad * 9.5493;

  // SerialBT.printf("X: %f, Y: %f, Z: %f \n",linearVelocityX, linearVelocityY, angularVelocityZ);
  // SerialBT.printf("  ");
  // SerialBT.printf("FL: %f, RL: %f, FR: %f, RR: %f \n",wheel_FL_rad, wheel_RL_rad, wheel_FR_rad, wheel_RR_rad);
  // delay(50);
  
  //MODBUS COMMANDS TO WRITE SPEED

  //OBLICZANIE PREDKOSCI DLA PRZEDNIEGO LEWEGO KOŁA
  float Speed_FL_float = fabs(wheel_FL_rpm) * 10000;
  unsigned int Speed_FL = round(Speed_FL_float);
  unsigned int Speed_FL_1 = Speed_FL / 65536;
  unsigned int Speed_FL_2 = Speed_FL - (Speed_FL_1*65536);
  unsigned int Speed_FL_3 = 65536 - Speed_FL_1 - 1;
  unsigned int Speed_FL_4 = 65536 - Speed_FL_2;

   if (wheel_FL_rpm > 0)
  {
    Speed_FL_to_send_1 = Speed_FL_1;
    Speed_FL_to_send_2 = Speed_FL_2;
  }
  else if (wheel_FL_rpm < 0)
  {
    Speed_FL_to_send_1 = Speed_FL_3;
    Speed_FL_to_send_2 = Speed_FL_4;
  }
  else
  {
    Speed_FL_to_send_1 = 0;
    Speed_FL_to_send_2 = 0;
  }

  //OBLICZANIE PREDKOSCI DLA PRZEDNIEGO PRAWEGO KOŁA
  float Speed_FR_float = fabs(wheel_FR_rpm) * 10000;
  unsigned int Speed_FR = round(Speed_FR_float);
  unsigned int Speed_FR_1 = Speed_FR / 65536;
  unsigned int Speed_FR_2 = Speed_FR - (Speed_FR_1*65536);
  unsigned int Speed_FR_3 = 65536 - Speed_FR_1 - 1;
  unsigned int Speed_FR_4 = 65536 - Speed_FR_2;

   if (wheel_FR_rpm < 0)
  {
    Speed_FR_to_send_1 = Speed_FR_1;
    Speed_FR_to_send_2 = Speed_FR_2;
  }
  else if (wheel_FR_rpm > 0)
  {
    Speed_FR_to_send_1 = Speed_FR_3;
    Speed_FR_to_send_2 = Speed_FR_4;
  }
  else
  {
    Speed_FR_to_send_1 = 0;
    Speed_FR_to_send_2 = 0;
  }

  //OBLICZANIE PREDKOSCI DLA TYLNIEGO LEWEGO KOŁA
  float Speed_RL_float = fabs(wheel_RL_rpm) * 10000;
  unsigned int Speed_RL = round(Speed_RL_float);
  unsigned int Speed_RL_1 = Speed_RL / 65536;
  unsigned int Speed_RL_2 = Speed_RL - (Speed_FR_1*65536);
  unsigned int Speed_RL_3 = 65536 - Speed_RL_1 - 1;
  unsigned int Speed_RL_4 = 65536 - Speed_RL_2;

   if (wheel_RL_rpm > 0)
  {
    Speed_RL_to_send_1 = Speed_RL_1;
    Speed_RL_to_send_2 = Speed_RL_2;
  }
  else if (wheel_RL_rpm < 0)
  {
    Speed_RL_to_send_1 = Speed_RL_3;
    Speed_RL_to_send_2 = Speed_RL_4;
  }
  else
  {
    Speed_RL_to_send_1 = 0;
    Speed_RL_to_send_2 = 0;
  }

  //OBLICZANIE PREDKOSCI DLA TYLNIEGO PRAWEGO KOŁA
  float Speed_RR_float = fabs(wheel_RR_rpm) * 10000;
  unsigned int Speed_RR = round(Speed_RR_float);
  unsigned int Speed_RR_1 = Speed_RR / 65536;
  unsigned int Speed_RR_2 = Speed_RR - (Speed_RR_1*65536);
  unsigned int Speed_RR_3 = 65536 - Speed_RR_1 - 1;
  unsigned int Speed_RR_4 = 65536 - Speed_RR_2;

   if (wheel_RR_rpm < 0)
  {
    Speed_RR_to_send_1 = Speed_RR_1;
    Speed_RR_to_send_2 = Speed_RR_2;
  }
  else if(wheel_RR_rpm > 0)
  {
    Speed_RR_to_send_1 = Speed_RR_3;
    Speed_RR_to_send_2 = Speed_RR_4;
  }
  else
  {
    Speed_RR_to_send_1 = 0;
    Speed_RR_to_send_2 = 0;
  }



  regs[4] =  Speed_FL_to_send_1; //LEWO PRZOD REJ.1
  regs[5] =  Speed_FL_to_send_2; //LEWO PRZOD REJ.2
  regs[16] =  Speed_FR_to_send_1; //PRAWO PRZOD REJ.1
  regs[17] =  Speed_FR_to_send_2; //PRAWO PRZOD REJ.2
  regs[10] =  Speed_RL_to_send_1; //LEWO TYL REJ.1
  regs[11] =  Speed_RL_to_send_2; //LEWO TYL REJ.2
  regs[22] =  Speed_RR_to_send_1; //PRAWO TYL REJ.1
  regs[23] =  Speed_RR_to_send_2; //PRAWO TYL REJ.2
  connectionSpeed();
  modbus_update(); 
  
}



void setup() {

 nh.getHardware()->setBaud(115200);
 nh.initNode();
 nh.advertise(pub);
 nh.subscribe(sub);

 // BLE init
 Serial.begin(115200);
 Serial.println("Starting Arduino BLE Client application...");
 BLEDevice::init("");
 BLEScan* pBLEScan = BLEDevice::getScan();
 pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
 pBLEScan->setInterval(1349);
 pBLEScan->setWindow(449);
 pBLEScan->setActiveScan(true);
 pBLEScan->start(5, false);
 xTaskCreate(taskBLE, "Task_BLE",STACK_SIZE, NULL, 1, &taskBleHandle);
 xTaskCreate(taskRosSerial, "Task_ROS_Serial",STACK_SIZE, NULL, 1, &taskRosSerialHandle);
  
 // SerialBT.begin("ESP32");
 // Serial.println("The device started, now you can pair it with bluetooth!");

  //Część związana z MODBUS-em
  
  // Pierwszy napęd
  modbus_construct(&packets[PACKET17], 1, PRESET_SINGLE_REGISTER, 0x0028, 0x0000, 0); //motor enable
  modbus_construct(&packets[PACKET1], 1, PRESET_SINGLE_REGISTER, 0x0025, 0x0000, 1);  //tryb sterownika 0-pozycyjny, 1-predkosciowy
  modbus_construct(&packets[PACKET5], 1, PRESET_SINGLE_REGISTER, 0x0013, 0x0000, 2);  //wartosc przyspieszenia accel
  modbus_construct(&packets[PACKET9], 1, PRESET_SINGLE_REGISTER, 0x0015, 0x0000, 3);  //wartosc opoznienia decel
  modbus_construct(&packets[PACKET13], 1, PRESET_MULTIPLE_REGISTERS, 0x00B4, 2, 4);   //predkosc w 2 rejestrach

  //Drugi napęd
  modbus_construct(&packets[PACKET18], 2, PRESET_SINGLE_REGISTER, 0x0028, 0x0000, 6);
  modbus_construct(&packets[PACKET2], 2, PRESET_SINGLE_REGISTER, 0x0025, 0x0000, 7);
  modbus_construct(&packets[PACKET6], 2, PRESET_SINGLE_REGISTER, 0x0013, 0x0000, 8);
  modbus_construct(&packets[PACKET10], 2, PRESET_SINGLE_REGISTER, 0x0015, 0x0000, 9);
  modbus_construct(&packets[PACKET14], 2, PRESET_MULTIPLE_REGISTERS, 0x00B4, 2, 10);

  //Trzeci napęd
  modbus_construct(&packets[PACKET19], 3, PRESET_SINGLE_REGISTER, 0x0028, 0x0000, 12);
  modbus_construct(&packets[PACKET3], 3, PRESET_SINGLE_REGISTER, 0x0025, 0x0000, 13);
  modbus_construct(&packets[PACKET7], 3, PRESET_SINGLE_REGISTER, 0x0013, 0x0000, 14);
  modbus_construct(&packets[PACKET11], 3, PRESET_SINGLE_REGISTER, 0x0015, 0x0000, 15);
  modbus_construct(&packets[PACKET15], 3, PRESET_MULTIPLE_REGISTERS, 0x00B4, 2, 16);

  
  //Czwarty napęd
  modbus_construct(&packets[PACKET20], 4, PRESET_SINGLE_REGISTER, 0x0028, 0x0000, 18);
  modbus_construct(&packets[PACKET4], 4, PRESET_SINGLE_REGISTER, 0x0025, 0x0000, 19);
  modbus_construct(&packets[PACKET8], 4, PRESET_SINGLE_REGISTER, 0x0013, 0x0000, 20);
  modbus_construct(&packets[PACKET12], 4, PRESET_SINGLE_REGISTER, 0x0015, 0x0000, 21);
  modbus_construct(&packets[PACKET16], 4, PRESET_MULTIPLE_REGISTERS, 0x00B4, 2, 22);
  
  
  // Inicjalizacja maszyny stanowej
  modbus_configure(&Serial2, baud, SERIAL_8N1, timeout, 
  polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS,regs);


}

void taskBLE(void *pvParameters){
  while(1){
    if (doConnect == true) {
      if (connectToServer()) {
        Serial.println("We are now connected to the BLE Server.");
        charStart->writeValue((uint8_t*)notifyON,3, true);
      } else {
        Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      }
      doConnect = false;
    }
    if (connectedBLE) {
      // String newValue = "Time since boot: " + String(millis()/1000);
      // Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      // charMeasurement->writeValue(newValue.c_str(), newValue.length());
    }else if(doScan){
      BLEDevice::getScan()->start(0);  
    }

    delay(1000); 
  }
}

void taskRosSerial(void *pvParameters){
  StaticJsonDocument<256> jsonDoc;
  jsonDoc["o"]["x"] = quaternionX;
  jsonDoc["o"]["y"] = quaternionY;
  jsonDoc["o"]["z"] = quaternionZ;
  jsonDoc["o"]["w"] = quaternionW;

  jsonDoc["a"]["x"] = acceleratioX;
  jsonDoc["a"]["y"] = acceleratioY;
  jsonDoc["a"]["z"] = acceleratioZ;

  jsonDoc["v"]["x"] = angularXrad;
  jsonDoc["v"]["y"] = angularYrad;
  jsonDoc["v"]["z"] = angularZrad;

  char buffer[256];
  size_t jsonSize = serializeJson(jsonDoc, buffer);
  char formbuffer[256];
  // printf(buffer);
  // printf("\n");
  str_msg.data=buffer;
  // str_msg.data="HELLO_ROS";
  pub.publish(&str_msg);
  // nh.spinOnce();
  nh.spinOnce();
  rosConnected();
}


void loop() {
  /* o - orientation, a - acceleration, v - angular velocity*/

  motorsON = 1;
  
  // Record the time
  currentMillis = millis();


  //MODBUS
  modbus_update();
  //USTAWIENIE TRYBU PREDKOSCIOWEGO I WARTOSCI PRZYSPIESZENIA
  //Pierwszy silnik
  regs[1] = 0x0001;
  regs[2] = 0x0019;
  regs[3] = 0xFFE7;
  //Drugi silnik
  regs[7] = 0x0001;
  regs[8] = 0x0019;   //2.5 rps2
  regs[9] = 0xFFE7;   //-2.5 rps2
  //Trzeci silnik
  regs[13] = 0x0001;
  regs[14] = 0x0019;
  regs[15] = 0xFFE7;
  //Czwarty silnik
  regs[19] = 0x0001;
  regs[20] = 0x0019;
  regs[21] = 0xFFE7;
  
 //Włączenie silników

   if (motorsON == 1) {

    digitalWrite(2, LOW); 
    regs[0] = 0x0001 ;  
    regs[6] = 0x0001 ;   
    regs[12] = 0x0001 ;  
    regs[18] = 0x0001 ;   
    packets[0].connection = 1; packets[1].connection = 1; packets[2].connection = 1; packets[3].connection = 1;
    packets[4].connection = 1; packets[5].connection = 1; packets[6].connection = 1; packets[7].connection = 1;
    packets[8].connection = 1; packets[9].connection = 1;
    packets[10].connection = 1; packets[11].connection = 1; packets[12].connection = 1; packets[13].connection = 1; 
    packets[14].connection = 1; packets[15].connection = 1; packets[16].connection = 1; packets[17].connection = 1; 
    packets[18].connection = 1; packets[19].connection = 1; 
     
    modbus_update();  
    }
  
  modbus_update();
  handleMovement();
  delay(10);
}

//
bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool conn = nh.connected();
  if (connected != conn)
  {
    connected = conn;
    if (!connected)
      stop();
      
    digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

void stop()
{
  linearVelocityX = linearVelocityY = angularVelocityZ = 0;
  
}
