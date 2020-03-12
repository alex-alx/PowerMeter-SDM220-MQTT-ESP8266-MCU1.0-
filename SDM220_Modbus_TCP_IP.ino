/* ***************************************
 * >>>>>>>>>>>>   Alx homesys 2018 <<<<<<<<<<<<<<<<
 *  **************************************
 *  
 *  1. Power meter MDS220 -> Modbus 485 TTL -> Modbus TCP-IP (port 502)
 *  2. Other side Raspberry PI with Modbus master and pooling data over (port502)
 *  
 *  Additionally added display for testing purposes (#include <LiquidCrystal_I2C.h>) and some /Comment lines/
 *  
 */

#include <ESP8266WiFi.h>
#include <SDM.h>  
//#include <LiquidCrystal_I2C.h>
 
const char* ssid = "your SSid";
const char* password = "your password";
int ModbusTCP_port = 502;

//setting the addresses
IPAddress ip(192, 168, xx, xx);
IPAddress gateway(192, 168, xx, xx);
IPAddress subnet(255, 255, 255, 0);
 
 
//////// Required for Modbus TCP / IP ///
#define maxInputRegister 20
#define maxHoldingRegister 20
 
#define MB_FC_NONE 0
#define MB_FC_READ_REGISTERS 3 //implemented
#define MB_FC_WRITE_REGISTER 6 //implemented
#define MB_FC_WRITE_MULTIPLE_REGISTERS 16 //implemented
//
// MODBUS Error Codes
//
#define MB_EC_NONE 0
#define MB_EC_ILLEGAL_FUNCTION 1
#define MB_EC_ILLEGAL_DATA_ADDRESS 2
#define MB_EC_ILLEGAL_DATA_VALUE 3
#define MB_EC_SLAVE_DEVICE_FAILURE 4
//
// MODBUS MBAP offsets
//
#define MB_TCP_TID 0
#define MB_TCP_PID 2
#define MB_TCP_LEN 4
#define MB_TCP_UID 6
#define MB_TCP_FUNC 7
#define MB_TCP_REGISTER_START 8
#define MB_TCP_REGISTER_NUMBER 10

SDM<9600,12,13,2>sdm; //SDM reading parameters (baud, rx pin, tx pin,DE amd RE pins);


// Addr: 0x3F, 20 chars & 4 lines. Sometimes display boards use address 0x27
//LiquidCrystal_I2C lcd(0x27, 16, 2); //Frentally display, use 0x3F if not working try 0x27 
 
byte ByteArray[60];
unsigned int MBHoldingRegister[maxHoldingRegister];
 
//////////////////////////////////////////////////////////////////////////
 
WiFiServer MBServer(ModbusTCP_port);
 
void setup() {
  

/*
//Initalize LCD
  lcd.init();
  lcd.noBacklight();
  lcd.backlight();
  lcd.begin(16,2); */
  
 pinMode(14, OUTPUT);
 
 Serial.begin(9600);
 sdm.begin(); // added by alx
 delay(100) ;
 WiFi.mode(WIFI_STA); // no access point ALX 
 WiFi.begin(ssid, password);
 WiFi.config(ip,gateway,subnet);
 delay(100) ;
 Serial.println(".");
 while (WiFi.status() != WL_CONNECTED) {
 delay(500);
 Serial.print(".");
 }
 MBServer.begin();

/* 
// Show WiFi status on LCD along with SSID of network
    lcd.setCursor(0, 0);
    lcd.print("WiFi: ");
    lcd.print(ssid);  */

    
 
 Serial.println("Connected ");
 Serial.print("ESP8266 Slave Modbus TCP/IP ");
 Serial.print(WiFi.localIP());
 Serial.print(":");
 Serial.println(String(ModbusTCP_port));
 Serial.println("Modbus TCP/IP Online");
 

}
 
 
void loop() {
 
 
 // Check if a client has connected // Modbus TCP/IP
 WiFiClient client = MBServer.available();
 if (!client) {
 return;
 }
 
 
 boolean flagClientConnected = 0;
 byte byteFN = MB_FC_NONE;
 int Start;
 int WordDataLength;
 int ByteDataLength;
 int MessageLength;
 
 // Modbus TCP/IP
 while (client.connected()) {



 if(client.available())
 {
 flagClientConnected = 1;
 int i = 0;
 while(client.available())
 {
 ByteArray[i] = client.read();
 i++;
 }
 
 client.flush();

 // *********************   measurments from SDM220 ***********************************************
 
 String volt_str = String(sdm.readVal(SDM220T_VOLTAGE), 0);
  long volt = volt_str.toInt();
  
 String curr_str = String(sdm.readVal(SDM220T_CURRENT), 0);
  long curr = curr_str.toInt(); 

 String powr_str = String(sdm.readVal(SDM220T_POWER), 0);
  long powr = powr_str.toInt(); 
  
 String freq_str = String(sdm.readVal(SDM220T_FREQUENCY), 0);
  long freq = freq_str.toInt(); 
  
 String Apwr_str = String(sdm.readVal(SDM220T_TOTAL_ACTIVE_ENERGY), 0);
  long Apwr = Apwr_str.toInt(); 

 String pwrfctr_str = String(sdm.readVal(SDM220T_POWER_FACTOR), 0);
  long pwrfctr = pwrfctr_str.toInt(); 

/*
    lcd.setCursor(0, 1);
    lcd.print(volt);
    lcd.print("V ");
    lcd.print(freq);
    lcd.print("Hz ");  */
    
  
// *************************************************************************


 
 ///////// Holding Register [0] A [9] = 10 Holding Registers Escritura
 ///////// Holding Register [0] A [9] = 10 Holding Registers Writing
 
 MBHoldingRegister[0] = volt;
 MBHoldingRegister[1] = curr;
 MBHoldingRegister[2] = powr;
 MBHoldingRegister[3] = freq;
 MBHoldingRegister[4] = Apwr;
 MBHoldingRegister[5] = pwrfctr;
 MBHoldingRegister[6] = 0;
 MBHoldingRegister[7] = 0;
 MBHoldingRegister[8] = 0;
 MBHoldingRegister[9] = 0; //random(0,12);
 
 
 
 
 ///////// Holding Register [10] A [19] = 10 Holding Registers Lectura
 ///// Holding Register [10] A [19] = 10 Holding Registers Reading
 
 int Temporal[10];
 
 Temporal[0] = MBHoldingRegister[10];
 Temporal[1] = MBHoldingRegister[11];
 Temporal[2] = MBHoldingRegister[12];
 Temporal[3] = MBHoldingRegister[13];
 Temporal[4] = MBHoldingRegister[14];
 Temporal[5] = MBHoldingRegister[15];
 Temporal[6] = MBHoldingRegister[16];
 Temporal[7] = MBHoldingRegister[17];
 Temporal[8] = MBHoldingRegister[18];
 Temporal[9] = MBHoldingRegister[19];
 
 /// Enable Output 14
 digitalWrite(14, MBHoldingRegister[14] );
 
 
 //// debug
 
 for (int i = 0; i < 10; i++) {
 
 Serial.print("[");
 Serial.print(i);
 Serial.print("] ");
 Serial.print(Temporal[i]);
 
 }
 Serial.println("");
 
 
 
 
//// end code - fin 
 
 
 //// rutine Modbus TCP
 byteFN = ByteArray[MB_TCP_FUNC];
 Start = word(ByteArray[MB_TCP_REGISTER_START],ByteArray[MB_TCP_REGISTER_START+1]);
 WordDataLength = word(ByteArray[MB_TCP_REGISTER_NUMBER],ByteArray[MB_TCP_REGISTER_NUMBER+1]);
 }
 
 // Handle request
 
 switch(byteFN) {
 case MB_FC_NONE:
 break;
 
 case MB_FC_READ_REGISTERS: // 03 Read Holding Registers
 ByteDataLength = WordDataLength * 2;
 ByteArray[5] = ByteDataLength + 3; //Number of bytes after this one.
 ByteArray[8] = ByteDataLength; //Number of bytes after this one (or number of bytes of data).
 for(int i = 0; i < WordDataLength; i++)
 {
 ByteArray[ 9 + i * 2] = highByte(MBHoldingRegister[Start + i]);
 ByteArray[10 + i * 2] = lowByte(MBHoldingRegister[Start + i]);
 }
 MessageLength = ByteDataLength + 9;
 client.write((const uint8_t *)ByteArray,MessageLength);
 
 byteFN = MB_FC_NONE;
 
 break;
 
 
 case MB_FC_WRITE_REGISTER: // 06 Write Holding Register
 MBHoldingRegister[Start] = word(ByteArray[MB_TCP_REGISTER_NUMBER],ByteArray[MB_TCP_REGISTER_NUMBER+1]);
 ByteArray[5] = 6; //Number of bytes after this one.
 MessageLength = 12;
 client.write((const uint8_t *)ByteArray,MessageLength);
 byteFN = MB_FC_NONE;
 break;
 
 case MB_FC_WRITE_MULTIPLE_REGISTERS: //16 Write Holding Registers
 ByteDataLength = WordDataLength * 2;
 ByteArray[5] = ByteDataLength + 3; //Number of bytes after this one.
 for(int i = 0; i < WordDataLength; i++)
 {
 MBHoldingRegister[Start + i] = word(ByteArray[ 13 + i * 2],ByteArray[14 + i * 2]);
 }
 MessageLength = 12;
 client.write((const uint8_t *)ByteArray,MessageLength); 
 byteFN = MB_FC_NONE;
 
 break;
  }
 }
}
