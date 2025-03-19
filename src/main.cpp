/*
  ModbusRTU ESP8266/ESP32
  Simple slave example

  (c)2019 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  modified 13 May 2020
  by brainelectronics

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/
#include <SPI.h>
#include <AD7190.h>
#include <ModbusRTU.h>
#include <Wire.h>


#define ADDR_Ax 0b000 //A2, A1, A0
#define ADDR (0b1010 << 3) + ADDR_Ax

/***********define CS two AD7190*****************************/
volatile uint8_t AD7190_CS = 0x00; //Not used pin CS selected
#define AD7190__1  32
#define AD7190__2  33
#define AD7190_RDY 27

#define SAMPLE_DELAY_INTERVAL     1
AD7190* ad7190_1 = NULL;
//AD7190* ad7190_2 = NULL;


#define MAIN_DEBUG_VERBOSE

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2 // Specify the on which is your LED
#endif

extern bool flag_WriteReg;
extern uint16_t WriteReg_Address;


#define MBUS_HW_SERIAL Serial2
#define RXTX_PIN 26
#define RXD2  GPIO_NUM_16 
#define TXD2 GPIO_NUM_17
#define REG_NUM 100
#define REG_Value 0
#define REG_Offset 0
#define SLAVE_ID 1
uint16_t offset = 1;
uint16_t value = 0;
uint16_t numregs = 20;

static const int VspiClk = 5000000; // 5 MHz
SPIClass * vspi = NULL;

  typedef struct{
			double coef;
			float coef_Risult;
			signed  int   Zero_Offset;
			signed    int   weight;
			signed  int   setpoint;
			unsigned  char  update_Rate;
			unsigned  char  Read[2];
			unsigned  char  write[2];
			unsigned  int 	analog_Read;
	    unsigned  int bytesRx1;
	    unsigned  int bytesRx2;
	    uint32_t analog_get;
      uint32_t analog_RECV;
	    unsigned  int Address;
	    float float_var;
       }ADC_t ;
			  ADC_t  analog1;
        ADC_t  analog2;
				ADC_t  EE_RW;

//volatile uint32_t AD7190_1_data = 0;
 
const int QueueElementSize = 1;
#define MAX_LINE_LENGTH (10)

typedef struct{
  char line[MAX_LINE_LENGTH];
  uint8_t line_length;
} message_t;

// Define two tasks for Blink & AnalogRead.
void TaskBlink( void *pvParameters );
void Task_Modbus( void *pvParameters );
void Task_AD7190( void *pvParameters );

//--> freeRTOS Queue
QueueHandle_t queue_AD7190;
// This TaskHandle will allow 
TaskHandle_t TaskBlink_Handle = NULL;
TaskHandle_t Task_Modbus_Handle;
TaskHandle_t Task_AD7190_Handle;


ModbusRTU mb;
/***********************************************************************/
void reset_ad7190_2(){

  //  The serial interface can be reset by writing a series of 1s to the
  //  DIN input. If a Logic 1 is written to the AD7190 DIN line for at
  //  least 40 serial clock cycles, the serial interface is reset. This ensures
  //  that the interface can be reset to a known state if the interface gets
  //  lost due to a software error or some glitch in the system. Reset
  //  returns the interface to the state in which it is expecting a write to
  //  the communications register. This operation resets the contents of
  //  all registers to their power-on values. Following a reset, the user
  //  should allow a period of 500 µs before addressing the serial
  //  interface.

  unsigned char register_word[6];         // At least 40? 5*8? 6*8?
  register_word[0] = 0xFF;                // can be reduced to only 5 bytes
  register_word[1] = 0xFF;
  register_word[2] = 0xFF;
  register_word[3] = 0xFF;
  register_word[4] = 0xFF;
  register_word[5] = 0xFF;
 
  vspi->beginTransaction(SPISettings(VspiClk, MSBFIRST, SPI_MODE3));
  vspi->transfer(register_word, sizeof(register_word));
   vspi->endTransaction();

  //  Following a reset, the user should allow a period of 500 µs 
  //  before addressing the serial interface. 
  delay(500);
}
/******************************************************/
void setRegisterValue(unsigned char registerAddress, uint32_t registerValue, unsigned char bytesNumber) {


  unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
  unsigned char* dataPointer    = (unsigned char*)&registerValue;
  unsigned char bytesNr         = bytesNumber;

  writeCommand[0] = AD7190_COMM_WRITE | AD7190_COMM_ADDR(registerAddress);

  while (bytesNr > 0) {
    writeCommand[bytesNr] = *dataPointer;
    dataPointer ++;
    bytesNr --;
  }
  vspi->beginTransaction(SPISettings(VspiClk, MSBFIRST, SPI_MODE3));
  vspi->transfer(writeCommand, bytesNumber + 1);
  vspi->endTransaction();
}

/******************************************************************/
void setModeContinuousRead(uint8_t commRegValue) {

  vspi->beginTransaction(SPISettings(VspiClk, MSBFIRST, SPI_MODE3));
  vspi->transfer(commRegValue);                 // send the device the register you want to read:
  vspi->endTransaction();
  
}
/*************************************************************/
uint32_t getRegisterValue(byte registerAddress, uint8_t bytesNumber) {
  
 byte inByte = 0;           // incoming byte from the SPI
  uint32_t result = 0;       // result to return

  uint8_t address = AD7190_COMM_READ | AD7190_COMM_ADDR(registerAddress);


  vspi->beginTransaction(SPISettings(VspiClk, MSBFIRST, SPI_MODE3));

  vspi->transfer(address);                      // send the device the register you want to read:

  result = vspi->transfer(0x00);                // Send a value of 0 to read the first byte returned:

  bytesNumber--;                                    // decrement the number of bytes left to read:
  while (bytesNumber > 0) {                         // if you still have another byte to read:
    result = result << 8;                           // shift the first byte left,
    inByte = vspi->transfer(0x00);              // then get the second byte:
    result = result | inByte;                       // combine the byte you just got with the previous ones
    bytesNumber--;  
    }
     vspi->endTransaction();
        return (result);                      
  }
  
/******************************************************/
void writeI2CByte(byte data_addr, byte data){
  Wire.beginTransmission(ADDR);
  Wire.write(data_addr);
  Wire.write(data);
  Wire.endTransmission();
}
/******************************************************/
byte readI2CByte(byte data_addr){
  byte data = 0;
  Wire.beginTransmission(ADDR);
  Wire.write(data_addr);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, 1); //retrieve 1 returned byte
  delay(1);
  if(Wire.available()){
    data = Wire.read();
  }
  return data;
}
/*************************************************************/
void Eeprom_Write_Obj(word addr,void *obj,byte size)
{
      byte i,*ptr=(byte *)obj;

      for (i=0;i<size;i++){
            writeI2CByte(addr++,*(ptr++));
             delay(10);
            }
}
/******************************************************/
void Eeprom_Read_Obj(word addr,void *obj,byte size)
{
      byte i,*ptr= (byte *)obj;

      for (i=0;i<size;i++){
            *(ptr++)=readI2CByte(addr++);
            delay(10);
      }
}
/*********************************************************************************/
                             //setup()
/*********************************************************************************/
void setup() {
// Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  // Set up two tasks to run independently.
  uint32_t blink_delay = 1000; // Delay between changing state on LED pin
  pinMode(AD7190__1, OUTPUT);
  pinMode(AD7190__2, OUTPUT);
  pinMode(AD7190_RDY, INPUT_PULLUP);

  Wire.begin();
  delay(10); 
 
//restore Zero_Offset & coef at startup
  // analog1.Zero_Offset =  (EEPROM.read(1) << 8 | EEPROM.read(0));
  Eeprom_Read_Obj(0 , &analog1.Zero_Offset , sizeof(analog1.Zero_Offset));
   delay(10);
  Eeprom_Read_Obj(4 , &analog1.coef_Risult , sizeof(analog1.coef_Risult));
  delay(10);
  
 // Initialise SPI Port HSPI with default pins:
  // SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  
  SPIClass* spi = new SPIClass(HSPI);

  uint8_t hspi_MISO_pin = 12;  
  ad7190_1 = new AD7190(spi, hspi_MISO_pin);
  //ad7190_2 = new AD7190(spi, hspi_MISO_pin);
  // Its also possible to use VSPI port
  // Initialise SPI Port V SPI with default pins:
  // SCLK = 18, MISO = 19, MOSI = 23, SS = 5

   
     vspi = new SPIClass(VSPI);
      //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
     vspi->begin(); // --> AD7190_2
   
  // uint8_t vspi_MISO_pin = 19;  
  // ad7190_2 = new AD7190(spi_VSPI, vspi_MISO_pin);

 
 
 

queue_AD7190 = xQueueCreate(QueueElementSize, sizeof(QueueElementSize));
if (queue_AD7190 == NULL) {
Serial.println("Queue can not be created");
}

xTaskCreatePinnedToCore(
    TaskBlink
    ,  "Task Blink" // A name just for humans
    ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  (void*) &blink_delay // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  2  // Priority
    ,  NULL // Task handle is not used here - simply pass NULL
    ,  1 // Core on which the task will run
    );

  // This variant of task creation can also specify on which core it will be run (only relevant for multi-core ESPs)
  xTaskCreatePinnedToCore(
    Task_Modbus
    ,  "Modbus_send_Receive"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  1  // Priority
    ,  &Task_Modbus_Handle // With task handle we will be able to manipulate with this task.
    ,  1 // Core on which the task will run
    );

   xTaskCreatePinnedToCore(
    Task_AD7190
    ,  "Analog Read"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  1  // Priority
    ,  &Task_Modbus_Handle // With task handle we will be able to manipulate with this task.
    ,  1 // Core on which the task will run
    ); 
   
//Serial.printf("Basic Multi Threading Arduino Example\n");
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.

}
/*****************************************/
void loop() {
 
}
/*************************************************************************************************************/
                                                //Task_Modbus
/*************************************************************************************************************/
void Task_Modbus(void *pvParameters){  // This is a task.
 
       static uint8_t i1=0;
       static signed int avrage1=0;
       static signed int samples1[15];
       uint8_t x1=0;
       uint32_t rawAd7190_1Data = 0x00;

    uint32_t Receive_data1 = 0;
    long ts;
    uint16_t Holding_Write = 0;

  ts = millis();
  pinMode(RXTX_PIN, OUTPUT);
  MBUS_HW_SERIAL.begin(9600, SERIAL_8N1 , RXD2, TXD2);
#if defined(ESP32) || defined(ESP8266)
  mb.begin(&MBUS_HW_SERIAL);
  mb.begin(&MBUS_HW_SERIAL, RXTX_PIN);
#else
  mb.begin(&MBUS_HW_SERIAL);
  //mb.begin(&Serial2, RXTX_PIN);  //or use RX/TX direction control pin (if required)
  mb.setBaudrate(9600);
#endif
  mb.slave(SLAVE_ID);

  mb.addHreg(REG_Offset , REG_Value , REG_NUM); // Add 100 holding register: addHreg(offset = 0, value = 0, numregs = 100)

  flag_WriteReg = 0;
/****************************************************************/
  for (;;){
       mb.task();
      // xQueueReceive(queue_AD7190, &Receive_data1, 5);
      // Serial.println(Receive_data1);
             samples1[i1] = Receive_data1;
                      i1++;
                  if(i1 == 14)
                    {  
                      for(x1=0 ; x1<15 ; x1++)
                        avrage1 += samples1[x1];
                        avrage1 /= 15; 
                        i1=0;
                    }	
              //Serial.println(i1);			
             analog1.weight=(avrage1 - analog1.Zero_Offset);//analog1.analog_Read
             analog1.weight *= analog1.coef_Risult;
             mb.Hreg(0,  analog1.weight);
    
/******************************************************************/
// Function code6 Write Single Register
/***************************************************************/
 if(flag_WriteReg == 1){ 
   /////////////////ZERO SCALE1/40042//////////////////////
      if((WriteReg_Address == 42) && (mb.Hreg(42) == 1)) //ZERO SCALE1
		  	{	
          analog1.Zero_Offset = avrage1;
          //Serial.println(mb.Hreg(42));
         Eeprom_Write_Obj(0 , &analog1.Zero_Offset , sizeof(analog1.Zero_Offset)); 
         flag_WriteReg = 0;
			    }		
     ////////////////Full SCALE1/40043//////////////////////
	    if(WriteReg_Address == 43)				
          {
            // Serial.println(mb.Hreg(43));
             analog1.coef = mb.Hreg(43);
             analog1.coef_Risult = analog1.coef/(avrage1 - analog1.Zero_Offset);
            // Serial.println(analog1.coef_Risult); 
            Eeprom_Write_Obj(4 , &analog1.coef_Risult , sizeof(analog1.coef_Risult)); 
            flag_WriteReg = 0;
           }
     } 
      // delay(1);
  //  if (millis() > ts + 1000) {
  //      ts = millis();
       
  //      mb.Hreg(0,  Receive);
  //      Serial.println(mb.Hreg(3));
          //Serial.println(Receive);
  //        }
    }
  }
 
 /*********************************************************************************************************************/ 
                                             //Task_AD7190
/**********************************************************************************************************************/
void Task_AD7190(void *pvParameters){  // This is a task.
      uint16_t count = 0;
      bool timeOut = false;
       uint32_t rawAd7190_1Data = 0x00;
       uint32_t rawAd7190_2Data = 0x00;
      // uint32_t sampleTime = 0x0;
      // static	signed int  adc_data;
      //  static uint8_t i1=0;
      //  static signed int avrage1=0;
      //  static signed int samples1[10];
      //  uint8_t x1=0;

       static uint8_t i2=0;
       static signed int avrage2=0;
       static signed int samples2[10];
       uint8_t x2=0;

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;
 
/******************define AD7190__1*****************************/
 
        if(ad7190_1->begin()){
            Serial.println(F("AD7190_1 begin: OK"));
            Serial.print("Device name: ");
            Serial.println(ad7190_1->getDeviceName());
          }else{
            Serial.println(F("AD7190_1 begin: FAIL"));
          }

          uint32_t ad7190_1_regConfigSettings = (AD7190_CONF_REFSEL | AD7190_CONF_CHAN(AD7190_CH_AIN1P_AIN2M) | AD7190_CONF_GAIN(AD7190_CONF_GAIN_128) | AD7190_CONF_BUF);   
          ad7190_1->setRegisterValue(AD7190_REG_CONF, ad7190_1_regConfigSettings, 3 );
          delay(1);
          uint32_t ad7190_1_regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(AD7190_FILTER_RATE_80));
          ad7190_1->setRegisterValue(AD7190_REG_MODE, ad7190_1_regModeSettings, 3 );
          delay(1);
         // Set Continuous Read Mode:
          uint8_t ad7190_1_regCommSetting = (AD7190_COMM_READ | AD7190_COMM_ADDR(AD7190_REG_DATA) | AD7190_COMM_CREAD);
          ad7190_1->setModeContinuousRead(ad7190_1_regCommSetting);
          delay(1);
/******************define AD7190__2**************************************/
          reset_ad7190_2();
          
          uint32_t ad7190_2_regConfigSettings = (AD7190_CONF_REFSEL | AD7190_CONF_CHAN(AD7190_CH_AIN1P_AIN2M) | AD7190_CONF_GAIN(AD7190_CONF_GAIN_128) | AD7190_CONF_BUF);
          setRegisterValue(AD7190_REG_CONF, ad7190_2_regConfigSettings, 3 );
          delay(1);
          
          uint32_t ad7190_2_regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(AD7190_FILTER_RATE_80));
          setRegisterValue(AD7190_REG_MODE, ad7190_2_regModeSettings, 3 );
          delay(1);
         
        // Set Continuous Read Mode:
          uint8_t ad7190_2_regCommSetting = (AD7190_COMM_READ | AD7190_COMM_ADDR(AD7190_REG_DATA) | AD7190_COMM_CREAD);
          vspi->beginTransaction(SPISettings(VspiClk, MSBFIRST, SPI_MODE3));
          vspi->transfer(ad7190_2_regCommSetting);
          vspi->endTransaction();
          delay(1);
 
/******************************************************/
     for (;;){
  
    // /***************Read AD7190_1 and send to Task_Modbus***************************************/
         // AD7190_CS = AD7190__1;
    // //        // timeOut = !ad7190_1->waitMisoGoLow();
    // //       // rawAd7190Data = ad7190->getDataRegisterAvg(3);
    // //       //    digitalWrite(AD7190_CS, LOW); 
    // //       //    uint8_t rdy1Pin = digitalRead(AD7190_RDY); 
    // //       //  if(rdy1Pin == 0 ){
           
           rawAd7190_1Data = ad7190_1->getRegisterValue(AD7190_REG_DATA, 3);
           analog1.analog_Read = rawAd7190_1Data = rawAd7190_1Data >> 8;
    //          // }
   
                delay(10);
            //analog1.analog_Read = rawAd7190_1Data = rawAd7190_1Data >> 8;
           // }
           // AD7190_CS = AD7190__2;
    //          // read a reference value from A0 and map it from 0 to 100
    //         // float real_value = rawAd7190_1Data;
            
     	
             //xQueueSend(queue_AD7190, &analog1.analog_Read, portMAX_DELAY);//portMAX_DELAY
    //         //count++;
              //  delay(10);
    //    /***************Read AD7190_2 and send to Task_Modbus***************************************/
                   rawAd7190_2Data = getRegisterValue(AD7190_REG_DATA, 3);
                   analog2.analog_Read = rawAd7190_2Data = rawAd7190_2Data >> 8;

       
          delay(10);
                   // send to Serial output every 100ms
           // use the Serial Ploter for a good visualization
            if (millis() > refresh_time) {
            //  Serial.print(analog1.analog_Read,4);
            //   Serial.print(",");
            //   Serial.print(analog2.analog_Read,4);
            //   Serial.print(",");
            //   Serial.print(estimated_value,4);
            //   Serial.println();
             // Serial.println(AD7190_CS);
              Serial.println(analog1.analog_Read, DEC);
            //  delay(10);
             Serial.println(analog2.analog_Read, DEC);
               refresh_time = millis() + SERIAL_REFRESH_TIME;
             }   
            
     }
}

/*********************************************************************************************************************/
                                                //TaskBlink
/********************************************************************************************************************/
  void TaskBlink(void *pvParameters){  // This is a task.

        //   uint8_t Temp[2]={0};
        //   float buf = 0;
        //   float num = 1.84;

        // Eeprom_Write_Obj(21 , &num , sizeof(num));
        // delay(10);
      
       // initialize digital LED_BUILTIN on pin 2 as an output.
        pinMode(LED_BUILTIN, OUTPUT);

  for (;;){ // A Task shall never return or exit.

        //  Eeprom_Read_Obj(21 , &buf , sizeof(buf));
        //  Serial.println(buf);
        //  delay(10);

     digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    // arduino-esp32 has FreeRTOS configured to have a tick-rate of 1000Hz and portTICK_PERIOD_MS
    // refers to how many milliseconds the period between each ticks is, ie. 1ms.
     delay(1000);
     digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
     delay(1000);
  }
}

