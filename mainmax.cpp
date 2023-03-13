#include <Arduino.h>
#include <U8g2lib.h>
#include <iostream>
#include <string>
#include <STM32FreeRTOS.h>

//Constants
  const uint32_t interval = 100; //Display update interval

  //mutex
  SemaphoreHandle_t keyArrayMutex;

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;
  //other
  volatile uint32_t currentStepSize;
  volatile uint8_t keyArray[7];
  volatile int step;
  volatile int counter=0;
  volatile int state=0;
  volatile int state2=0;
  volatile int octave =0;
  volatile int32_t alpha = 0;
  volatile int32_t vstep = 0;
  volatile int delayen = 1;
  //Step Size Array
  const uint32_t stepSizes [13] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007189, 96418756, 0 };
  const std::string notes[13] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B","No key pressed"};
  const uint32_t vibrato[11] = {3,30,300,3000,30000,300000,30000,3000,300,30,3};


//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);
  unsigned char RA2 = rowIdx & 0b00000100; //RA2
  unsigned char RA1 = rowIdx & 0b00000010;//RA1
  unsigned char RA0 = rowIdx & 0b00000001;//RA2
  digitalWrite(RA2_PIN, RA2);
  digitalWrite(RA1_PIN, RA1);
  digitalWrite(RA0_PIN, RA0);
  digitalWrite(REN_PIN,HIGH);
 // Serial.println("setrow");

}

uint8_t readCols(){
  int a= digitalRead(C0_PIN);
  int b = digitalRead(C1_PIN);
  int c = digitalRead(C2_PIN);
  int d = digitalRead(C3_PIN);
  uint8_t colout = (a<<3) | (b<<2) | (c<<1) | (d);
  return colout;
  //Serial.println("READCOLS");
  //std::string a= std::to_string(digitalRead(C0_PIN));
  //std::string b= std::to_string(digitalRead(C1_PIN));
  //std::string c= std::to_string(digitalRead(C2_PIN));
  //std::string d= std::to_string(digitalRead(C3_PIN));
  //std::string out = "0000"+a+b+c+d;
//  uint8_t outint = std::stoi(out);
 // return outint;
}

//int step = -1;
void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
 // Serial.println("DISPLAY MAIN");
while(1){
  vTaskDelayUntil( &xLastWakeTime, xFrequency );
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(2,20);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.print(keyArray[2], HEX);
    u8g2.print(" ");
    u8g2.print(keyArray[1], HEX);
    u8g2.print(" ");
    u8g2.print(keyArray[0], HEX);
    u8g2.drawStr(2, 10, notes[step].c_str());
    u8g2.setCursor(2,30);
    u8g2.print(counter);
    u8g2.print("     octave:");
    u8g2.print(octave);
    u8g2.sendBuffer();     
    xSemaphoreGive(keyArrayMutex);
  //  Serial.println("DISPLAY LOOP"); // transfer internal memory to the display
    

    //Toggle LED
    digitalToggle(LED_BUILTIN);
}

}

void vibratoTask(void * pvParameters){
  const TickType_t xFrequency = 500/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
   while (1){
      
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    int i =1000000;
  
  for(int h=0; h<=i;h++){
    vstep = vstep+1;
  }
  for(int d=0;d<=i*2;d++){
    vstep = vstep-1;
  }
  for(int o=0; o<=i;o++){
    vstep = vstep+1;
  }
   
  
   }


}


void sampleISR()
{
  static uint32_t phaseAcc = 0;
  //Serial.println(currentStepSize);
  //Serial.println(octave);
  
  //Serial.println(vstep);
  phaseAcc += (currentStepSize + vstep);
  
  int32_t Vout = (phaseAcc >> 24) - 128;
  if (counter <=8){
    if (counter >= 0){
      Vout = Vout >> (8 - counter);
    }
  }
  int alpha = 0.2;

 int32_t filtered = (1 - 0.15) * filtered + 0.15 * Vout;
  
  analogWrite(OUTR_PIN, Vout + 128);
 // Serial.println("isr");
//  if(delayen = 1){
//   if (counter >= 0){
     
    
//   delay(2);
//   int32_t Vout2 = Vout >> (8-(counter - 1));
//   analogWrite(OUTR_PIN, Vout2  + 128);
//   delay(2);
//   int32_t Vout3 = Vout2 >> (8-(counter - 1));
//   analogWrite(OUTR_PIN, Vout3  + 128);
//   }

//  }
}



void scanKeysTask(void * pvParameters) {
    const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

   // Serial.println("KEYS MAIN");
    while (1){
      
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    //Serial.println("KEYSLOOP");
    step = 12;
    for (uint8_t i = 0; i < 6; i++) {
      setRow(i);
      delayMicroseconds(3);
      uint8_t keys = readCols();
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      keyArray[i] = keys;
      xSemaphoreGive(keyArrayMutex);

    }
    //Rotation detection Knob 3
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint8_t test = keyArray[3]; 
    xSemaphoreGive(keyArrayMutex); 
    uint8_t maskA = 0b1100;  
    uint8_t Abit = (test & maskA) >> 2;
    uint8_t maskB = 0b0100;  
    uint8_t Bbit = (test & maskB) >> 3; //gives me A and B
    
    
    switch(state){
      case 0:
      if(Abit == 0b11){
        counter = counter-1;
        state = 1;
      }
      if(Abit == 0b10){
        counter=counter+1;
        state = 1;
        //delayMicroseconds(1);

      }
      if(Abit == 0b00){
        
        state = 0;

      }
      if(Abit == 0b01){
        
        state = 0;

      }
      break;
      case 1:
      if(Abit== 0b10){
        counter = counter-1;
        state = 0;
        
      }
      if(Abit ==0b00){
        counter = counter+1;
        state = 0;
        //delayMicroseconds(1);
      }
      if(Abit ==0b11){
        
        state = 1;
      }
      if(Abit ==0b01){
        state = 1;
      }
      break;

    }
    if (counter >= 8){
      counter =8;
    }
    if (counter <= 0 ){
      counter =0;
    }
   //end of knob control 
   //knob 4 - octave

  //  xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
  //   uint8_t test2 = keyArray[4]; 
  //   xSemaphoreGive(keyArrayMutex); 
  //   uint8_t maskA2 = 0b1100;  
  //   uint8_t Abit2 = (test2 & maskA2) >> 2;
    
  //   switch(state2){
  //     case 0:
  //     if(Abit2 == 0b11){
  //       octave = octave-1;
  //       state2 = 1;
  //     }
  //     if(Abit2 == 0b10){
  //       octave=octave+1;
  //       state2 = 1;
  //       //delayMicroseconds(1);

  //     }
  //     if(Abit2 == 0b00){
        
  //       state2 = 0;

  //     }
  //     if(Abit2 == 0b01){
        
  //       state2 = 0;

  //     }
  //     break;
  //     case 1:
  //     if(Abit2== 0b10){
  //       octave = octave -1;
  //       state2 = 0;
        
  //     }
  //     if(Abit2 ==0b00){
  //       octave  = octave +1;
  //       state2 = 0;
  //       //delayMicroseconds(1);
  //     }
  //     if(Abit2 ==0b11){
        
  //       state2 = 1;
  //     }
  //     if(Abit2 ==0b01){
  //       state2 = 1;
  //     }
  //     break;

  //   }
  //   if (octave  >= 2){
  //     octave  =2;
  //   }
  //   if (octave  <= -1 ){
  //     octave  =-1;
  //   }
    
    for (uint8_t i = 0; i < 3; i++){
      
      //Serial.println(keyArray[i]);
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);

      uint8_t val = keyArray[i];
      xSemaphoreGive(keyArrayMutex);
    
        
      for (uint8_t j = 0; j < 4; j++){ 
        
          
        if (~val & (1 << j)){
            

           step = 3-j +(i*4);
           //Serial.println(step);
             
            
        }
        

        }
    
        

      
      
      currentStepSize = stepSizes[step] ;
      
      //__atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
      //delayMicroseconds(300);
    
      

    }
    }

}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
 // Serial.println("START1");

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);
 // Serial.println("START2");

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply
 // Serial.println("START3");
  //Initialise UART
  Serial.begin(9600);
 // Serial.println("Hello Max");
  //Serial.println("START4");



  

  //thread run
  TaskHandle_t vibratoHandle = NULL;
  xTaskCreate(
  vibratoTask,		/* Function that implements the task */
  "vibrato",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &vibratoHandle );
  //Serial.println("START5");

  TaskHandle_t  displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "display",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &displayUpdateHandle );	
//  Serial.println("START6");

TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &scanKeysHandle );
  //Serial.println("START5");

    //settimer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Serial.println("END OF SETUP");
  //Semaphore Mutex
  keyArrayMutex = xSemaphoreCreateMutex();

  

  //init scheduler
  vTaskStartScheduler();


}



 

void loop() {
  

  }
  
