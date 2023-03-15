#include <Arduino.h>
#include <U8g2lib.h>
#include <iostream>
#include <string>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

#include "knobs.h"

// Constants

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// other
volatile uint32_t currentStepSize[13];
volatile int step;

// Step Size Array
const uint32_t stepSizes[13] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007189, 96418756, 0};
const std::string notes[13] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "No key pressed"};

/// key pressed
volatile bool keypressed;
// Key Array
SemaphoreHandle_t keyArrayMutex;
volatile uint8_t keyArray[7];

// Knobs
Knob Volume(8, 8, 0, 3);
Knob Octave(0, 3, 0, 2);

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// // queue
// QueueHandle_t msgInQ;
// QueueHandle_t msgOutQ;

// // CAN communication variables
// uint8_t TX_Message[8] = {0};
// uint8_t RX_Message[8] = {0};

// global handle for a FreeRTOS mutex
SemaphoreHandle_t RX_Message_Mutex;
SemaphoreHandle_t CAN_TX_Semaphore;

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

void sampleISR()
{
  int32_t Vout = 0;
  int n = 12;
  static int32_t phaseAcc[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  if (keypressed)
  {
    for (int i = 0; i < n; i++)
    {
      if (currentStepSize[i] != 0)
      {
        phaseAcc[i] += currentStepSize[i];
        Vout += (phaseAcc[i] >> 24 + 128);
      }
    }
    

    analogWrite(OUTR_PIN, Vout - 128);
  }
}
void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, LOW);
  unsigned char RA2 = rowIdx & 0b00000100; // RA2
  unsigned char RA1 = rowIdx & 0b00000010; // RA1
  unsigned char RA0 = rowIdx & 0b00000001; // RA2
  digitalWrite(RA2_PIN, RA2);
  digitalWrite(RA1_PIN, RA1);
  digitalWrite(RA0_PIN, RA0);
  digitalWrite(REN_PIN, HIGH);
}

uint8_t readCols()
{
  int a = digitalRead(C0_PIN);
  int b = digitalRead(C1_PIN);
  int c = digitalRead(C2_PIN);
  int d = digitalRead(C3_PIN);
  uint8_t colout = (a << 3) | (b << 2) | (c << 1) | (d);
  return colout;
}
// void generateMSG(volatile uint8_t *)
// {
// }

// void CAN_RX_ISR(void)
// {
//   uint8_t RX_Message_ISR[8];
//   uint32_t ID;
//   CAN_RX(ID, RX_Message_ISR);
//   xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
// }

// void CAN_TX_ISR(void)
// {
//   xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
// }

void scanKeysTask(void *pvParameters)
{

  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t keys;

  // CAN BUS
  // uint8_t TX_Message[8] = {0};

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    __atomic_store_n(&step, 12, __ATOMIC_RELAXED);

    for (uint8_t i = 0; i < 3; i++)
    {
      setRow(i);
      delayMicroseconds(3);
      keys = readCols();

      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      keyArray[i] = keys;
      xSemaphoreGive(keyArrayMutex);

      for (uint8_t j = 0; j < 4; j++)
      {
        if (!((keys >> j) & 1))
        {
          __atomic_store_n(&currentStepSize[3 - j + 4 * i], stepSizes[3 - j + 4 * i], __ATOMIC_RELAXED);
        }
        else
        {
          __atomic_store_n(&currentStepSize[3 - j + 4 * i], 0, __ATOMIC_RELAXED);
        }
      }
    }
    int key_loop = 0xf;
    for (int i = 0; i < 8; i++)
    {
      key_loop = keyArray[i] & key_loop;
    }
    if (key_loop == 0xf)
      keypressed = false;
    else
      keypressed = true;

    /// for each key
  }
  Volume.read_knob();
  Octave.read_knob();

  // TX_Message[0] = 'p';
  // TX_Message[1] = Octave.value;
  // TX_Message[2] = step;

  // CAN_TX(0x123, TX_Message);
}

void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // // CAN Bus
  // uint32_t ID;
  // uint8_t RX_Message[8] = {0};

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // while (CAN_CheckRXLevel())
    // {
    //   CAN_RX(ID, RX_Message);
    // }

    // Update display
    u8g2.clearBuffer();                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.drawStr(2, 10, notes[step].c_str());

    u8g2.setCursor(2, 20);
    u8g2.print(keyArray[2], HEX);
    u8g2.print(" ");
    u8g2.print(keyArray[1], HEX);
    u8g2.print(" ");
    u8g2.print(keyArray[0], HEX);

    u8g2.drawStr(2, 30, "V: ");
    u8g2.setCursor(15, 30);
    u8g2.print(Volume.value);
    u8g2.drawStr(25, 30, "O: ");
    u8g2.setCursor(40, 30);
    u8g2.print(Octave.value);

    // u8g2.setCursor(66, 30);
    // u8g2.print((char)RX_Message[0]);
    // u8g2.print(RX_Message[1]);
    // u8g2.print(RX_Message[2]);

    u8g2.sendBuffer(); // transfer internal memory to the display
    xSemaphoreGive(keyArrayMutex);

    // Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

// void decodeTask(void *pvParameters)
// {
//   while (1)
//   {
//     xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
//   }
// }
void setup()
{

  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);

  // settimer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // // Initialise CAN
  // CAN_Init(true);
  // setCANFilter(0x123, 0x7ff);
  // CAN_RegisterRX_ISR(CAN_RX_ISR);
  // CAN_RegisterTX_ISR(CAN_TX_ISR);
  // CAN_Start();
  // CAN_Start();

  // // Initialise queue handler
  // msgInQ = xQueueCreate(36, 8);
  // msgOutQ = xQueueCreate(36, 8);

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      2,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
      displayUpdateTask,     /* Function that implements the task */
      "displayUpdate",       /* Text name for the task */
      256,                   /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      1,                     /* Task priority */
      &displayUpdateHandle); /* Pointer to store the task handle */

  // Initialise Decode loop
  // TaskHandle_t decodeHandle = NULL;
  // xTaskCreate(
  //     decodeTask,   /* Function that implements the task */
  //     "decode",     /* Text name for the task */
  //     256,          /* Stack size in words, not bytes */
  //     NULL,         /* Parameter passed into the task */
  //     1,            /* Task priority */
  //     &decodeHandle /* Pointer to store the task handle */
  // );

  keyArrayMutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop()
{
}