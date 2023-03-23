#include <Arduino.h>
#include <U8g2lib.h>
#include <iostream>
#include <string>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include <algorithm>

#include "knobs.h"

////////control disabling threads == comment out the thread you want to test ==> normal mode == all the thread definitions commented out

// #define DISABLE_THREADS_Scankeys
#define DISABLE_THREADS_decodeTask
#define DISABLE_THREADS_displayUpdateHandle
#define DISABLE_THREADS_CAN_TX_Task

///////// directives to deactivate the statements that attach ISRs.

#define Disable_CAN_RegisterRX_TX_ISR
#define Disable_attachInterrupt_sampleISR
#define Disable_msgque

//////////////////////// if test define it will work
#define TEST_SCANKEYS
#define 
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

// Step Size and notes
const uint32_t stepSizes[13] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007189, 96418756, 0};
const std::string notes[13] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "No key pressed"};
const uint8_t key_size = 36;
volatile uint8_t keyArray[7];
volatile uint32_t currentStepSize[key_size] = {0};
volatile uint8_t pressed[key_size] = {0};

// Multiple Modules
bool receiver = true;
uint8_t position = 8;
bool position_set;
volatile bool east;
volatile bool west;

// CAN Bus
uint8_t RX_Message[8] = {0};

// Queues
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

// mutex and semaphores
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t CAN_TX_Semaphore;

// Knobs
Knob Volume(8, 8, 0, 0);
Knob Octave(4, 7, 1, 1);
Knob Waveform(0, 3, 0, 2);

// joystick
volatile int32_t bstep = 0;
volatile int joyX = 563;
volatile int joyXbias;

// waveforms
const unsigned char sinetable[128] = {
    0,
    0,
    0,
    0,
    1,
    1,
    1,
    2,
    2,
    3,
    4,
    5,
    5,
    6,
    7,
    9,
    10,
    11,
    12,
    14,
    15,
    17,
    18,
    20,
    21,
    23,
    25,
    27,
    29,
    31,
    33,
    35,
    37,
    40,
    42,
    44,
    47,
    49,
    52,
    54,
    57,
    59,
    62,
    65,
    67,
    70,
    73,
    76,
    79,
    82,
    85,
    88,
    90,
    93,
    97,
    100,
    103,
    106,
    109,
    112,
    115,
    118,
    121,
    124,
    128,
    131,
    134,
    137,
    140,
    143,
    146,
    149,
    152,
    155,
    158,
    162,
    165,
    167,
    170,
    173,
    176,
    179,
    182,
    185,
    188,
    190,
    193,
    196,
    198,
    201,
    203,
    206,
    208,
    211,
    213,
    215,
    218,
    220,
    222,
    224,
    226,
    228,
    230,
    232,
    234,
    235,
    237,
    238,
    240,
    241,
    243,
    244,
    245,
    246,
    248,
    249,
    250,
    250,
    251,
    252,
    253,
    253,
    254,
    254,
    254,
    255,
    255,
    255,
};

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

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

  static uint32_t phaseAcc[key_size] = {0};

  int wavetype = Waveform.value;
  int32_t Vout = 0;

  for (int i = 0; i < key_size; i++)
  {
    if (currentStepSize[i] != 0)
    {
      int bend = abs(joyX - joyXbias);
      if (bend < 28)
      {
        bend = 28;
      }
      if (joyX > joyXbias)
      {
        bstep = bend * 17961;
      }
      if (joyX <= joyXbias)
      {
        bstep = bend * -17961;
      }
      if (bend < 50)
      {
        bstep = 0;
      }

      phaseAcc[i] += (int)(currentStepSize[i]) + bstep;

      if (wavetype == 0)
      {
        // Sawtooth - linear increase
        Vout += (int)(phaseAcc[i] >> 24) - 128;
      }
      else if (wavetype == 1)
      {
        // Square - set high or low
        Vout += (phaseAcc[i] >> 24) > 128 ? -128 : 127;
      }
      else if (wavetype == 2)
      {
        // Triangle - <128 linear increase, >128 linear decrease
        if ((phaseAcc[i] >> 24) >= 128)
        {
          Vout += 2 * (((255 - (phaseAcc[i] >> 24)) * 2) - 127);
        }
        else
        {
          Vout += 2 * ((phaseAcc[i] >> 23) - 128);
        }
      }
      else if (wavetype == 3)
      {
        // sinusoid - lookup table
        int idx;

        if ((phaseAcc[i] >> 24) >= 128)
        {
          idx = 255 - (phaseAcc[i] >> 24);
        }
        else
        {
          idx = phaseAcc[i] >> 24;
        }

        Vout += 2 * (sinetable[idx] - 128);
      }
    }
  }

  Vout = (int)Vout >> (8 - Volume.value);

  Vout = std::max(std::min((int)Vout, 127), -128);

  analogWrite(OUTR_PIN, Vout + 128);
}

void CAN_RX_ISR(void)
{
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR(void)
{
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
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

void scanKeysTask(void *pvParameters)
{

  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t keys;

  int vol = 0;
  int oct = 0;
  int wavetype = 0;

  int local_key;
  int local_stepsize;

  int relative_octive;

  uint8_t TX_Message[8] = {0};

  while (1)
  {
#ifndef TEST_SCANKEYS
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
#endif
    relative_octive = Octave.value - 4;

    for (uint8_t i = 0; i < 7; i++)
    {

      setRow(i);
      delayMicroseconds(3);
      keys = readCols();

      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      keyArray[i] = keys;
      xSemaphoreGive(keyArrayMutex);

      if (i < 3)
      {
        for (uint8_t j = 0; j < 4; j++)
        {

          local_key = 3 - j + (i * 4);

          if (~keys & (1 << j))
          {
            if (receiver)
            {
              if (relative_octive > 0)
              {
                local_stepsize = stepSizes[local_key] << relative_octive;
              }
              else
              {
                local_stepsize = stepSizes[local_key] >> abs(relative_octive);
              }
              __atomic_store_n(&currentStepSize[local_key], local_stepsize, __ATOMIC_RELAXED);
            }
            if (!receiver && (!pressed[local_key]))
            {

              TX_Message[0] = 'P';
              TX_Message[1] = Octave.value;
              TX_Message[2] = local_key;
              TX_Message[3] = position;

              xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
            }
            pressed[local_key] = 1;
          }
          else
          {

            if (receiver)
            {
              __atomic_store_n(&currentStepSize[local_key], 0, __ATOMIC_RELAXED);
            }
            if (!receiver && pressed[local_key])
            {

              TX_Message[0] = 'R';
              TX_Message[1] = Octave.value;
              TX_Message[2] = local_key;
              TX_Message[3] = position;

              xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
            }
            pressed[local_key] = 0;
          }
        }
      }
      else if (i == 5)
      {
        if (~keys & 1)
        {
          west = true;
        }
        else
        {
          west = false;
        }
      }
      else if (i == 6)
      {
        if (~keys & 1)
        {
          east = true;
        }
        else
        {
          east = false;
        }
      }
    }

    if (receiver)
    {
      Volume.read_knob();
      Octave.read_knob();
      Waveform.read_knob();
      joyX = analogRead(A1);
      if (vol != Volume.value || oct != Octave.value || wavetype != Waveform.value)
      {

        TX_Message[0] = 'K';
        TX_Message[1] = Volume.value;
        TX_Message[2] = Octave.value;
        TX_Message[3] = Waveform.value;

        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      }

      vol = Volume.value;
      oct = Octave.value;
      wavetype = Waveform.value;
    }
#ifdef TEST_SCANKEYS
    break;
#endif
  }
}

void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // CAN Bus
  uint32_t ID;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Update display
    u8g2.clearBuffer();                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    int count = 0;
    for (int i = 0; i < key_size; i++)
    {
      if (pressed[i] == 1)
      {
        Serial.println(count);
        u8g2.drawStr(2 + 15 * count, 10, notes[i % 12].c_str());
        count++;
      }
    }

    u8g2.setCursor(2, 20);
    u8g2.print(keyArray[2], HEX);
    u8g2.print(" ");
    u8g2.print(keyArray[1], HEX);
    u8g2.print(" ");
    u8g2.print(keyArray[0], HEX);

    receiver ? u8g2.drawStr(66, 20, "Receiver") : u8g2.drawStr(66, 20, "Sender");
    u8g2.setCursor(120, 20);
    u8g2.print(position, HEX);

    u8g2.drawStr(2, 30, "V: ");
    u8g2.setCursor(15, 30);
    u8g2.print(Volume.value);
    u8g2.drawStr(25, 30, "O: ");
    u8g2.setCursor(40, 30);
    u8g2.print(Octave.value);
    u8g2.drawStr(50, 30, "W: ");
    u8g2.setCursor(65, 30);
    u8g2.print(Waveform.value);

    u8g2.setCursor(75, 30);
    u8g2.print((char)RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);

    u8g2.sendBuffer(); // transfer internal memory to the display
    xSemaphoreGive(keyArrayMutex);

    // Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void broadcastPosition()
{
  uint8_t TX_Message[8] = {0};

  TX_Message[0] = 'H';
  TX_Message[1] = 0;
  TX_Message[2] = position;

  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

void broadcastEndHandshake()
{
  uint8_t TX_Message[8] = {0};

  if (position != 0)
  {
    __atomic_store_n(&receiver, false, __ATOMIC_RELAXED);
  }
  Octave.set_value(position + 4);
  TX_Message[0] = 'E';
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

void decodeTask(void *pvParameters)
{

  int local_stepsize;
  int relative_octive;
  int local_key;

  int handshake_count = 0;

  while (1)
  {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

    switch (RX_Message[0])
    {
    case 'R':
      if (receiver)
      {
        local_stepsize = stepSizes[12];
        local_key = 12 * RX_Message[3] + RX_Message[2];
        pressed[local_key] = 0;
        __atomic_store_n(&currentStepSize[local_key], local_stepsize, __ATOMIC_RELAXED);
      }
      break;

    case 'P':

      if (receiver)
      {
        relative_octive = RX_Message[1] - 4;
        local_key = 12 * RX_Message[3] + RX_Message[2];
        pressed[local_key] = 1;
        Serial.println(local_key);
        if (relative_octive > 0)
        {
          local_stepsize = stepSizes[RX_Message[2]] << relative_octive;
        }
        else
        {
          local_stepsize = stepSizes[RX_Message[2]] >> abs(relative_octive);
        }
        __atomic_store_n(&currentStepSize[local_key], local_stepsize, __ATOMIC_RELAXED);
      }
      break;

    case 'K':
      Volume.set_value(RX_Message[1]);
      Octave.set_value(RX_Message[2] + position);
      Waveform.set_value(RX_Message[3]);
      break;

    case 'H':
      Serial.println("Received H");
      if (west)
      {
        handshake_count++;
      }
      if (!west)
      {
        if (!position_set)
        {
          position = RX_Message[2] + 1;
          position_set = true;
          if (!east)
          {
            Serial.println("End");
            broadcastEndHandshake();
          }
          else
          {
            setOutMuxBit(HKOE_BIT, LOW);
            delayMicroseconds(1000000);
            broadcastPosition();
          }
        }
      }
      break;
    case 'E':
      Serial.println("Received E");
      if (position == 0)
      {
        __atomic_store_n(&receiver, true, __ATOMIC_RELAXED);
      }
      else
      {
        __atomic_store_n(&receiver, false, __ATOMIC_RELAXED);
      }
      Octave.set_value(position + 4);
      break;
    }
  }
}

void CAN_TX_Task(void *pvParameters)
{
  uint8_t msgOut[8];
  while (1)
  {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
}

void checkBoards()
{
  xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
  if (~keyArray[5] & 1)
  {
    west = true;
  }
  else if (~keyArray[6] & 1)
  {
    east = true;
  }
  xSemaphoreGive(keyArrayMutex);

  if (!west)
  {
    // leftmost board
    position = 0;
    position_set = true;

    if (!east)
    {
      // Only one module - end handshake
      CAN_Init(true);
    }
  }
}

void initialHandshake()
{
  if (position_set)
  {
    if (!east)
    {
      Octave.set_value(position + 4);
    }
    else
    {
      setOutMuxBit(HKOE_BIT, LOW);
      delayMicroseconds(1000000);
      broadcastPosition();
    }
  }
}

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
#ifndef Disable_attachInterrupt_sampleISR
  sampleTimer->attachInterrupt(sampleISR);
#endif
  sampleTimer->resume();

  // Initialise CAN
  CAN_Init(false);
  // CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
#ifndef Disable_CAN_RegisterRX_TX_ISR
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
#endif
// Initialise Queues
#ifndef Disable_msgque
  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);
#else
  msgInQ = xQueueCreate(384, 8);
  msgOutQ = xQueueCreate(384, 8);
#endif

  // mutex and semaphores
  keyArrayMutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);

  // Initialise Threads
  TaskHandle_t scanKeysHandle = NULL;
#ifndef DISABLE_THREADS_Scankeys
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      256,              /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      2,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */
#endif

  TaskHandle_t displayUpdateHandle = NULL;
#ifndef DISABLE_THREADS_displayUpdateHandle
  xTaskCreate(
      displayUpdateTask,     /* Function that implements the task */
      "displayUpdate",       /* Text name for the task */
      256,                   /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      1,                     /* Task priority */
      &displayUpdateHandle); /* Pointer to store the task handle */
#endif

  TaskHandle_t decodeHandle = NULL;
#ifndef DISABLE_THREADS_decodeTask
  xTaskCreate(
      decodeTask, /* Function that implements the task */
      "decode",   /* Text name for the task */
      256,        /* Stack size in words, not bytes */
      NULL,       /* Parameter passed into the task */
      1,          /* Task priority */
      &decodeHandle);
#endif
  TaskHandle_t txHandle = NULL;
#ifndef DISABLE_THREADS_CAN_TX_Task
  xTaskCreate(
      CAN_TX_Task, /* Function that implements the task */
      "tx",        /* Text name for the task */
      256,         /* Stack size in words, not bytes */
      NULL,        /* Parameter passed into the task */
      1,           /* Task priority */
      &txHandle);
#endif

  joyXbias = analogRead(A1);

  // initialise handshake
  setOutMuxBit(HKOW_BIT, HIGH); // Enable west handshake
  setOutMuxBit(HKOE_BIT, HIGH); // Enable east handshake

  delayMicroseconds(2000000); // wait 1 sec for other boards to start

  for (int i = 5; i <= 6; i++)
  {
    setRow(i);
    delayMicroseconds(2);
    keyArray[i] = readCols();
  }

  /// Testing each thread ///////////////
#ifdef TEST_SCANKEYS
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    scanKeysTask(nullptr);
  }
  Serial.println(micros() - startTime);
  while (1)
    ;
#endif

  checkBoards();
  CAN_Start();
  delayMicroseconds(1000000);
  initialHandshake();
  vTaskStartScheduler();
  

}

void loop()
{
}