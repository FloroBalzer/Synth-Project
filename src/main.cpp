#include <Arduino.h>
#include <U8g2lib.h>
#include <iostream>
#include <string>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

#include <knobs.h>

// Constants
const uint32_t interval = 100; // Display update interval

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
volatile uint32_t currentStepSize[12] = {0};
volatile int step;
volatile bool keypressed;

// Step Size Array
float frequencies[] = {261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392, 415.3, 440, 466.16, 493.88};
const uint32_t stepSizes[] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007189, 96418756, 0};
const std::string notes[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "No Key"};

// receiver stuff from zack
bool receiver = true;
uint8_t position;
bool position_set;

bool east;
bool west;

// CAN Bus
uint8_t RX_Message[8] = {0};

// Queues
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

// mutex and semaphores
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t CAN_TX_Semaphore;

// key Array
SemaphoreHandle_t keyArrayMutex;
volatile uint8_t keyArray[7];
volatile uint8_t key_size = 12;
volatile uint8_t numKeyPress = 0;

// Knobs
Knob Volume(8, 8, 0, 3);
Knob Octave(0, 3, 0, 2);
Knob Waveform(0, 3, 0, 1);

// joystick
volatile int32_t vstep = 0;
volatile int32_t bstep = 0;
volatile int joyY = 490;
volatile int joyX = 563;
volatile int joyXbias;
volatile int joyYbias;

// waveforms
//  This is the only wave we set up "by hand"
//  Note: this is the first half of the wave,
//        the second half is just this mirrored.
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
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

uint8_t readCols()
{
  int C0 = digitalRead(C0_PIN);
  int C1 = digitalRead(C1_PIN);
  int C2 = digitalRead(C2_PIN);
  int C3 = digitalRead(C3_PIN);
  uint8_t col_input = (C0 << 3) | (C1 << 2) | (C2 << 1) | (C3);

  return col_input;
}

void sampleISR()
{

  static uint32_t phaseAcc[12] = {0};

  // start bend
  // end bend
  // int wavetype = Waveform.value;
  int32_t Vout = 0;
  if (receiver)
  {
    for (int i = 0; i < key_size; i++)
    {
      if (currentStepSize[i] != 0)
      {

        //     int bend = abs(joyX - joyXbias);
        //     if (bend < 28)
        //     {
        //       bend = 28;
        //     }
        //     if (joyX > joyXbias)
        //     {
        //       bstep = bend * 17961;
        //     }
        //     if (joyX <= joyXbias)
        //     {
        //       bstep = bend * -17961;
        //     }
        //     if (bend < 50)
        //     {
        //       bstep = 0;
        //     }
        //   }
        //   else
        //   {
        //     bstep = 0;
        //   }
        // if (wavetype == 0)
        // {
        // Sawtooth
        phaseAcc[i] += (int)(currentStepSize[i]);
        Vout += (int)(phaseAcc[i] >> 24) - 128;
      }
    }
  }

  // }
  // else if (wavetype == 1)
  // {
  //   // Square
  //   phaseAcc[i] += (currentStepSize[i]) << Octave.value;
  //   Vout += (phaseAcc[i] >> 24) > 128 ? -128 : 127;
  // }
  // else if (wavetype == 2)
  // {
  //   phaseAcc[i] += (currentStepSize[i]) << Octave.value;
  //   // Triangle
  //   if ((phaseAcc[i] >> 24) >= 128)
  //   {

  //     Vout += ((255 - (phaseAcc[i] >> 24)) * 2) - 128;
  //   }
  //   else
  //   {
  //     // equivalent to phaseAcc[i] >> 24 * 2
  //     Vout += (phaseAcc[i] >> 23) - 128;
  //   }
  // }
  // else if (wavetype == 3)
  // {
  //   // sinusoid
  //   int idx;
  //   phaseAcc[i] += (currentStepSize[i]) << Octave.value;

  //   if ((phaseAcc[i] >> 24) >= 128)
  //   {
  //     idx = 255 - (phaseAcc[i] >> 24);
  //   }
  //   else
  //   {
  //     idx = phaseAcc[i] >> 24;
  //   }

  //   Vout += sinetable[idx] + 128;
  // }
  // }

  Vout = (int)Vout >> (8 - Volume.value);
  if (Vout > 127)
  {
    Vout = 127;
  }
  else if (Vout < -128)
  {
    Vout = 128;
  }

  analogWrite(OUTR_PIN, Vout + 128);
}
void scanKeysTask(void *pvParameters)
{

  const TickType_t xFrequency = 200 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t keys;
  int wavetype;

  int local_key;
  int local_stepsize;
  // CAN BUS
  uint8_t TX_Message[8] = {0};

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    __atomic_store_n(&step, 12, __ATOMIC_RELAXED);

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
          if (~keys & (1 << j))
          {
            __atomic_store_n(&currentStepSize[3 - j + (i * 4)], stepSizes[3 - j + (i * 4)], __ATOMIC_RELAXED);
          }
          else
          {
            __atomic_store_n(&currentStepSize[3 - j + (i * 4)], 0, __ATOMIC_RELAXED);
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
    // Serial.println(keyArray[0] & keyArray[1] & keyArray[2]);
    //   Serial.println(keyArray[i], BIN);
    // Serial.println(numKeyPress);
    // Serial.println("br");

    Volume.read_knob();
    Octave.read_knob();
    Waveform.read_knob();

    if (step == 12)
    {
      TX_Message[0] = 'R';
      TX_Message[1] = Octave.value;
    }
    else
    {
      TX_Message[0] = 'P';
      TX_Message[1] = Octave.value;
      TX_Message[2] = step;
    }

    // CAN_TX(0x123, TX_Message);
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
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
    u8g2.drawStr(2, 10, notes[step].c_str());

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
// void vibratoTask(void *pvParameters)
// {
//   const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
//   TickType_t xLastWakeTime = xTaskGetTickCount();
//   int v_count = 0;
//   bool hit_peak = false;
//   while (1)
//   {

//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     int t = 1;
//     {
//       for (int j; j < key_size; j++)
//       {
//         if (currentStepSize[j] != 0)
//         {
//           int center = abs(joyY - joyYbias);
//           // y of joystick min 900 max 138 mid 490
//           if (!hit_peak)
//             v_count++;
//           else
//             v_count--;
//           if (v_count == 128)
//             hit_peak = true;
//           if (v_count == 0)
//             hit_peak = false;
//           vstep = (sinetable[v_count] - 128) * 10e4;
//           delay()
//         }
//       }
//     }
//   }
// }
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
    receiver = false;
  }
  Octave.set_value(position + 4);
  TX_Message[0] = 'E';
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}
void decodeTask(void *pvParameters)
{
  bool external_pressed;

  int local_stepsize;
  int relative_octive;

  while (1)
  {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

    switch (RX_Message[0])
    {
    case 'R':
      external_pressed = false;
      Octave.set_value(RX_Message[1] + position);
      local_stepsize = stepSizes[12];
      __atomic_store_n(&currentStepSize, local_stepsize, __ATOMIC_RELAXED);
      Octave.set_value(RX_Message[1] + position);
      break;

    case 'P':
      external_pressed = true;
      Octave.set_value(RX_Message[1] + position);
      relative_octive = Octave.value - 4;
      if (relative_octive > 0)
      {
        local_stepsize = stepSizes[RX_Message[2]] << relative_octive;
      }
      else
      {
        local_stepsize = stepSizes[RX_Message[2]] >> abs(relative_octive);
      }
      __atomic_store_n(&currentStepSize, local_stepsize, __ATOMIC_RELAXED);
      break;

    case 'H':
      Serial.println(RX_Message[0]);
      if (!west)
      {
        if (!position_set)
        {
          position = RX_Message[2] + 1;
          position_set = true;
        }
        if (!east)
        {
          broadcastEndHandshake();
        }
        else
        {
          setOutMuxBit(HKOW_BIT, LOW);
          broadcastPosition();
        }
      }
      break;
    case 'E':
      if (position == 0)
      {
        receiver = true;
      }
      else
      {
        position = false;
      }
      Octave.set_value(position + 4);
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
  if (~keyArray[5] & 1)
  {
    west = true;
    Serial.println("Connected West");
  }
  else if (~keyArray[6] & 1)
  {
    east = true;
    Serial.println("Connected East");
  }

  if (!west)
  {
    // leftmost board
    position = 0;
    position_set = true;
    Serial.println("Leftmost board");

    if (!east)
    {
      // Only one module - end handshake
      Serial.println("Only One Module");
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
      setOutMuxBit(HKOW_BIT, LOW);
      broadcastPosition();
    }
  }
}

void setup()
{
  // put your setup code here, to run once:

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
  Serial.println("Hello World");

  // Timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // Initialise CAN
  CAN_Init(false);
  // CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  // Initialise Queues
  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);

  // mutex and semaphores
  keyArrayMutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);
  // Initialise Threads
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
  TaskHandle_t txHandle = NULL;
  xTaskCreate(
      CAN_TX_Task, /* Function that implements the task */
      "tx",        /* Text name for the task */
      256,         /* Stack size in words, not bytes */
      NULL,        /* Parameter passed into the task */
      1,           /* Task priority */
      &txHandle);
  // initialise handshake
  setOutMuxBit(HKOW_BIT, HIGH); // Enable west handshake
  setOutMuxBit(HKOE_BIT, HIGH); // Enable east handshake
  for (int i = 5; i <= 6; i++)
  {
    setRow(i);
    delayMicroseconds(2);
    keyArray[i] = readCols();
  }
  // TaskHandle_t vibratoHandle = NULL;
  // xTaskCreate(
  //     vibratoTask,     /* Function that implements the task */
  //     "vibrato",       /* Text name for the task */
  //     64,              /* Stack size in words, not bytes */
  //     NULL,            /* Parameter passed into the task */
  //     1,               /* Task priority */
  //     &vibratoHandle); /* Pointer to store the task handle */

  delayMicroseconds(1000000); // wait 1 sec for other boards to start
  checkBoards();
  CAN_Start();
  delayMicroseconds(1000000);
  initialHandshake();
  joyXbias = analogRead(A1);
  joyYbias = analogRead(A0);

  vTaskStartScheduler();
}

void loop()
{
}