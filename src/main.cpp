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
volatile uint32_t currentStepSize[13];
volatile int step;
volatile bool keypressed;

// Step Size Array
double frequencies[] = {261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392, 415.3, 440, 466.16, 493.88};
const uint32_t stepSizes[] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007189, 96418756, 0};
const std::string notes[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "No Key"};

// key Array
SemaphoreHandle_t keyArrayMutex;
volatile uint8_t keyArray[7];

// Knobs
Knob Volume(8, 8, 0, 3);
Knob Octave(0, 3, 0, 2);
Knob Waveform(0, 3, 0, 1);

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

void sampleISR()
{

  static uint32_t phaseAcc[12] = {0};
  int wavetype = Waveform.value;
  int32_t Vout = 0;

  for (int i = 0; i < 12; i++)
  {
    if (wavetype == 0)
    {
      // Sawtooth
      phaseAcc[i] += currentStepSize[i];
      Vout += (phaseAcc[i] >> 24) + 128;
    }
    else if (wavetype == 1)
    {
      // Square
      phaseAcc[i] += currentStepSize[i];
      Vout += (phaseAcc[i] >> 24) > 128 ? -128 : 127;
    }
    else if (wavetype == 2)
    {
      phaseAcc[i] += currentStepSize[i];
      // Triangle
      if ((phaseAcc[i] >> 24) >= 128)
      {

        Vout += ((255 - (phaseAcc[i] >> 24)) * 2) - 128;
      }
      else
      {
        // equivalent to phaseAcc[i] >> 24 * 2
        Vout += (phaseAcc[i] >> 23) - 128;
      }
    }

    else if (wavetype == 3)
    {
      // sinusoid
      int idx;
      phaseAcc[i] += currentStepSize[i];

      if ((phaseAcc[i] >> 24) >= 128)
      {
        idx = 255 - (phaseAcc[i] >> 24);
      }
      else
      {
        idx = phaseAcc[i] >> 24;
      }

      Vout += sinetable[idx] + 128;
    }
  }

  Vout = Vout >> (8 - Volume.value);

  analogWrite(OUTR_PIN, Vout - 128);
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

void scanKeysTask(void *pvParameters)
{

  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t keys;
  int wavetype;

  // CAN BUS
  uint8_t TX_Message[8] = {0};

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    for (uint8_t i = 0; i < 5; i++)
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
      }
    }

    Volume.read_knob();
    Octave.read_knob();
    Waveform.read_knob();

    TX_Message[0] = 'p';
    TX_Message[1] = Octave.value;
    TX_Message[2] = step;

    CAN_TX(0x123, TX_Message);
  }
}

void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // CAN Bus
  uint32_t ID;
  uint8_t RX_Message[8] = {0};

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    while (CAN_CheckRXLevel())
    {
      CAN_RX(ID, RX_Message);
    }

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
  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
  CAN_Start();

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

  keyArrayMutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop()
{
}