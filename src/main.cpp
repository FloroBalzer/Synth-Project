#include <Arduino.h>
#include <U8g2lib.h>
#include <iostream>
#include <string>

// Constants
const uint32_t interval = 100; // Display update interval
const uint32_t stepSizes[] = {51076922, 54112683, 57330004, 60740598, 64352275, 68178701, 72231588, 76528508, 81077269, 85899345, 91006452, 96418111};

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

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// other
volatile uint32_t currentStepSize;

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
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
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

void loop()
{
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  while (millis() < next)
    ; // Wait for next interval

  next += interval;

  // Update display
  u8g2.clearBuffer();                  // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
  u8g2.drawStr(2, 10, "Hello World!"); // write something to the internal memory
  u8g2.setCursor(2, 20);

  double frequencies[] = {261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392, 415.3, 440, 466.16, 493.88};

  uint8_t keyArray[7];
  std::string note;

  for (int i = 0; i <= 2; i++)
  {
    setRow(i);
    delayMicroseconds(3);
    uint8_t keys = readCols();
    keyArray[i] = keys;
    if (keyArray[i] == 0b0111 && i == 0)
    {
      note = "C";
      currentStepSize = stepSizes[0];
    }
    else if (keyArray[i] == 0b1011 && i == 0)
    {
      note = "C#";
      currentStepSize = stepSizes[1];
    }

    else if (keyArray[i] == 0b1101 && i == 0)
    {
      note = "D";
      currentStepSize = stepSizes[2];
    }

    else if (keyArray[i] == 0b1110 && i == 0)
    {
      note = "D#";
      currentStepSize = stepSizes[3];
    }

    else if (keyArray[i] == 0b0111 && i == 1)
    {
      note = "E";
      currentStepSize = stepSizes[4];
    }

    else if (keyArray[i] == 0b1011 && i == 1)
    {
      note = "F";
      currentStepSize = stepSizes[5];
    }

    else if (keyArray[i] == 0b1101 && i == 1)
    {
      note = "F#";
      currentStepSize = stepSizes[6];
    }

    else if (keyArray[i] == 0b1110 && i == 1)
    {
      note = "G";
      currentStepSize = stepSizes[7];
    }

    else if (keyArray[i] == 0b0111 && i == 2)
    {
      note = "G#";
      currentStepSize = stepSizes[8];
    }

    else if (keyArray[i] == 0b1011 && i == 2)
    {
      note = "A";
      currentStepSize = stepSizes[9];
    }

    else if (keyArray[i] == 0b1101 && i == 2)
    {
      note = "A#";
      currentStepSize = stepSizes[10];
    }

    else if (keyArray[i] == 0b1110 && i == 2)
    {
      note = "B";
      currentStepSize = stepSizes[11];
    }

    std::cout << keyArray[i] << std::endl;
  }

  std::cout << currentStepSize << std::endl;
  u8g2.setCursor(2, 20);
  u8g2.print(keyArray[0], HEX);
  u8g2.print(" ");
  u8g2.print(keyArray[1], HEX);
  u8g2.print(" ");
  u8g2.print(keyArray[2], HEX);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(2, 30, note.c_str());
  u8g2.sendBuffer(); // transfer internal memory to the display

  // Toggle LED
  digitalToggle(LED_BUILTIN);
}