#include <Arduino.h>
#include <avr/interrupt.h>
#include <limits.h>

#define GREEN_LED 3
#define RED_LED 4

// IMPORTANT: change PCMSK1 (and possibly PCICR) if changing these pin numbers
#define BUTTON_PIN A0
#define CLK A1
#define DT A2

#define SCK 5
#define SDL 6

const int INPUT_MODE_DELAY = 1000;               // go into input mode after button is pressed for this duration
const uint8_t DEBOUNCE_DELAY = 50;               // how many milliseconds in same state to call it a real change
volatile unsigned long pressedSince = ULONG_MAX; // milliseconds that the button has been held pressed for

volatile unsigned long timerEndMillis = 0; // the time is up when clock reaches this value

volatile unsigned long flashGreenUntil = 0;
volatile unsigned long flashRedUntil = 0;

unsigned long showCounterUntil = 0;

const unsigned long TIMER_INTERVAL = 3600000; // 1hr, in milliseconds

bool isInputMode = false;

volatile unsigned long lastDebounceTime = 0;
void handleButtonPress(bool isPressed)
{
  // debounce the button
  if ((millis() - lastDebounceTime) < DEBOUNCE_DELAY)
    return;
  lastDebounceTime = millis();

  if (!isPressed)
  {
    pressedSince = ULONG_MAX; // reset, since button is no longer pressed
    return;
  }

  if (isPressed && isInputMode)
  {
    isInputMode = false;
    Serial.println("Exited input mode.");
    return;
  }

  pressedSince = millis(); // start tracking how long the button is held in for

  unsigned long secondsLeft = (timerEndMillis - millis()) / 1000;
  if (timerEndMillis == 0)
  {
    // timer hasn't started yet
    secondsLeft = 0;
  }
  else if (secondsLeft > TIMER_INTERVAL / 1000)
  {
    // timer has already elapsed
    secondsLeft = 0;
  }

  if (secondsLeft > 0 && flashGreenUntil <= millis())
  {
    // timer hasn't elapsed yet, flash the red LED for 5 seconds
    flashRedUntil = millis() + 5000;
  }
  else if (secondsLeft == 0)
  {
    // time is up - show the green LED
    flashGreenUntil = 3000;
    timerEndMillis = millis() + TIMER_INTERVAL; // reset the timer for a new interval
    secondsLeft = TIMER_INTERVAL / 1000;
  }
  Serial.print("Button pressed. Time remaining: ");
  Serial.println(secondsLeft);

  // clock out minutes remaining to display in binary on the LED bar graph
  byte minutesLeft = (byte)(secondsLeft / 60);
  Serial.print("Minutes remaining: ");
  Serial.println((int)minutesLeft);
  shiftOut(SDL, SCK, MSBFIRST, minutesLeft);
  showCounterUntil = millis() + 5000; // show the remaining time on the display for 5 seconds
}

void setup()
{
  // rotary encoder button pin setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // rotary encoder input setup
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);

  // set up rotary encoder inputs for Pin Change Interrupts
  cli();                // turn off interrupts while we change things
  PCICR |= 0b00000010;  // enable PCI for Port C
  PCMSK1 |= 0b00000111; // enable A0, A1, A2 pins
  sei();                // turn interrupts back on

  // shift register setup
  digitalWrite(SCK, LOW);
  pinMode(SCK, OUTPUT);
  digitalWrite(SDL, LOW);
  pinMode(SDL, OUTPUT);
  shiftOut(SDL, SCK, MSBFIRST, 0);

  // red/green pin setup
  digitalWrite(GREEN_LED, LOW);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  pinMode(RED_LED, OUTPUT);

  Serial.begin(115200);
}

volatile unsigned long lastEncoderDebounce = 0;
volatile uint8_t prevNextCode = 0;
volatile uint16_t store = 0;
volatile int8_t counter, val;

uint8_t read_rotary()
{
  // https://www.best-microcontroller-projects.com/rotary-encoder.html
  static int8_t rot_enc_table[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

  prevNextCode <<= 2;
  if (digitalRead(DT))
    prevNextCode |= 0x02;
  if (digitalRead(CLK))
    prevNextCode |= 0x01;
  prevNextCode &= 0x0f;

  if (rot_enc_table[prevNextCode])
  {
    store <<= 4;
    store |= prevNextCode;
    if ((store & 0xff) == 0x2b)
      return -1;
    if ((store & 0xff) == 0x17)
      return 1;
  }

  return 0;
}

void handleEncoder()
{

  if (val = read_rotary())
  {
    counter += val;
    Serial.print(prevNextCode & 0x0f, HEX);
    Serial.print(" ");

    if ((prevNextCode & 0x0f) == 0x0b)
    {
      Serial.print("eleven ");
      Serial.println(store, HEX);
    }
    if ((prevNextCode & 0x0f) == 0x07)
    {
      Serial.print("seven ");
      Serial.println(store, HEX);
    }
  }
}

volatile uint8_t lastCpins = PINC;
ISR(PCINT1_vect)
{
  // Port C interrupt
  uint8_t changed = (lastCpins ^ PINC) & 0b00000111; // changed if different value from before, and valid pin
  lastCpins = PINC;

  if (changed % 2 == 1)
  {
    bool isPressed = (PINC & 1) == 0;
    handleButtonPress(isPressed);
  }
  else
  {
    handleEncoder();
  }
}

const unsigned long FLASH_DELAY = 500; // in milliseconds
void tryFlash(volatile unsigned long &flashUntil, unsigned long &lastChange, uint8_t pin)
{
  const uint8_t shouldFlash = flashUntil != 0 && flashUntil >= millis();

  // check if we're done flashing
  if (!shouldFlash)
  {
    if (lastChange != 0 || flashUntil != 0 || digitalRead(pin) != LOW)
    {
      // reset for next time
      digitalWrite(pin, LOW);
      flashUntil = 0;
      lastChange = 0;
    }
    return;
  }

  // check if it's not time to change state yet
  if (lastChange + FLASH_DELAY >= millis() && lastChange != 0)
  {
    return;
  }

  lastChange = millis();

  // turn on if just starting, or we were off before
  if (lastChange == 0 || digitalRead(pin) == LOW)
  {
    digitalWrite(pin, HIGH);
    return;
  }
  // otherwise turn off
  digitalWrite(pin, LOW);
}

void loop()
{
  static unsigned long lastRedChange = 0;
  static unsigned long lastGreenChange = 0;
  if (pressedSince < ULONG_MAX)
  {
    int buttonMillis = millis() - pressedSince;
    if (buttonMillis > INPUT_MODE_DELAY && !isInputMode)
    {
      isInputMode = true;
      Serial.println("Entered input mode.");
    }
  }

  tryFlash(flashGreenUntil, lastGreenChange, GREEN_LED);
  tryFlash(flashRedUntil, lastRedChange, RED_LED);

  int counterRemaining = showCounterUntil - millis();
  if (showCounterUntil != 0 && counterRemaining < 0)
  {
    shiftOut(SDL, SCK, MSBFIRST, 0); // shift all zeros to the display
    showCounterUntil = 0;            // don't keep checking and shifting out zeros
  }
}