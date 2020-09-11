#include <Arduino.h>
#include <avr/interrupt.h>
#include <limits.h>
#include <EEPROM.h>

#define GREEN_LED 3
#define RED_LED 4

// IMPORTANT: change PCMSK1 (and possibly PCICR) if changing these pin numbers
#define BUTTON_PIN A0 // either integral with rotary encoder, or standalone
#define CLK A1        // rotary encoder CLK or 'A' pin
#define DT A2         // rotary encoder DT or 'B' pin

#define SCK 5 // shift register serial clock line
#define SDL 6 // shift register serial data line

const int INPUT_MODE_DELAY = 1000;               // go into input mode after button is pressed for this duration
const uint8_t DEBOUNCE_DELAY = 50;               // how many milliseconds in same state to call it a real change
volatile unsigned long pressedSince = ULONG_MAX; // milliseconds that the button has been held pressed for

volatile unsigned long timerEndMillis = 0; // the time is up when clock reaches this value

volatile unsigned long flashGreenUntil = 0;
volatile unsigned long flashRedUntil = 0;

unsigned long showCounterUntil = 0;

unsigned long timerInterval = 3600000; // 1hr, in milliseconds

volatile uint8_t inputMode = 0;
volatile unsigned long lastDebounceTime = 0;

void handleButtonPress(bool isPressed);

void showValue(byte value, unsigned long duration = ULONG_MAX)
{
  shiftOut(SDL, SCK, MSBFIRST, value);
  showCounterUntil = millis() + duration;
}

void incrementMode()
{
  inputMode++;
  Serial.print("inputMode=");
  Serial.println(inputMode);
  byte intervalMinutes;
  switch (inputMode)
  {
  case 1:
    // adjust the timer interval
    flashGreenUntil = flashRedUntil = showCounterUntil = 0;
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    intervalMinutes = (byte)(timerInterval / (1000UL * 60UL)); // milliseconds to minutes
    Serial.print("intervalMinutes: ");
    Serial.println(intervalMinutes);
    showValue(intervalMinutes);
    break;
  case 2:
    // just left the timer interval adjustment, so write it to EEPROM if it changed
    intervalMinutes = (byte)(timerInterval / (60000UL));
    EEPROM.update(0, intervalMinutes);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    break;
  default:
    // bad case or wrapped, so reset and break
    inputMode = 0;
    Serial.println("Exited input mode.");
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    shiftOut(SDL, SCK, MSBFIRST, 0);
    lastDebounceTime = 0;
    handleButtonPress(true); // simulate additional button press to show time left and blinkenlights
    break;
    return;
  }
}

void handleButtonPress(bool isPressed)
{
  // debounce the button
  if ((millis() - lastDebounceTime) < DEBOUNCE_DELAY)
    return;
  lastDebounceTime = millis();

  if (!isPressed)
  {
    // ISR triggered on the button being released
    pressedSince = ULONG_MAX; // reset, since button is no longer pressed
    return;
  }

  if (isPressed && inputMode > 0)
  {
    // button was pressed when already in an input mode
    incrementMode();
    return;
  }

  pressedSince = millis(); // start tracking how long the button is held in for

  unsigned long secondsLeft = (timerEndMillis - millis()) / 1000;
  if (timerEndMillis == 0) // timer hasn't started yet
  {
    secondsLeft = 0;
  }
  else if (secondsLeft > timerInterval / 1000) // timer has already elapsed
  {
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
    flashGreenUntil = millis() + 15000;
    timerEndMillis = millis() + timerInterval; // reset the timer for a new interval
    secondsLeft = timerInterval / 1000;
  }
  Serial.print("Button pressed. Time remaining: ");
  Serial.println(secondsLeft);

  // clock out minutes remaining to display in binary on the LED bar graph
  byte minutesLeft = (byte)(secondsLeft / 60);
  Serial.print("Minutes remaining: ");
  Serial.println((int)minutesLeft);
  showValue(minutesLeft, 5000);
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

  // get timer interval from EEPROM
  timerInterval = EEPROM.read(0) * 60000UL;

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

void doEncoderStep(bool isCw)
{
  int newIntervalMinutes;
  switch (inputMode)
  {
  case 0:
    return;
  case 1:
    // change the timer interval
    newIntervalMinutes = (timerInterval / 60000) + (isCw ? 1 : -1);
    if (newIntervalMinutes > 256)
      newIntervalMinutes = 256;
    if (newIntervalMinutes < 0)
      newIntervalMinutes = 0;
    timerInterval = newIntervalMinutes * 60000;
    showValue(newIntervalMinutes);
    break;

  case 2:
  default:
    return;
  }
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
      doEncoderStep(false);
    }
    if ((prevNextCode & 0x0f) == 0x07)
    {
      Serial.print("seven ");
      Serial.println(store, HEX);
      doEncoderStep(true);
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
  if (pressedSince < ULONG_MAX) // button is currently held down
  {
    int buttonMillis = millis() - pressedSince;
    if (buttonMillis > INPUT_MODE_DELAY && inputMode == 0)
    {
      incrementMode();
      return;
    }
  }

  if (inputMode > 0)
  {
    return;
  }

  tryFlash(flashGreenUntil, lastGreenChange, GREEN_LED);
  tryFlash(flashRedUntil, lastRedChange, RED_LED);

  int counterRemaining = showCounterUntil - millis();
  if (showCounterUntil != 0 && counterRemaining < 0)
  {
    showValue(0);         // clear the display
    showCounterUntil = 0; // don't keep checking and shifting out zeros
  }
}