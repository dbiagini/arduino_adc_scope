#define ANALOG_IN A3
#define ANALOG_OUT 3
#define BUTTON_PIN 2
int static_variable = 500;

// Buffer settings
#define BUFFER_SIZE 500 // Adjust size as needed
volatile int sampleBuffer[BUFFER_SIZE];
volatile uint8_t bufferHead = 0; // Write index
volatile uint8_t bufferTail = 0; // Read index

bool sampling = false;
volatile unsigned long lastDebounceTime = 0; // To store the last time the ISR was triggered
const unsigned long debounceDelay = 50; // 50 ms debounce time

void setup() {
  cli(); // Disable global interrupts during setup
  
  // Initialize digital pin LED_BUILTIN as an output
  pinMode(PD0, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING); // Adjust trigger type as needed
  
  // // Setup Timer1 for PWM signal generation on ANALOG_OUT
  // TCCR1A = 0;
  // TCCR1B = 0;
  // TCNT1 = 0;
  // OCR1A = 1249; // Adjusted value for 100 Hz frequency
  // TCCR1B |= (1 << WGM12); // CTC mode
  // TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler = 64
  // TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt

  // Setup Timer1 for PWM signal generation on ANALOG_OUT
TCCR1A = 0; // Clear Timer1 Control Register A
TCCR1B = 0; // Clear Timer1 Control Register B
TCNT1 = 0;  // Clear the timer counter

// Set OCR1A for 10 Hz frequency
OCR1A = 6249; // Adjusted value for 10 Hz frequency

// Set CTC mode and prescaler of 256
TCCR1B |= (1 << WGM12); // CTC mode
TCCR1B |= (1 << CS12); // Prescaler = 256

// Enable Timer1 compare interrupt
TIMSK1 |= (1 << OCIE1A);

  // Setup Timer2 for 500 Hz sampling
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 124; // Adjusted value for 500 Hz sampling frequency
  TCCR2A |= (1 << WGM21); // CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21); // Prescaler = 256
  TIMSK2 |= (1 << OCIE2A); // Enable Timer2 compare interrupt
  
  // Set output pin for the signal
  pinMode(ANALOG_OUT, OUTPUT);

  sei(); // Enable global interrupts

  Serial.begin(230400);
  digitalWrite(PD0, LOW); // Turn the LED off
  Serial.println("Program started. Waiting for button press...");
}

void loop() {
  // Check if there are new samples in the buffer
  while (bufferHead != bufferTail) {
    // Temporarily disable interrupts while accessing shared variables
    cli();
    int val = sampleBuffer[bufferTail];
    bufferTail = (bufferTail + 1) % BUFFER_SIZE; // Update tail index
    sei(); // Re-enable interrupts

    // Print the sampled value
    Serial.print("Variable_1:");
    Serial.println(val);
  }
}

// ISR for Timer1: Used for PWM signal generation on ANALOG_OUT
ISR(TIMER1_COMPA_vect) {
  static bool state = LOW;
  state = !state;
  digitalWrite(ANALOG_OUT, state);
}

// ISR for Timer2: Used for analog sampling at controlled intervals
ISR(TIMER2_COMPA_vect) {
  if (sampling) {
    int sample = analogRead(ANALOG_IN);

    // Add the sample to the buffer if there is space
    uint8_t nextHead = (bufferHead + 1) % BUFFER_SIZE;
    if (nextHead != bufferTail) { // Check if the buffer is not full
      sampleBuffer[bufferHead] = sample;
      bufferHead = nextHead; // Move the head to the next position
    } else {
      // Buffer is full, handle overflow if necessary (e.g., discard sample)
      sampling = false;
    }
  }
}

// ISR for button press with debounce logic
void buttonISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    if (sampling) {
      sampling = false;
      digitalWrite(PD0, LOW); // Turn the LED off
    } else {
      sampling = true;
      digitalWrite(PD0, HIGH); // Turn the LED on
    }
    lastDebounceTime = currentTime; // Update debounce time
  }
}