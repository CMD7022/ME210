#define pwmPin 9 // Pin connected to the PWM output

unsigned long previousMillis = 0;
const unsigned long interval = 15000; // Interval for each state (10 seconds)
bool pwmState = false; // Initial state of the PWM output

void setup() {
  pinMode(pwmPin, OUTPUT); // Set the PWM pin as an output
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time
  
  if (currentMillis - previousMillis >= interval) {
    // If 10 seconds have elapsed, toggle the state
    previousMillis = currentMillis;
    pwmState = !pwmState; // Toggle PWM state
    if (pwmState) {
      analogWrite(pwmPin, 255); // 80% of 255
    } else {
      analogWrite(pwmPin, 0); // Turn off the PWM signal
    }
  }
}