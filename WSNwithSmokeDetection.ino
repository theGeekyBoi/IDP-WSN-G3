/**
 * Solar Power Management System with Temperature Monitoring and Smoke Detection
 * 
 * This Arduino firmware manages a solar power system with voltage regulation,
 * temperature monitoring, smoke detection, Bluetooth communication, and power-saving features.
 * 
 * Hardware Configuration:
 * - Thermistor on analog pin A0
 * - Input voltage sensor on analog pin A1
 * - Solar panel voltage on analog pin A2
 * - Photoresistor-based smoke detector on analog pin A5
 * - Bluetooth control pin on digital pin 8
 * - PWM output on pin 9 (controls DC-DC converter)
 * - Software serial for Bluetooth on pins 2 (RX) and 3 (TX)
 */

#include <SoftwareSerial.h>

// Initialize software serial for Bluetooth communication
SoftwareSerial mySerial(2, 3); // RX, TX

//------------------------------------------------------------------------------
// Pin Configuration
//------------------------------------------------------------------------------
const int thermistorPin = A0;    // Analog input for thermistor
const int inputVoltagePin = A1;  // Analog input for system input voltage
const int solarPin = A2;         // Analog input for solar panel voltage
const int photoresPin = A5;      // Analog input for photoresistor smoke detector
const int bluetoothPin = 8;      // Digital pin to control Bluetooth module

//------------------------------------------------------------------------------
// Thermistor Configuration
//------------------------------------------------------------------------------
const float seriesResistor = 15000.0;      // Fixed resistor value in voltage divider (15k ohms)
const float nominalResistance = 11200.0;   // Thermistor resistance at reference temperature (25°C)
const float nominalTemperature = 25.0;     // Reference temperature in Celsius
const float betaCoefficient = 3950.0;      // Beta value of thermistor (material constant)
const int adcMax = 1023;                   // Maximum ADC value (10-bit resolution)
double steinhart;                          // Current temperature value in Celsius
double prevTemp = 0.0;                     // Previous temperature reading for change detection

//------------------------------------------------------------------------------
// Smoke Detector Configuration
//------------------------------------------------------------------------------
const int smokeThreshold = 60;             // Threshold for smoke detection (lower value indicates smoke)
bool smoke = false;                        // Current smoke detection state
bool wasSmoke = false;                     // Previous smoke detection state (for edge detection)

//------------------------------------------------------------------------------
// Power Saving Mode Configuration
//------------------------------------------------------------------------------
bool powerSavingMode = false;              // Current power saving mode state
bool prevPSM = !powerSavingMode;           // Previous state (initialized to force initial config)
const double tempThreshold = 0.75;         // Temperature change threshold to trigger transmission
const double dangerousTemp = 47.5;         // Temperature threshold for immediate alert
const float thresholdInputVoltage = 3.5;   // Input voltage threshold to enter power saving mode

//------------------------------------------------------------------------------
// PWM and Voltage Regulation Configuration
//------------------------------------------------------------------------------
// Initial duty cycle percentage (modifiable by control loop)
float dutyCycle = 52.38;

// Target output voltage that the system should maintain
const float targetVoltage = 10.3;

// Acceptable error margin for voltage regulation
const float tolerance = 0.01;

// Scaling factor to convert ADC readings to actual voltage
const float scaleFactor = 2.471;

// Limits for the duty cycle to prevent excessive changes
const float minDuty = 20.0;  // Minimum allowed duty cycle
const float maxDuty = 75.0;  // Maximum allowed duty cycle

//------------------------------------------------------------------------------
// PI Controller Configuration
//------------------------------------------------------------------------------
// PI Controller Gains for feedback control
const float Kp = 0.3;   // Proportional gain (determines response speed)
const float Ki = 0.01;  // Integral gain (helps reduce steady-state error)

// Stores accumulated error for integral control
float integralError = 0.0;

// Step size for duty cycle adjustment based on PI control output
float dutyStep = 0.00;

//------------------------------------------------------------------------------
// PWM Frequency Configuration
//------------------------------------------------------------------------------
const float fixedFrequency_kHz = 70.0;     // Desired PWM frequency in kHz
const float fixedFrequency_Hz = fixedFrequency_kHz * 1000.0; // Convert to Hz

/**
 * Setup function - runs once at initialization
 * Configures pins, serial communication, PWM, and power saving mode
 */
void setup() {
  // Initialize serial communications
  Serial.begin(9600);
  mySerial.begin(9600);
  
  // Configure Bluetooth control pin
  pinMode(bluetoothPin, OUTPUT);

  // Disable interrupts temporarily for PWM setup
  cli();

  // Configure PWM pins
  pinMode(9, OUTPUT);    // Set pin 9 as PWM output
  pinMode(A2, INPUT);    // Set analog pin A2 as input for voltage measurement
  pinMode(A5, INPUT);    // Set analog pin A5 as input for smoke detector

  // Configure Timer1 for Fast PWM Mode 14 (ICR1 as top value)
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // No prescaler (full speed clock)

  // Set the initial PWM duty cycle
  setDutyCycle(dutyCycle);

  // Re-enable interrupts after configuration
  sei();

  // Initialize power saving mode configuration
  configurePowerSaving(powerSavingMode);
  prevPSM = powerSavingMode;
}

/**
 * Main loop function - runs repeatedly
 * Monitors temperature, smoke, manages power saving, and controls voltage regulation
 */
void loop() {
  // Check input voltage and determine if power saving mode is needed
  float measuredInputVoltage = ADCread(inputVoltagePin) * (5.0 / 1023.0);
  powerSavingMode = (measuredInputVoltage < thresholdInputVoltage);

  // Read thermistor and calculate temperature
  int adcValue = ADCread(thermistorPin);
  double voltage = adcValue * (5.0 / adcMax);
  double resistance = seriesResistor * (5.0 / voltage - 1.0);
  calcSteinhart(resistance);
  
  // Read smoke detector using power-efficient ADC read
  int smokeReading = ADCread(photoresPin);
  
  // Determine smoke state based on threshold
  if(smokeThreshold > smokeReading) {
    smoke = true;
  } else {
    smoke = false;
  }

  // Transmit data if temperature exceeds danger threshold, changes significantly, or smoke is detected
  if((steinhart >= dangerousTemp) || ((fabs(steinhart - prevTemp)) > tempThreshold) || smoke) {
    digitalWrite(bluetoothPin, HIGH);  // Enable Bluetooth module
    delay(125);
    
    // Send temperature data to both serial interfaces
    Serial.print(steinhart);
    mySerial.print(steinhart);
    
    // Add smoke indicator if smoke is detected
    if(smoke) {
      Serial.print("S");
      mySerial.print("S");
      wasSmoke = true;
    }
    
    // Complete the transmission with newlines
    Serial.println();
    mySerial.println();
    
    // Save current temperature as previous for next comparison
    prevTemp = steinhart;
    
    delay(125);
    digitalWrite(bluetoothPin, LOW);   // Disable Bluetooth module
  }
  
  // Special case: If smoke has cleared, send an "all clear" notification
  if(!smoke && wasSmoke) {
    digitalWrite(bluetoothPin, HIGH);  // Enable Bluetooth module
    delay(125);
    
    // Send temperature data to indicate normal conditions
    Serial.print(steinhart);
    mySerial.print(steinhart);
    Serial.println();
    mySerial.println();
    
    wasSmoke = false;  // Reset the smoke state flag
    
    delay(125);
    digitalWrite(bluetoothPin, LOW);   // Disable Bluetooth module
  }

  // Check if power saving mode state has changed
  if (powerSavingMode != prevPSM) {
    configurePowerSaving(powerSavingMode);
    prevPSM = powerSavingMode;
  }

  // Handle system behavior based on power mode
  if (powerSavingMode) {
    // In power saving mode, sleep most of the time
    for (uint8_t i = 0; i < 30; ++i) {
      goSleep();    // each goSleep() ≈1 s
    }
  } else {
    // In normal mode, actively regulate voltage
    float solarValue = ADCread(solarPin);
    float measuredVoltage = (solarValue / 1023.0) * 5.0 * scaleFactor;
    
    // Calculate and update PWM duty cycle based on voltage regulation needs
    updateDutyCycle(measuredVoltage);
    setDutyCycle(dutyCycle);
    
    delay(1000);  // Wait 1 second before next adjustment
  }
}

/**
 * Calculate temperature using Steinhart-Hart equation
 * 
 * @param resistance Thermistor resistance in ohms
 */
void calcSteinhart(double resistance) {
  // Calculate temperature using Beta formula (Steinhart approximation)
  steinhart = resistance / nominalResistance;        // (R/Ro)
  steinhart = log(steinhart);                        // ln(R/Ro)
  steinhart /= betaCoefficient;                      // 1/B * ln(R/Ro)
  steinhart += 1.0 / (nominalTemperature + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                       // Invert
  steinhart -= 273.15;                               // Convert to Celsius
}

/**
 * Set PWM duty cycle by configuring Timer1 registers
 * 
 * @param duty Duty cycle percentage (0-100)
 */
void setDutyCycle(float duty) {
  if (duty < 0 || duty > 100) return;  // Ensure the duty cycle is valid

  // Compute ICR1 based on desired PWM frequency
  ICR1 = (16000000.0 / fixedFrequency_Hz) - 1;

  // Compute OCR1A value to set duty cycle percentage
  OCR1A = (ICR1 * duty) / 100;
}

/**
 * Update duty cycle based on PI control algorithm
 * 
 * @param measuredVoltage Current measured output voltage
 */
void updateDutyCycle(float measuredVoltage) {
  // Compute the error between target and measured voltage
  float error = targetVoltage - measuredVoltage;

  // Only accumulate error if we're not already at the step limits
  if (!(dutyStep == 5 || dutyStep == -5)) {
    if (abs(error) > tolerance) {
      integralError += error; // Accumulate error for integral control
    } else {
      integralError = 0;      // Reset integral error when within tolerance
    }
  }

  // Calculate new step using PI controller and apply limits
  dutyStep = (Kp * error) + (Ki * integralError);
  dutyStep = constrain(dutyStep, -5, 5);

  // Update duty cycle within bounds
  dutyCycle += dutyStep;
  dutyCycle = constrain(dutyCycle, minDuty, maxDuty);
}

/**
 * Power-efficient ADC read function
 * Enables ADC only during the reading, then disables it to save power
 * 
 * @param pin Analog pin to read
 * @return ADC reading (0-1023)
 */
int ADCread(int pin) {
  ADCSRA |=  _BV(ADEN);     // Turn ADC on
  int v = analogRead(pin);   // Take reading
  ADCSRA &= ~_BV(ADEN);     // Turn ADC off to save power
  return v;
}

/**
 * Configure power saving mode settings
 * 
 * @param on True to enable power saving mode, false to disable
 */
void configurePowerSaving(bool on) {
  if (on) {
    // SM = 010 → Power‑down mode
    MCUCR = (MCUCR & ~(_BV(SM2)|_BV(SM1)|_BV(SM0))) | _BV(SM1);

    // Clear any watchdog reset flag
    MCUSR &= ~_BV(WDRF);

    // Allow WDT changes
    WDTCSR |= _BV(WDCE)|_BV(WDE);

    // Set watchdog timer for interrupt-only mode with ~1s prescaler
    WDTCSR = _BV(WDIE) | _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
  } else {
    // Turn watchdog timer completely off
    MCUSR &= ~_BV(WDRF);
    WDTCSR |= _BV(WDCE)|_BV(WDE);
    WDTCSR = 0;

    // Restore sleep mode bits to Idle mode
    MCUCR &= ~(_BV(SM2)|_BV(SM1)|_BV(SM0));
  }
}

/**
 * Put the microcontroller to sleep
 * This function enables the sleep mode bit, executes the sleep instruction,
 * then disables sleep mode after waking up
 */
void goSleep() {
  MCUCR |=  _BV(SE);     // Enable sleep mode
  asm volatile("sleep"); // Enter sleep mode (assembly instruction)
  MCUCR &= ~_BV(SE);     // Disable sleep mode after waking
}

/**
 * Watchdog Timer Interrupt Service Routine
 * This empty ISR is called when the watchdog timer expires
 * It wakes the device from sleep mode
 */
void WDT_vect(void) __attribute__((signal,used));
void WDT_vect(void) {
  // Empty ISR - just wakes up the device
}
