#include <ADC_MC.h>

#define Serial SerialUSB

// Create an ADC object with default 3.3V supply voltage
ADC_MC adc;

void setup() {
  // Initialize the ADC
  adc.init();
  
  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the voltage from pin A0 with custom settings
  // Reference voltage: INT1V, Sampling rate: SAMPLENUM_16, Gain: GAIN_2x
  double voltage = adc.readVoltage_once(A0, INT1V, SAMPLENUM_16, GAIN_2x);
  
  // Print the measured voltage
  Serial.print("Measured Voltage from A0: ");
  Serial.println(voltage);

  // Read the voltage from pin A1 with custom settings
  // Reference voltage: INT1V, Sampling rate: SAMPLENUM_16, Gain: GAIN_2x
  double voltage = adc.readVoltage_once(A1, INT1V, SAMPLENUM_16, GAIN_2x);
  
  // Print the measured voltage
  Serial.print("Measured Voltage from A1: ");
  Serial.println(voltage);
  
  // Wait for a second before the next read
  delay(1000);
}
