#include <ADC_MC.h>

// Create an ADC object with default 3.3V supply voltage
ADC_MC adc;

void setup() {
  // Initialize the ADC
  adc.init();
  
  // Start serial communication for debugging
  Serial.begin(9600);
  
  // Start the ADC in free-run mode with custom settings
  // Reference voltage: INTVCC1, Sampling rate: SAMPLENUM_64, Gain: GAIN_4x
  adc.start_FREERUN(A0, INTVCC1, SAMPLENUM_64, GAIN_4x);
}

void loop() {
  // Get the current voltage measured by the ADC in free-run mode
  double voltage = adc.get_Voltage_freerun();
  
  // Print the measured voltage
  Serial.print("Free-Run Mode Voltage: ");
  Serial.println(voltage);

  // Get the current voltage measured by the ADC in free-run mode
  int result = adc.get_result_freerun();

  // Print the result
  Serial.print("Free-Run Mode result: ");
  Serial.println(result);
  
  // Wait for a second before the next read
  delay(1000);
}
