/**
Library für den ADC des  ATSAMD21G18
Author: J.Neufeld & E.Wolf
Date: 09/2024
*/

#include <Arduino.h>
#include <inttypes.h>


// Definitionen für die Auswahl der Referenzspannung
#define INTVCC1 10
#define INTVCC0 0
#define INT1V 1
#define VREFA 20
#define VREFB 30

// Definitionen für die Auswahl der Abtastanzahl
#define SAMPLENUM_1 1
#define SAMPLENUM_2 2
#define SAMPLENUM_4 4
#define SAMPLENUM_8 8
#define SAMPLENUM_16 16
#define SAMPLENUM_32 32
#define SAMPLENUM_64 64
#define SAMPLENUM_128 128
#define SAMPLENUM_256 129
#define SAMPLENUM_512 130
#define SAMPLENUM_1024 131

// Definitionen für die Auswahl der Verstärkung
#define GAIN_DIV2 0
#define GAIN_1x 1
#define GAIN_2x 2
#define GAIN_4x 4
#define GAIN_8x 8
#define GAIN_16x 16

class ADC_MC {
public:
  /**
   * @brief Constructor for the ADC object with 3.3V as supply voltage
   */
  ADC_MC();

  /**
   * @brief Constructor for the ADC object with the specified supply voltage
   * @param VCC The supply voltage of the MC
   */
  ADC_MC(double VCC);

  /**
   * @brief Initializes the ADC for the first time
   */
  void init();

  /**
   * @brief Reads the voltage with current settings
   * @return The measured voltage
   */
  int readPin_once();

  /**
   * @brief Reads the value from the specified pin once
   * @param REF_select Sets the reference voltage for the measurement. Possible values are:
   * - INT1V
   * - INTVCC0 (VCC/2)
   * - INTVCC1 (VCC/1.48)
   * - VREFA
   * - VREFB
   * @param SMPR Sets the sampling rate of the ADC. Possible values are:
   * - SAMPLENUM_1
   * - SAMPLENUM_2
   * - SAMPLENUM_4
   * - SAMPLENUM_8
   * - SAMPLENUM_16
   * - SAMPLENUM_32
   * - SAMPLENUM_64
   * - SAMPLENUM_128
   * - SAMPLENUM_256
   * - SAMPLENUM_512
   * - SAMPLENUM_1024
   * @param GAIN Sets the gain of the ADC. Possible values are:
   * - GAIN_DIV2
   * - GAIN_1x
   * - GAIN_2x
   * - GAIN_4x
   * - GAIN_8x
   * - GAIN_16x
   * @return The measured value
   */
  int readPin_once(uint8_t pin, uint8_t REF_select, uint8_t SMPR, uint8_t GAIN);

  /**
   * @brief Reads the voltage from the specified pin once
   * @param REF_select Sets the reference voltage for the measurement. Possible values are:
   * - INT1V
   * - INTVCC0 (VCC/2)
   * - INTVCC1 (VCC/1.48)
   * - VREFA
   * - VREFB
   * @param SMPR Sets the sampling rate of the ADC. Possible values are:
   * - SAMPLENUM_1
   * - SAMPLENUM_2
   * - SAMPLENUM_4
   * - SAMPLENUM_8
   * - SAMPLENUM_16
   * - SAMPLENUM_32
   * - SAMPLENUM_64
   * - SAMPLENUM_128
   * - SAMPLENUM_256
   * - SAMPLENUM_512
   * - SAMPLENUM_1024
   * @param GAIN Sets the gain of the ADC. Possible values are:
   * - GAIN_DIV2
   * - GAIN_1x
   * - GAIN_2x
   * - GAIN_4x
   * - GAIN_8x
   * - GAIN_16x
   * @return The measured voltage
   */
  double readVoltage_once(uint8_t pin, uint8_t REF_select, uint8_t SMPR, uint8_t GAIN);

  /**
   * @brief Reads the voltage with current settings
   * @return The measured voltage
   */
  double readVoltage_once();

  /**
   * @brief Converts the measured value to voltage with current settings
   * @param measurement The measured value
   * @return The converted voltage
   */
  double voltage_conv(int measurement);

  /**
   * @brief Gets the result of the ADC conversion
   * @return The result of the conversion
   */
  int get_result_freerun();

  /**
   * @brief Gets the current voltage measured by the ADC
   * @return The current voltage
   */
  double get_Voltage_freerun();

  /**
   * @brief Gets the current sample number setting of the ADC
   * @return The sample number setting
   */
  uint8_t get_ADC_SAMPLENUM();

  /**
   * @brief Starts the ADC in free-run mode with the specified settings
   * @param REF_select Sets the reference voltage for the measurement. Possible values are:
   * - INT1V
   * - INTVCC0 (VCC/2)
   * - INTVCC1 (VCC/1.48)
   * - VREFA
   * - VREFB
   * @param SMPR Sets the sampling rate of the ADC. Possible values are:
   * - SAMPLENUM_1
   * - SAMPLENUM_2
   * - SAMPLENUM_4
   * - SAMPLENUM_8
   * - SAMPLENUM_16
   * - SAMPLENUM_32
   * - SAMPLENUM_64
   * - SAMPLENUM_128
   * - SAMPLENUM_256
   * - SAMPLENUM_512
   * - SAMPLENUM_1024
   * @param GAIN Sets the gain of the ADC. Possible values are:
   * - GAIN_DIV2
   * - GAIN_1x
   * - GAIN_2x
   * - GAIN_4x
   * - GAIN_8x
   * - GAIN_16x
   */
  void start_FREERUN(uint8_t pin, uint8_t REF_select, uint8_t SMPR, uint8_t GAIN);

  /**
   * @brief Gets the maximum voltage that can be measured by the ADC with current settings
   * @return The maximum voltage
   */
  double get_MAX_Voltage();

  /**
   * @brief Sets the supply voltage of the MC
   * @param VCC The supply voltage
   */
  void set_VCC(double VCC);

  /**
   * @brief Gets the supply voltage of the MC
   * @return The supply voltage
   */
  double get_VCC();

  /**
   * @brief Gets the reference voltage A
   * @return The reference voltage A
   */
  double get_VREFA();

  /**
   * @brief Sets the reference voltage A (PIN PA03, ARef)
   * @param VREFA_V The reference voltage A
   */
  void set_VREFA(double VREFA_V);

  /**
   * @brief Gets the reference voltage B (PIN PA04, A3)
   * @return The reference voltage B
   */
  double get_VREFB();

  /**
   * @brief Sets the reference voltage B
   * @param VREFB_V The reference voltage B
   */
  void set_VREFB(double VREFB_V);

  /**
   * @brief Configures the settings of the ADC
   * @param pin Sets the pin to be measured
   * @param REF_select Sets the reference voltage for the measurement. Possible values are:
   * - INT1V
   * - INTVCC0 (VCC/2)
   * - INTVCC1 (VCC/1.48)
   * - VREFA
   * - VREFB
   * @param SMPR Sets the sampling rate of the ADC. Possible values are:
   * - SAMPLENUM_1
   * - SAMPLENUM_2
   * - SAMPLENUM_4
   * - SAMPLENUM_8
   * - SAMPLENUM_16
   * - SAMPLENUM_32
   * - SAMPLENUM_64
   * - SAMPLENUM_128
   * - SAMPLENUM_256
   * - SAMPLENUM_512
   * - SAMPLENUM_1024
   * @param GAIN Sets the gain of the ADC. Possible values are:
   * - GAIN_DIV2
   * - GAIN_1x
   * - GAIN_2x
   * - GAIN_4x
   * - GAIN_8x
   * - GAIN_16x
   */
  void set_SETTINGS(uint8_t pin, uint8_t REF_select, uint8_t SMPR, uint8_t GAIN);

private:
  double VCC;
  double VREFA_V;
  double VREFB_V;
  int REF_select;
  int SMPR;
  int GAIN;

  /**
   * @brief Setzt den ADC in den Freilaufmodus
   * @param FREERUN True, um den Freilaufmodus zu aktivieren, false, um ihn zu deaktivieren
   * @return True, wenn erfolgreich, false andernfalls
   */
  bool set_FREERUN(bool FREERUN);

  /**
   * @brief Aktiviert den ADC
   * @return True, wenn erfolgreich, false andernfalls
   */
  bool enable();

  /**
   * @brief Setzt die Referenzspannung für den ADC
   * @param REF_select Die Auswahl der Referenzspannung
   */
  void set_REFERENCE(uint8_t REF_select);

  /**
   * @brief Setzt die Abtastanzahl für den ADC
   * @param SMPR Die Auswahl der Abtastanzahl
   */
  void set_SAMPLENUM(uint8_t SMPR);

  /**
   * @brief Setzt die Verstärkung für den ADC
   * @param GAIN Die Auswahl der Verstärkung
   */
  void set_GAIN(uint8_t GAIN);

  /**
   * @brief Holt die aktuelle Referenzspannung
   * @return Die aktuelle Referenzspannung
   */
  double get_Reference();
};
