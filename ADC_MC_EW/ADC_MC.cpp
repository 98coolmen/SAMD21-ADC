
/**
Library für den ADC des  ATSAMD21G18
Author: J.Neufeld & E.Wolf
Date: 09/2024
*/

#include "ADC_MC.h"

#include <Arduino.h>
#include <inttypes.h>



// -------------------------------------------------------------------------- //
ADC_MC::ADC_MC() {
  this->VCC = 3.3;
  this->REF_select = 0;
  this->SMPR = 0;
  this->GAIN = 0;
  this->VREFA_V = 0.0;
  this->VREFB_V = 0.0;
}
ADC_MC::ADC_MC(double VCC) {
  this->VCC = VCC;
}

void ADC_MC::init() {

  ADC->CTRLA.bit.ENABLE = 0;  // Disable the ADC, has do be disabled during configuration
  // Enable the APBC clock for the ADC
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;

  SYSCTRL->OSC8M.reg |= SYSCTRL_OSC8M_ENABLE;

  // Setup clock GCLK3 for no div factor
  GCLK->GENDIV.reg |= GCLK_GENDIV_ID(3) | GCLK_GENDIV_DIV(0);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  // configure the generator of the generic clock, which is 8MHz clock
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_ID(3);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  // enable clock, set gen clock number, and ID to where the clock goes (30 is ADC)
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(3) | GCLK_CLKCTRL_ID(30);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  uint32_t bias = (*((uint32_t *)ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *)ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

  ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
  ADC->CTRLB.bit.PRESCALER = 0x00;  // No prescaler

  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
  set_REFERENCE(INT1V);
  set_SAMPLENUM(SAMPLENUM_1);
  set_GAIN(GAIN_DIV2);
  //enbl_oversample(os_select);
}

/* gibt die Eingegebende Eingangsspannung des MC zurück */
double ADC_MC::get_VCC() {
  return this->VCC;
}

/* setzt die Eingegebende Eingangsspannung des MC */
void ADC_MC::set_VCC(double VCC) {
  this->VCC = VCC;
}

/* gibt die Eingegebende VREFA spannung des MC zurück */
double ADC_MC::get_VREFA() {
  return this->VREFA_V;
}

/* setzt die Eingegebende VREFA spannung des MC */
void ADC_MC::set_VREFA(double VREFA_V) {
  this->VREFA_V = VREFA_V;
}

/* gibt die Eingegebende VREFB spannung des MC zurück */
double ADC_MC::get_VREFB() {
  return this->VREFB_V;
}

/* setzt die Eingegebende VREFB spannung des MC */
void ADC_MC::set_VREFB(double VREFB_V) {
  this->VREFB_V = VREFB_V;
}


/*---------------Einstellungen und Funktionen dazu------------------*/
/* Setzt due Spannungsreferenz des ADCs
Options are INTVCC1 (1/1.48 VCC), INTVCC0 (1/2 VCC), INT1V (1V), VREFA (external), VREFB (external)*/
void ADC_MC::set_REFERENCE(uint8_t REF_select) {
  this->REF_select = REF_select;
  ADC->CTRLA.bit.ENABLE = 0;  // Schaltet den ADC aus um Einstellungen zu ändern
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)
    ;                  // Wait for MCU to finish setting registers
  switch (REF_select)  // Options are INTVCC1 (1/2 VDDANA), INTVCC0 (1/1.48 VDDANA), INT1V (1V), VREFA (external), VREFB (external)
  {
    case INTVCC1:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;  // 1/2 von VDDANA

      break;
    case INTVCC0:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val;  // 1/1.48 von VDDANA

      break;
    case INT1V:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;  // 1 V reference voltage

      break;
    case VREFA:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;  // External reference on VREFA pin

      break;
    case VREFB:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFB_Val;  // External reference on VREFB pin

      break;
    default:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;  // 1 V reference voltage

      break;
  }
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)  // Wait for MCU to finish setting registers
    ;
}
uint8_t ADC_MC::get_ADC_SAMPLENUM() {
  // Auslesen der aktuellen Anzahl der Samples
  uint8_t sampleNum = ADC->AVGCTRL.bit.SAMPLENUM;
  return sampleNum;
}
/* Setzt die Anzahl an Samples die der ADC nimmt (je höher desto besser)*/
void ADC_MC::set_SAMPLENUM(uint8_t SMPR) {
  this->SMPR = SMPR;
  ADC->CTRLA.bit.ENABLE = 0;  // Schaltet den ADC aus um Einstellungen zu ändern
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)
    ;  // Wait for MCU to finish setting registers
  switch (SMPR) {
    case SAMPLENUM_1:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_1_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x0;
      break;
    case SAMPLENUM_2:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_2_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x1;
      break;
    case SAMPLENUM_4:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_4_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x2;
      ADC->AVGCTRL.bit.SAMPLENUM = 0x2;
      break;
    case SAMPLENUM_8:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_8_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x3;
      break;
    case SAMPLENUM_16:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_16_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x4;
      break;
    case SAMPLENUM_32:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_32_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x4;
      break;
    case SAMPLENUM_64:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_64_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x4;
      break;
    case SAMPLENUM_128:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_128_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x4;
      break;
    case SAMPLENUM_256:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_256_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x4;
      break;
    case SAMPLENUM_512:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_512_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x4;
      break;
    case SAMPLENUM_1024:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_1024_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x4;
      break;
    default:
      ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_1_Val;
      ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
      ADC->AVGCTRL.bit.ADJRES = 0x0;
      break;
  }
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)  // Wait for MCU to finish setting registers
    ;
}
/* Setzt den GAIN des ADC */
void ADC_MC::set_GAIN(uint8_t GAIN) {
  this->GAIN = GAIN;
  ADC->CTRLA.bit.ENABLE = 0;  // Schaltet den ADC aus um Einstellungen zu ändern
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)
    ;            // Wait for MCU to finish setting registers
  switch (GAIN)  // Select Gain
  {
    case GAIN_DIV2:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;  // Gain = 0.5, default Gain
      break;
    case GAIN_1x:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;  // Gain = 1
      break;
    case GAIN_2x:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_2X_Val;  // Gain = 2
      break;
    case GAIN_4x:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_4X_Val;  // Gain = 4
      break;
    case GAIN_8x:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_8X_Val;  // Gain = 8
      break;
    case GAIN_16x:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_16X_Val;  // Gain = 16
      break;
    default:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;  // Gain = 0.5, default Gain
      break;
  }
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)  // Wait for MCU to finish setting registers
    ;
}
/* Setzt den ADC in den Freerun Modus */
bool ADC_MC::set_FREERUN(bool FREERUN) {
  ADC->CTRLA.bit.ENABLE = 0;  // Schaltet den ADC aus um Einstellungen zu ändern
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)
    ;  // Wait for MCU to finish setting registers
  if (FREERUN == 1) {
    // Freerun-Modus aktivieren
    ADC->CTRLB.bit.FREERUN = 1;
    while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)  // Wait for MCU to finish setting registers
      ;
    return 1;
  } else {
    // Freerun-Modus aktivieren
    ADC->CTRLB.bit.FREERUN = 0;
    while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)  // Wait for MCU to finish setting registers
      ;
    return 0;
  }
}
/* Startet Freerunning Modus mit den eingestellen Einstellungen */
void ADC_MC::start_FREERUN(uint8_t pin, uint8_t REF_select, uint8_t SMPR, uint8_t GAIN) {
  set_SETTINGS(pin, REF_select, SMPR, GAIN);
  set_FREERUN(true);
  enable();
}
/*Startet den ADC*/
bool ADC_MC::enable() {
  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
  ADC->SWTRIG.reg = ADC_SWTRIG_START;  // Start conversion
  ADC->SWTRIG.bit.START = 1;
  return 1;
}


/* gibt die maximal messbare Spannung aus bei den aktuellen Einstellungen aus */
double ADC_MC::get_MAX_Voltage() {
  double ref_vol = get_Reference();
  double gain_temp;
  if (GAIN == GAIN_DIV2) {
    gain_temp = 0.5;
  } else {
    gain_temp = GAIN;
  }
  return (ref_vol / gain_temp);
}

/* Stellt die Einstellungen ein auf dem ADC */
void ADC_MC::set_SETTINGS(uint8_t pin, uint8_t REF_select, uint8_t SMPR, uint8_t GAIN) {
  this->REF_select = REF_select;
  this->SMPR = SMPR;
  this->GAIN = GAIN;
  ADC->CTRLA.bit.ENABLE = 0;  // Schaltet den ADC aus um Einstellungen zu ändern
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)
    ;  // Wait for MCU to finish setting registers
       // Freerun-Modus aktivieren
  set_FREERUN(0);
  ADC->INPUTCTRL.bit.MUXPOS = pin;  // Hier wird der gemessende Pin festgelegt
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)
    ;  // Wait for MCU to finish setting registers

  ADC->AVGCTRL.reg |= ADC_AVGCTRL_ADJRES(0);    // | ADC_AVGCTRL_SAMPLENUM(0x8);
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY)  // Wait for MCU to finish setting registers
    ;

  set_REFERENCE(REF_select);
  set_SAMPLENUM(SMPR);
  set_GAIN(GAIN);
}

/*gibt die einestellte Reference wieder*/
double ADC_MC::get_Reference() {
  double ref_vol;
  if (this->REF_select == INTVCC1) {
    ref_vol = this->VCC / 2;
  } else if (this->REF_select == INT1V) {
    ref_vol = INT1V;
  } else if (this->REF_select == INTVCC0) {
    ref_vol = this->VCC / 1.48;
  } else if (this->REF_select == VREFA) {
    ref_vol = this->VREFA_V;
  } else if (this->REF_select == VREFB) {
    ref_vol = this->VREFB_V;
  }
  return ref_vol;
}

/*-------------------Messungen und Funktionen--------------------*/


/* ließt den angegebenden Pin einmal aus */
int ADC_MC::readPin_once(uint8_t pin, uint8_t REF_select, uint8_t SMPR, uint8_t GAIN) {

  set_SETTINGS(pin, REF_select, SMPR, GAIN);

  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
  ADC->SWTRIG.reg = ADC_SWTRIG_START;  // Start conversion
  while (!ADC->INTFLAG.bit.RESRDY)     // Wait for the conversion to complete
  {
  }
  int ADC_result = ADC->RESULT.reg;
  // ADC_result = ADC_result_bin; // * (ADC_FSR / ADC_resolution); // convert binary value to decimal voltage
  return ADC_result;
}

/*Misst einen Pin und wandlet direkt in Volt an je nach Refernez*/
double ADC_MC::readVoltage_once(uint8_t pin, uint8_t REF_select, uint8_t SMPR, uint8_t GAIN) {
  int res = readPin_once(pin, REF_select, SMPR, GAIN);
  double out = voltage_conv(res);
  return out;
}

/* ließt den angegebenden Pin einmal aus */
int ADC_MC::readPin_once() {
  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
  ADC->SWTRIG.reg = ADC_SWTRIG_START;  // Start conversion
  while (!ADC->INTFLAG.bit.RESRDY)     // Wait for the conversion to complete
  {
  }
  int ADC_result = ADC->RESULT.reg;
  // ADC_result = ADC_result_bin; // * (ADC_FSR / ADC_resolution); // convert binary value to decimal voltage
  return ADC_result;
}

/*Misst einen Pin und wandlet direkt in Volt an je nach Refernez*/
double ADC_MC::readVoltage_once() {
  int res = readPin_once();
  double out = voltage_conv(res);
  return out;
}

/* Wandelt den gemessenden Wert in Volt um je nach Referenz */
double ADC_MC::voltage_conv(int measurment) {
  double ref_vol = get_Reference();

  double gain_temp;
  if (GAIN == GAIN_DIV2) {
    gain_temp = 0.5;
  } else {
    gain_temp = GAIN;
  }

  double out = (double(measurment) / 0b111111111111) * ref_vol / gain_temp;
  //double out = (double(measurment) / (pow(2, 0b111111111111) - 1)) * ref_vol * gain_temp;
  return out;
}

/* gibt das Resultat wieder */
int ADC_MC::get_result_freerun() {
  while (!ADC->INTFLAG.bit.RESRDY)  // Wait for the conversion to complete
  {
  }
  int ADC_result = ADC->RESULT.reg;
  // ADC_result = ADC_result_bin; // * (ADC_FSR / ADC_resolution); // convert binary value to decimal voltage
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;  //reset der Flag das das ergebniss anliegt
  return ADC_result;
}

/*Gibt die gemessende Spannung unter beachtung der aktuell eingestellen Werte aus*/
double ADC_MC::get_Voltage_freerun() {
  while (!ADC->INTFLAG.bit.RESRDY)  // Wait for the conversion to complete
  {
  }
  int ADC_result = ADC->RESULT.reg;
  // ADC_result = ADC_result_bin; // * (ADC_FSR / ADC_resolution); // convert binary value to decimal voltage
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;  //reset der Flag das das ergebniss anliegt
  double out = voltage_conv(ADC_result);
  return out;
}
