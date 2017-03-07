#include "tsys01.h"
#include <Wire.h>

// TSYS01 device commands
#define TSYS01_RESET_COMMAND 0x1E
#define TSYS01_START_ADC_CONVERSION 0x48
#define TSYS01_READ_ADC_TEMPERATURE 0x00
#define PROM_ADDRESS_READ_ADDRESS_0 0xA0
#define PROM_ADDRESS_READ_ADDRESS_1 0xA2
#define PROM_ADDRESS_READ_ADDRESS_2 0xA4
#define PROM_ADDRESS_READ_ADDRESS_3 0xA6
#define PROM_ADDRESS_READ_ADDRESS_4 0xA8
#define PROM_ADDRESS_READ_ADDRESS_5 0xAA
#define PROM_ADDRESS_READ_ADDRESS_6 0xAC
#define PROM_ADDRESS_READ_ADDRESS_7 0xAE
#define PROM_ELEMENTS_NUMBER 8

#define TSYS01_CONVERSION_TIME 10

// function
tsys01::tsys01(void)
    : coeff_mul{COEFF_MUL_0, COEFF_MUL_1, COEFF_MUL_2, COEFF_MUL_3,
                COEFF_MUL_4} {
  tsys01_coeff_read = false;
}

/**
 * \brief Perform initial configuration. Has to be called once.
 */
void tsys01::begin(void) { Wire.begin(); }

/**
 * \brief Configures TSYS01 I2C address to be used depending on HW configuration
 *
 * \param[in] address : TSYS01 I2C address
 *
 */
void tsys01::set_address(enum tsys01_address address) {
  if (address == tsys01_i2c_address_csb_1)
    tsys01_i2c_address = TSYS01_ADDR_CSB_1;
  else
    tsys01_i2c_address = TSYS01_ADDR_CSB_0;
}

/**
 * \brief Check whether TSYS01 device is connected
 *
 * \return bool : status of TSYS01
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool tsys01::is_connected(void) {
  Wire.beginTransmission((uint8_t)tsys01_i2c_address);
  return (Wire.endTransmission() == 0);
}

/**
 * \brief Writes the TSYS01 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys01_status tsys01::write_command(uint8_t cmd) {
  uint8_t i2c_status;

  Wire.beginTransmission((uint8_t)tsys01_i2c_address);
  Wire.write(cmd);
  i2c_status = Wire.endTransmission();

  if (i2c_status == tsys01_STATUS_ERR_OVERFLOW)
    return tsys01_status_no_i2c_acknowledge;
  if (i2c_status != tsys01_STATUS_OK)
    return tsys01_status_i2c_transfer_error;

  return tsys01_status_ok;
}

/**
 * \brief Reset the TSYS01 device
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys01_status tsys01::reset(void) {
  return write_command(TSYS01_RESET_COMMAND);
}

/**
 * \brief Reads the tsys01 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] uint16_t* : Value read in EEPROM
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys01_status tsys01::read_eeprom_coeff(uint8_t command, uint16_t *coeff) {
  enum tsys01_status status;
  uint8_t i2c_status;
  uint8_t buffer[2];
  uint8_t i;

  Wire.beginTransmission((uint8_t)tsys01_i2c_address);
  Wire.write(command);
  i2c_status = Wire.endTransmission();

  Wire.requestFrom((uint8_t)tsys01_i2c_address, 2U);
  for (i = 0; i < 2; i++) {
    buffer[i] = Wire.read();
  }

  // Send the conversion command

  if (i2c_status == tsys01_STATUS_ERR_OVERFLOW)
    return tsys01_status_no_i2c_acknowledge;
  if (i2c_status != tsys01_STATUS_OK)
    return tsys01_status_i2c_transfer_error;

  *coeff = (buffer[0] << 8) | buffer[1];

  return tsys01_status_ok;
}

/**
 * \brief Reads the tsys01 EEPROM coefficients to store them for computation.
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys01_status_crc_error : CRC error on PROM coefficients
 */
enum tsys01_status tsys01::read_eeprom(void) {
  enum tsys01_status status;
  uint8_t i;

  // Read all coefficients from EEPROM
  for (i = 0; i < PROM_ELEMENTS_NUMBER; i++) {
    status = read_eeprom_coeff(PROM_ADDRESS_READ_ADDRESS_0 + i * 2,
                               eeprom_coeff + i);
    if (status != tsys01_status_ok)
      return status;
  }

  // CRC check
  if (crc_check(eeprom_coeff))
    return tsys01_status_crc_error;

  tsys01_coeff_read = true;

  return tsys01_status_ok;
}

/**
 * \brief CRC check
 *
 * \param[in] uint16_t *: List of EEPROM coefficients
 *
 * \return bool : TRUE if CRC is OK, FALSE if KO
 */
bool tsys01::crc_check(uint16_t *n_prom) {
  uint8_t cnt;
  uint16_t sum = 0;

  for (cnt = 0; cnt < PROM_ELEMENTS_NUMBER; cnt++)
    // Sum each byte of the coefficients
    sum += ((n_prom[cnt] >> 8) + (n_prom[cnt] & 0xFF));

  return (sum & 0xFF == 0);
}

/**
 * \brief Reads the temperature ADC value
 *
 * \param[out] uint32_t* : Temperature ADC value.
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys01_status_crc_error : CRC error on PROM coefficients
 */
enum tsys01_status tsys01::conversion_and_read_adc(uint32_t *adc) {
  enum tsys01_status status;
  uint8_t buffer[3];
  uint8_t i;

  /* Read data */

  // Wait for conversion
  Wire.beginTransmission((uint8_t)tsys01_i2c_address);
  Wire.write((uint8_t)TSYS01_START_ADC_CONVERSION);
  Wire.endTransmission();
  delay(TSYS01_CONVERSION_TIME);

  // Read ADC
  Wire.beginTransmission((uint8_t)tsys01_i2c_address);
  Wire.write((uint8_t)TSYS01_READ_ADC_TEMPERATURE);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)tsys01_i2c_address, 3U);
  for (i = 0; i < 3; i++) {
    buffer[i] = Wire.read();
  }

  if (status != tsys01_status_ok)
    return status;

  // Send the read command
  if (status != tsys01_status_ok)
    return status;

  *adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) |
         (uint32_t)buffer[2];

  return status;
}

/**
 * \brief Reads the temperature ADC value and compute the degree Celsius one.
 *
 * Perform algorithm computation based on adc 16 bits ( 24-bits value is divided
 * by 256)<BR>
 * \verbatim
 *  T (degC) =     (-2) * k4 * 10e-21 * adc^4 +
 *                    4 * k3 * 10e-16 * adc^3 +
 *                 (-2) * k2 * 10e-11 * adc^2 +
 *                    1 * k1 * 10e-6  * adc +
 *               (-1.5) * k0 * 10e-2
 * \endverbatim
 *
 * Factored into
 * \verbatim
 *  T (degC) = 10e-2.( a.k0 + 10e1 * 10e-5.adc.( b.k1 + 10e-5.adc ( c.k2 +
 * 10e-5.adc.( d.k3 + 10e-5.adc.e.k4 ) ) ) )
 * \endverbatim
 *
 * \param[out] float* : Celsius Degree temperature value
 *
 * \return tsys01_status : status of TSYS01
 *       - tsys01_status_ok : I2C transfer completed successfully
 *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys01_status_crc_error : CRC error on PROM coefficients
 */

enum tsys01_status tsys01::read_temperature(float *temperature) {
  enum tsys01_status status = tsys01_status_ok;
  uint32_t adc;
  uint8_t i;
  float temp = 0;

  // If first time temperature is requested, get EEPROM coefficients
  if (tsys01_coeff_read == false)
    status = read_eeprom();
  if (status != tsys01_status_ok)
    return status;

  status = conversion_and_read_adc(&adc);
  if (status != tsys01_status_ok)
    return status;

  adc /= 256;

  for (i = 4; i > 0; i--) {
    temp += coeff_mul[i] *
            eeprom_coeff[1 + (4 - i)]; // eeprom_coeff[1+(4-i)] equiv. ki
    temp *= (float)adc / 100000;
  }
  temp *= 10;
  temp += coeff_mul[0] * eeprom_coeff[5];
  temp /= 100;

  *temperature = temp;

  return status;
}
