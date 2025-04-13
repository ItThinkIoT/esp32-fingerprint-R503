/*!
 * @file ItThink_Fingerprint.cpp
 *
 * @mainpage ItThink Fingerprint Sensor Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for our optical Fingerprint sensor
 *
 *
 * These displays use TTL Serial to communicate, 2 pins are required to
 * interface
 * ItThink invests time and resources providing this open source code,
 * please support ItThink and open-source hardware by purchasing
 * products from ItThink!
 *
 * @section author Author
 *
 * Written by Guihgo for ItThink.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 *
 */

#include "ItThink_Fingerprint.h"

// #define FINGERPRINT_DEBUG

/*!
 * @brief Just Write the command packet
 */
#define WRITE_CMD_PACKET(...)                                                \
  uint8_t data[] = {__VA_ARGS__};                                            \
  ItThink_Fingerprint_Packet packet(FINGERPRINT_COMMANDPACKET, sizeof(data), \
                                    data);                                   \
  writeStructuredPacket(packet);

/*!
 * @brief Await the command packet
 */
#define AWAIT_CMD_PACKET(await, ...)                                                   \
  WRITE_CMD_PACKET(__VA_ARGS__);                                                       \
  if (getStructuredPacket(&packet, DEFAULTTIMEOUT, (uint32_t)await) != FINGERPRINT_OK) \
    return FINGERPRINT_PACKETRECIEVEERR;                                               \
  if (packet.type != FINGERPRINT_ACKPACKET)                                            \
    return FINGERPRINT_PACKETRECIEVEERR;

/*!
 * @brief Gets the command packet
 */
#define GET_CMD_PACKET(...)                           \
  WRITE_CMD_PACKET(__VA_ARGS__);                      \
  if (getStructuredPacket(&packet) != FINGERPRINT_OK) \
    return FINGERPRINT_PACKETRECIEVEERR;              \
  if (packet.type != FINGERPRINT_ACKPACKET)           \
    return FINGERPRINT_PACKETRECIEVEERR;

/*!
 * @brief Sends the command packet
 */
#define SEND_CMD_PACKET(...)   \
  GET_CMD_PACKET(__VA_ARGS__); \
  return packet.data[0];

// #define RESULT_FIND(fingerID, confidence)                    \
//   (*fingerID) = (packet.data[2] << 8) | packet.data[3];        \
//   (*confidence) = (packet.data[4] << 8) | packet.data[5];

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

#if defined(__AVR__) || defined(ESP8266) || defined(FREEDOM_E300_HIFIVE1)
/**************************************************************************/
/*!
    @brief  Instantiates sensor with Software Serial
    @param  ss Pointer to SoftwareSerial object
    @param  password 32-bit integer password (default is 0)
*/
/**************************************************************************/
ItThink_Fingerprint::ItThink_Fingerprint(SoftwareSerial *ss,
                                         uint32_t password)
{
  thePassword = password;
  theAddress = 0xFFFFFFFF;

  hwSerial = NULL;
  swSerial = ss;
  mySerial = swSerial;
}
#endif

/**************************************************************************/
/*!
    @brief  Instantiates sensor with Hardware Serial
    @param  hs Pointer to HardwareSerial object
    @param  password 32-bit integer password (default is 0)

*/
/**************************************************************************/
ItThink_Fingerprint::ItThink_Fingerprint(HardwareSerial *hs,
                                         uint32_t password)
{
  thePassword = password;
  theAddress = 0xFFFFFFFF;

#if defined(__AVR__) || defined(ESP8266) || defined(FREEDOM_E300_HIFIVE1)
  swSerial = NULL;
#endif
  hwSerial = hs;
  mySerial = hwSerial;
}

/**************************************************************************/
/*!
    @brief  Instantiates sensor with a stream for Serial
    @param  serial Pointer to a Stream object
    @param  password 32-bit integer password (default is 0)

*/
/**************************************************************************/

ItThink_Fingerprint::ItThink_Fingerprint(Stream *serial, uint32_t password)
{

  thePassword = password;
  theAddress = 0xFFFFFFFF;

  hwSerial = NULL;
#if defined(__AVR__) || defined(ESP8266) || defined(FREEDOM_E300_HIFIVE1)
  swSerial = NULL;
#endif
  mySerial = serial;
}

/**************************************************************************/
/*!
    @brief  Initializes serial interface and baud rate
    @param  baudrate Sensor's UART baud rate (usually 57600, 9600 or 115200)
*/
/**************************************************************************/
void ItThink_Fingerprint::begin(uint32_t baudrate)
{
  // delay(1000); // one second delay to let the sensor 'boot up'

  if (hwSerial)
    hwSerial->begin(baudrate);
#if defined(__AVR__) || defined(ESP8266) || defined(FREEDOM_E300_HIFIVE1)
  if (swSerial)
    swSerial->begin(baudrate);
#endif
}

/**************************************************************************/
/*!
    @brief  Verifies the sensors' access password (default password is
   0x0000000). A good way to also check if the sensors is active and responding
    @returns True if password is correct
*/
/**************************************************************************/
boolean ItThink_Fingerprint::verifyPassword(void)
{
  return checkPassword() == FINGERPRINT_OK;
}

uint8_t ItThink_Fingerprint::checkPassword(void)
{
  GET_CMD_PACKET(FINGERPRINT_VERIFYPASSWORD, (uint8_t)(thePassword >> 24),
                 (uint8_t)(thePassword >> 16), (uint8_t)(thePassword >> 8),
                 (uint8_t)(thePassword & 0xFF));
  if (packet.data[0] == FINGERPRINT_OK)
    return FINGERPRINT_OK;
  else
    return FINGERPRINT_PACKETRECIEVEERR;
}

/**************************************************************************/
/*!
    @brief  Get the sensors parameters, fills in the member variables
    status_reg, system_id, capacity, security_level, device_addr, packet_len
    and baud_rate
    @returns True if password is correct
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::getParameters(void)
{
  GET_CMD_PACKET(FINGERPRINT_READSYSPARAM);

  status_reg = ((uint16_t)packet.data[1] << 8) | packet.data[2];
  system_id = ((uint16_t)packet.data[3] << 8) | packet.data[4];
  capacity = ((uint16_t)packet.data[5] << 8) | packet.data[6];
  security_level = ((uint16_t)packet.data[7] << 8) | packet.data[8];
  device_addr = ((uint32_t)packet.data[9] << 24) |
                ((uint32_t)packet.data[10] << 16) |
                ((uint32_t)packet.data[11] << 8) | (uint32_t)packet.data[12];
  packet_len = ((uint16_t)packet.data[13] << 8) | packet.data[14];
  if (packet_len == 0)
  {
    packet_len = 32;
  }
  else if (packet_len == 1)
  {
    packet_len = 64;
  }
  else if (packet_len == 2)
  {
    packet_len = 128;
  }
  else if (packet_len == 3)
  {
    packet_len = 256;
  }
  baud_rate = (((uint16_t)packet.data[15] << 8) | packet.data[16]) * 9600;

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to take an image of the finger pressed on surface
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_NOFINGER</code> if no finger detected
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_IMAGEFAIL</code> on imaging error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::getImage(void)
{
  SEND_CMD_PACKET(FINGERPRINT_GETIMAGE);
}
/**************************************************************************/
/*!
    @brief   Ask the sensor to take an image of the finger pressed on surface
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_NOFINGER</code> if no finger detected
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_IMAGEFAIL</code> on imaging error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::getImageEx(void)
{
  SEND_CMD_PACKET(FINGERPRINT_GETIMAGE_EX);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to its firmware version
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::getFirmwareVersion(void)
{
  GET_CMD_PACKET(FINGERPRINT_FIRMWARE_VERSION);
  // firmware_version = (uint32_t)packet.data;
  if (packet.data[0] != FINGERPRINT_OK)
    return FINGERPRINT_PACKETRECIEVEERR;

  memcpy(firmware_version, packet.data + 1, 32);

  return FINGERPRINT_OK;
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to auto enroll
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_TIMEOUT</code> timeout of 15secs was reached
    @returns <code>FINGERPRINT_AUTO_ENROLL_ALREADY_EXISTS</code> fingerprint already exists
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::autoEnroll(uint8_t location, bool allowDupID, bool allowDupFP, bool returnErrorStatus, bool fingerMustLeave)
{
  AWAIT_CMD_PACKET(15000, FINGERPRINT_AUTO_ENROLL, location, allowDupID, allowDupFP, returnErrorStatus, fingerMustLeave);

  if (packet.data[0] == FINGERPRINT_AUTO_ENROLL_TIMEOUT)
    return FINGERPRINT_TIMEOUT;

  if (packet.data[0] != FINGERPRINT_OK)
    return FINGERPRINT_PACKETRECIEVEERR;

  while (packet.data[1] != FINGERPRINT_AUTO_ENROLL_STORAGE_TEMPLATE)
  {
    if (getStructuredPacket(&packet, DEFAULTTIMEOUT, (uint32_t)15000) != FINGERPRINT_OK)
      return FINGERPRINT_PACKETRECIEVEERR;
    if (packet.type != FINGERPRINT_ACKPACKET)
      return FINGERPRINT_PACKETRECIEVEERR;

    if (packet.data[0] == FINGERPRINT_AUTO_ENROLL_ALREADY_EXISTS)
      return FINGERPRINT_AUTO_ENROLL_ALREADY_EXISTS;
  }

  return FINGERPRINT_OK;
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to auto verify
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_AUTO_VERIFY_FAIL</code> failed on verify fingerprint
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::autoVerify(uint8_t securityLevel, uint8_t startPosition, uint8_t searches, bool keyStep, uint8_t maxTries, uint16_t scoreThreshold)
{
  auto waitUntil = millis() + 15000;
  WRITE_CMD_PACKET(FINGERPRINT_AUTO_VERIFY, securityLevel, startPosition, searches, keyStep, maxTries);

  uint8_t times = 1;
  uint16_t score = 0;

  while (times <= maxTries && millis() <= waitUntil)
  {
    auto r = getStructuredPacket(&packet, DEFAULTTIMEOUT /* , (uint32_t)15000 */);

    // Serial.println("Data[0]: 0x" + String(packet.data[0], HEX));
    // Serial.println("Data[1]: 0x" + String(packet.data[1], HEX));
    // Serial.println("Data[2]: 0x" + String(packet.data[2], HEX));
    // Serial.println("Data[3]: 0x" + String(packet.data[3], HEX));
    // Serial.println("Data[4]: 0x" + String(packet.data[4], HEX));
    // Serial.println("Data[5]: 0x" + String(packet.data[5], HEX));
    // Serial.println("Data[6]: 0x" + String(packet.data[6], HEX));
    // Serial.println("Data[7]: 0x" + String(packet.data[7], HEX));

    if (packet.type != FINGERPRINT_ACKPACKET)
      continue;

    if (packet.data[1] == FINGERPRINT_AUTO_VERIFY_SEARCH_STEP)
    {
      if (packet.data[0] == FINGERPRINT_OK)
      {
        score = (packet.data[4] << 8) + packet.data[5];
        break;
      }
      else if (packet.data[0] == FINGERPRINT_AUTO_VERIFY_FAILED_SEARCH)
      {
        break;
      }

      times++;
      continue;
    }
    Serial.println("r: " + String(r, HEX));
    if (r == FINGERPRINT_AWAITED)
    {
      break;
    }
  }

  if (score <= scoreThreshold)
    return FINGERPRINT_AUTO_VERIFY_FAIL;

  /* RESULT_FIND(&fingerID, &confidence); */

  fingerID = (packet.data[2] << 8) | packet.data[3];
  confidence = (packet.data[4] << 8) | packet.data[5];

  return FINGERPRINT_OK;
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to verify
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_NOTFOUND</code> failed on verify fingerprint
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::verify(uint8_t maxTries, uint32_t await)
{
  auto startAt = millis();
  auto awaitUntil = startAt + await;

  uint8_t r = FINGERPRINT_TIMEOUT;
  uint8_t tries = maxTries;

  bool imageCollected, imageConverted = false;

  while (millis() <= awaitUntil && tries > 0)
  {
    mySerial->flush();
    delay(100);
    if (!imageCollected)
    {
      Serial.println("getImage - Taking image...");
      r = getImage();
      if (r != FINGERPRINT_OK)
      {
        Serial.println("getImage - Image not taken. Trying again. Result: " + String(r, HEX));
        continue;
      }
      imageCollected = true;
      Serial.println("getImage - Image Ex taken");
    }

    if (!imageConverted)
    {
      r = image2Tz();

      if (r != FINGERPRINT_OK)
      {
        Serial.println("image2Tz - Image not converted. Trying again. Result: " + String(r, HEX));
      }

      if (r == FINGERPRINT_PACKETRECIEVEERR)
      {
        awaitUntil += (millis() - startAt);
        continue;
      }

      if (r > FINGERPRINT_PACKETRECIEVEERR)
      {
        awaitUntil += (millis() - startAt);
        imageCollected = false;
        continue;
      }

      imageConverted = true;
      Serial.println("image2Tz - Image converted");
    }

    r = fingerSearch();
    if (r == FINGERPRINT_OK)
    {
      Serial.println("fingerSearch - Found !");
      return FINGERPRINT_OK;
    }
    if (r == FINGERPRINT_NOTFOUND)
    {
      tries--;
      awaitUntil += await;
      imageCollected = false;
      imageConverted = false;
    }

    Serial.println("fingerSearch - Not Found - trying again: " + String(tries) + "| Return: 0x" + String(r, HEX));
    continue;
  }

  return FINGERPRINT_TIMEOUT;
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to convert image to feature template
    @param slot Location to place feature template (put one in 1 and another in
   2 for verification to create model)
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_IMAGEMESS</code> if image is too messy
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_FEATUREFAIL</code> on failure to identify
   fingerprint features
    @returns <code>FINGERPRINT_INVALIDIMAGE</code> on failure to identify
   fingerprint features
*/
uint8_t ItThink_Fingerprint::image2Tz(uint8_t slot)
{
  SEND_CMD_PACKET(FINGERPRINT_IMAGE2TZ, slot);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to take two print feature template and create a
   model
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_ENROLLMISMATCH</code> on mismatch of fingerprints
*/
uint8_t ItThink_Fingerprint::createModel(void)
{
  SEND_CMD_PACKET(FINGERPRINT_REGMODEL);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to store the calculated model for later matching
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
   to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t ItThink_Fingerprint::storeModel(uint16_t location)
{
  SEND_CMD_PACKET(FINGERPRINT_STORE, 0x01, (uint8_t)(location >> 8),
                  (uint8_t)(location & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to load a fingerprint model from flash into buffer 1
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t ItThink_Fingerprint::loadModel(uint16_t location)
{
  SEND_CMD_PACKET(FINGERPRINT_LOAD, 0x01, (uint8_t)(location >> 8),
                  (uint8_t)(location & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to transfer 256-byte fingerprint template from the
   buffer to the UART
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t ItThink_Fingerprint::getModel(void)
{
  SEND_CMD_PACKET(FINGERPRINT_UPLOAD, 0x01);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to delete a model in memory
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
   to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t ItThink_Fingerprint::deleteModel(uint16_t location)
{
  SEND_CMD_PACKET(FINGERPRINT_DELETE, (uint8_t)(location >> 8),
                  (uint8_t)(location & 0xFF), 0x00, 0x01);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to delete ALL models in memory
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
   to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t ItThink_Fingerprint::emptyDatabase(void)
{
  SEND_CMD_PACKET(FINGERPRINT_EMPTY);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to search the current slot 1 fingerprint features to
   match saved templates. The matching location is stored in <b>fingerID</b> and
   the matching confidence in <b>confidence</b>
    @returns <code>FINGERPRINT_OK</code> on fingerprint match success
    @returns <code>FINGERPRINT_NOTFOUND</code> no match made
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::fingerFastSearch(void)
{
  // high speed search of slot #1 starting at page 0x0000 and page #0x00A3
  GET_CMD_PACKET(FINGERPRINT_HISPEEDSEARCH, 0x01, 0x00, 0x00, 0x00, 0xA3);
  fingerID = 0xFFFF;
  confidence = 0xFFFF;

  fingerID = packet.data[1];
  fingerID <<= 8;
  fingerID |= packet.data[2];

  confidence = packet.data[3];
  confidence <<= 8;
  confidence |= packet.data[4];

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Control the built in LED
    @param on True if you want LED on, False to turn LED off
    @returns <code>FINGERPRINT_OK</code> on success
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::LEDcontrol(bool on)
{
  if (on)
  {
    SEND_CMD_PACKET(FINGERPRINT_LEDON);
  }
  else
  {
    SEND_CMD_PACKET(FINGERPRINT_LEDOFF);
  }
}

/**************************************************************************/
/*!
    @brief   Control the built in Aura LED (if exists). Check datasheet/manual
    for different colors and control codes available
    @param control The control code (e.g. breathing, full on)
    @param speed How fast to go through the breathing/blinking cycles
    @param coloridx What color to light the indicator
    @param count How many repeats of blinks/breathing cycles
    @returns <code>FINGERPRINT_OK</code> on fingerprint match success
    @returns <code>FINGERPRINT_NOTFOUND</code> no match made
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::LEDcontrol(uint8_t control, uint8_t speed,
                                        uint8_t coloridx, uint8_t count)
{
  SEND_CMD_PACKET(FINGERPRINT_AURALEDCONFIG, control, speed, coloridx, count);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to search the current slot fingerprint features to
   match saved templates. The matching location is stored in <b>fingerID</b> and
   the matching confidence in <b>confidence</b>
   @param slot The slot to use for the print search, defaults to 1
    @returns <code>FINGERPRINT_OK</code> on fingerprint match success
    @returns <code>FINGERPRINT_NOTFOUND</code> no match made
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::fingerSearch(uint8_t slot)
{
  // search of slot starting thru the capacity
  GET_CMD_PACKET(FINGERPRINT_SEARCH, slot, 0x00, 0x00, (uint8_t)(capacity >> 8),
                 (uint8_t)(capacity & 0xFF));

  fingerID = 0xFFFF;
  confidence = 0xFFFF;

  fingerID = packet.data[1];
  fingerID <<= 8;
  fingerID |= packet.data[2];

  confidence = packet.data[3];
  confidence <<= 8;
  confidence |= packet.data[4];

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Ask the sensor for the number of templates stored in memory. The
   number is stored in <b>templateCount</b> on success.
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::getTemplateCount(void)
{
  GET_CMD_PACKET(FINGERPRINT_TEMPLATECOUNT);

  templateCount = packet.data[1];
  templateCount <<= 8;
  templateCount |= packet.data[2];

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Set the password on the sensor (future communication will require
   password verification so don't forget it!!!)
    @param   password 32-bit password code
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::setPassword(uint32_t password)
{
  SEND_CMD_PACKET(FINGERPRINT_SETPASSWORD, (uint8_t)(password >> 24),
                  (uint8_t)(password >> 16), (uint8_t)(password >> 8),
                  (uint8_t)(password & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Writing module registers
    @param   regAdd 8-bit address of register
    @param   value 8-bit value will write to register
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_ADDRESS_ERROR</code> on register address error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::writeRegister(uint8_t regAdd, uint8_t value)
{

  SEND_CMD_PACKET(FINGERPRINT_WRITE_REG, regAdd, value);
}

/**************************************************************************/
/*!
    @brief   Change UART baudrate
    @param   baudrate 8-bit Uart baudrate
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::setBaudRate(uint8_t baudrate)
{

  return (writeRegister(FINGERPRINT_BAUD_REG_ADDR, baudrate));
}

/**************************************************************************/
/*!
    @brief   Change security level
    @param   level 8-bit security level
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::setSecurityLevel(uint8_t level)
{

  return (writeRegister(FINGERPRINT_SECURITY_REG_ADDR, level));
}

/**************************************************************************/
/*!
    @brief   Change packet size
    @param   size 8-bit packet size
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t ItThink_Fingerprint::setPacketSize(uint8_t size)
{

  return (writeRegister(FINGERPRINT_PACKET_REG_ADDR, size));
}
/**************************************************************************/
/*!
    @brief   Helper function to process a packet and send it over UART to the
   sensor
    @param   packet A structure containing the bytes to transmit
*/
/**************************************************************************/

void ItThink_Fingerprint::writeStructuredPacket(
    const ItThink_Fingerprint_Packet &packet)
{

  mySerial->write((uint8_t)(packet.start_code >> 8));
  mySerial->write((uint8_t)(packet.start_code & 0xFF));
  mySerial->write(packet.address[0]);
  mySerial->write(packet.address[1]);
  mySerial->write(packet.address[2]);
  mySerial->write(packet.address[3]);
  mySerial->write(packet.type);

  uint16_t wire_length = packet.length + 2;
  mySerial->write((uint8_t)(wire_length >> 8));
  mySerial->write((uint8_t)(wire_length & 0xFF));

#ifdef FINGERPRINT_DEBUG
  Serial.print("-> 0x");
  Serial.print((uint8_t)(packet.start_code >> 8), HEX);
  Serial.print(", 0x");
  Serial.print((uint8_t)(packet.start_code & 0xFF), HEX);
  Serial.print(", 0x");
  Serial.print(packet.address[0], HEX);
  Serial.print(", 0x");
  Serial.print(packet.address[1], HEX);
  Serial.print(", 0x");
  Serial.print(packet.address[2], HEX);
  Serial.print(", 0x");
  Serial.print(packet.address[3], HEX);
  Serial.print(", 0x");
  Serial.print(packet.type, HEX);
  Serial.print(", 0x");
  Serial.print((uint8_t)(wire_length >> 8), HEX);
  Serial.print(", 0x");
  Serial.print((uint8_t)(wire_length & 0xFF), HEX);
#endif

  uint16_t sum = ((wire_length) >> 8) + ((wire_length) & 0xFF) + packet.type;
  for (uint8_t i = 0; i < packet.length; i++)
  {
    mySerial->write(packet.data[i]);
    sum += packet.data[i];
#ifdef FINGERPRINT_DEBUG
    Serial.print(", 0x");
    Serial.print(packet.data[i], HEX);
#endif
  }

  mySerial->write((uint8_t)(sum >> 8));
  mySerial->write((uint8_t)(sum & 0xFF));

#ifdef FINGERPRINT_DEBUG
  Serial.print(", 0x");
  Serial.print((uint8_t)(sum >> 8), HEX);
  Serial.print(", 0x");
  Serial.println((uint8_t)(sum & 0xFF), HEX);
#endif

  return;
}

/**************************************************************************/
/*!
    @brief   Helper function to receive data over UART from the sensor and
   process it into a packet
    @param   packet A structure containing the bytes received
    @param   timeout how many milliseconds we're willing to wait
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_TIMEOUT</code> or
   <code>FINGERPRINT_BADPACKET</code> on failure
*/
/**************************************************************************/
uint8_t
ItThink_Fingerprint::getStructuredPacket(ItThink_Fingerprint_Packet *packet,
                                         uint16_t timeout, uint32_t await)
{
  uint8_t byte;
  uint16_t idx = 0, timerTimeout = 0;

  auto awaitUntil = millis() + await;

#ifdef FINGERPRINT_DEBUG
  Serial.printf("(timeout: %u ms |  await: %u | awaitUntil: %u) <- ", timeout, await, awaitUntil);
#endif

  while (!await || millis() <= awaitUntil)
  {
    while (!mySerial->available())
    {
      delay(1);
      timerTimeout++;
      if (timerTimeout >= timeout)
      {
        if (!await)
        {
#ifdef FINGERPRINT_DEBUG
          Serial.println("Timed out");
#endif
          return FINGERPRINT_TIMEOUT;
        }
      }
    }
    // timerTimeout = 0;
    byte = mySerial->read();
#ifdef FINGERPRINT_DEBUG
    Serial.print("0x");
    Serial.print(byte, HEX);
    Serial.print(", ");
#endif
    switch (idx)
    {
    case 0:
      if (byte != (FINGERPRINT_STARTCODE >> 8))
        continue;
      packet->start_code = (uint16_t)byte << 8;
      break;
    case 1:
      packet->start_code |= byte;
      if (packet->start_code != FINGERPRINT_STARTCODE)
      {
        if (await)
        {
          packet->start_code = 0;
          idx = -1;
          break;
        }

        return FINGERPRINT_BADPACKET;
      }

      break;
    case 2:
    case 3:
    case 4:
    case 5:
      packet->address[idx - 2] = byte;
      break;
    case 6:
      packet->type = byte;
      break;
    case 7:
      packet->length = (uint16_t)byte << 8;
      break;
    case 8:
      packet->length |= byte;
      break;
    default:
      packet->data[idx - 9] = byte;
      if ((idx - 8) == packet->length)
      {
#ifdef FINGERPRINT_DEBUG
        Serial.println(" OK ");
#endif
        return FINGERPRINT_OK;
      }
      break;
    }
    idx++;
    if ((idx + 9) >= sizeof(packet->data))
    {
      return FINGERPRINT_BADPACKET;
    }
  }

#ifdef FINGERPRINT_DEBUG
  Serial.println("Awaited");
#endif
  return FINGERPRINT_AWAITED;
}