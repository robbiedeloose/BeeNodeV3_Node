#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTHEX(x) Serial.print(x, HEX)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

#include <arduino.h>

#include <DallasTemperature.h> // for ds18
#include <EEPROM.h>            //EEPROM functions
#include <OneWire.h>           //for ds18
#include <RF24.h> //Library for nRF24L01, using version https://github.com/TMRh20/RF24
#include <RF24Network.h> //Library for networking nRF24L01s, using version https://github.com/TMRh20/RF24Network
#include <SPI.h>         //nRF24L01 uses SPI communication

// node specific declarations
#define ledPin 13
#define configButton 2
bool config = FALSE;
byte nodeId[4];
byte nodeAddress = 1;
float iREF = 1.1;

// One wire definitions
#define ONE_WIRE_BUS A0
#define TEMPERATURE_PRECISION 10 // Lower resolution
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress; // temp addresses
int const numberOfSensors = 3;
byte deviceAddress[numberOfSensors][8]; // address aray
int foundDevices = 0;                   // Number of temperature devices found
float temperatures[numberOfSensors];

// NODE ID in EEPRom - 4 Bytes /////////////////////////////////////////////////
void getNodeId() {
  // char firstByte = EEPROM.read(1);
  DEBUG_PRINT("First byte: ");
  DEBUG_PRINTLN(EEPROM.read(1));
  if (EEPROM.read(1) == '#') {
    DEBUG_PRINT("Node id: ");
    for (int i = 0; i < 4; i++) {
      nodeId[i] = EEPROM.read(i + 2);
      DEBUG_PRINTHEX(nodeId[i]);
    }
    Serial.println();
  } else {
    DEBUG_PRINTLN("Address not set. Generating code...");
    randomSeed(analogRead(A0));
    for (int i = 0; i < 4; i++) {
      nodeId[i] = random(255);
      EEPROM.write(i + 2, nodeId[i]);
      DEBUG_PRINTHEX(nodeId[i]);
    }
    DEBUG_PRINTLN();
    EEPROM.write(1, '#');
  }
}

// NODE ADDRESS from pin headers ///////////////////////////////////////////////
void getNodeAdress() {
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  nodeAddress = 0;

  if (digitalRead(5) == LOW) {
    nodeAddress = nodeAddress + 1;
  }
  if (digitalRead(6) == LOW) {
    nodeAddress = nodeAddress + 2;
  }
  if (digitalRead(7) == LOW) {
    nodeAddress = nodeAddress + 3;
  }

  if (nodeAddress == 0)
    nodeAddress = 1;

  Serial.print("Node Address: ");
  Serial.println(nodeAddress);
}

// DS28B20 ADDRESS FUNCTIONS ///////////////////////////////////////////////////
void loadDS18Addresses() {
  int startByte = 6;
  for (int a = 0; a < numberOfSensors; a++) {
    for (int i = 0; i < 8; i++) {
      deviceAddress[a][i] = EEPROM.read(i + startByte + (8 * a));
    }
  }
}

void clearDS18Addresses() {
  int startByte = 6;
  for (int a = 0; a < numberOfSensors; a++) {
    for (int i = 0; i < 8; i++) {
      EEPROM.write(i + startByte + (8 * a), 0xFF);
    }
  }
}

void printDS18AddressesArray() {
  for (int a = 0; a < numberOfSensors; a++) {
    for (int i = 0; i < 8; i++) {
      Serial.print(deviceAddress[a][i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// function to print a device address
void printDS18Addresses() {
  int startByte = 6;
  Serial.println("-----------------------");
  for (int a = 0; a < numberOfSensors; a++) {
    for (int i = 0; i < 8; i++) {
      Serial.print(EEPROM.read(i + startByte + (8 * a)), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("-----------------------");
}

bool checkAndSaveDS18Address(DeviceAddress tempAddress) {
  int startByte = 6;
  int b = 0;
  for (int a = 0; a < numberOfSensors; a++) {
    DEBUG_PRINT("Checking address slot ");
    DEBUG_PRINTLN(a);
    // is next address empty? - No
    if (EEPROM.read(startByte + (8 * a)) != 0xFF) {
      // check new address against saved address
      for (int i = 0; i < 8; i++) {
        // check 8 bytes
        if (tempAddress[i] == (EEPROM.read(i + startByte + (8 * a)))) {
          b++;
          if (b == 8) { // if byte matches 8 times, return
            DEBUG_PRINTLN("Allready exists");
            return false;
          }
        } else // byte differs, continue to next address slot
        {
          DEBUG_PRINTLN("Slot is taken");
          break;
        }
      }
    }
    // is next address empty? - yes
    else {
      // save address
      // dim led to signal that an address is beeing saved
      digitalWrite(ledPin, LOW);
      DEBUG_PRINTLN("Saving...");
      for (int i = 0; i < 8; i++) {
        EEPROM.write(i + startByte + (8 * a), tempAddress[i]);
      }
      delay(1000);
      // turn led back on
      digitalWrite(ledPin, HIGH);
      // return true to signal adress is saved
      return true;
    }
  }
  return false;
}

void getTemperatures() {
  sensors.requestTemperatures(); // Send the command to get temperatures
  delay(200);
  for (int a = 0; a < numberOfSensors; a++) {
    // delay(200);
    temperatures[a] = sensors.getTempC(deviceAddress[a]);
    Serial.print("Sensor ");
    Serial.print(a + 1);
    Serial.print(": ");
    Serial.println(temperatures[a]);
  }
}

// CONFIG MODE /////////////////////////////////////////////////////////////////
// Check config button is low and got inro config mode
void checkConfigButton() {
  if (digitalRead(configButton) == LOW) {
    if (!config) {
      DEBUG_PRINTLN("reseting addresses");
      clearDS18Addresses();
      foundDevices = 0;
    }
    config = TRUE;
  } else {
    if (config) {
      DEBUG_PRINTLN("loading addresses");
      loadDS18Addresses();
    }
    config = FALSE;
  }
}

// config mode enables saving of addresses to eeprom
void runConfig() {
  digitalWrite(ledPin, HIGH);
  sensors.begin();
  foundDevices = sensors.getDeviceCount();
  for (int i = 0; i < foundDevices; i++) {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i)) {
      if (checkAndSaveDS18Address(tempDeviceAddress))
        printDS18Addresses();
    }
  }
  delay(500);
  digitalWrite(ledPin, LOW);
}

// BATTERY MEASUREMENT /////////////////////////////////////////////////////////

// This function makes 8 ADC measurements but does nothing with them
// Since after a reference change the ADC can return bad readings. This function
// is used to get rid of the first
// 8 readings to ensure next reading is accurate
void burn8Readings() {
  for (int i = 0; i < 8; i++) {
    analogRead(A0);
    analogRead(A1);
  }
}

// This function uses the known internal reference value of the 328p (~1.1V) to
// calculate the VCC value which comes from a battery
// This was leveraged from a great tutorial found at
// https://code.google.com/p/tinkerit/wiki/SecretVoltmeter?pageId=110412607001051797704
float fReadVcc() {
  analogReference(EXTERNAL); // set the ADC reference to AVCC
  burn8Readings(); // make 8 readings but don't use them to ensure good reading
                   // after ADC reference change
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  unsigned long start = millis(); // get timer value
  while ((start + 3) > millis())
    ;                  // delay for 3 milliseconds
  ADCSRA |= _BV(ADSC); // Start ADC conversion
  while (bit_is_set(ADCSRA, ADSC))
    ;                  // wait until conversion is complete
  int result = ADCL;   // get first half of result
  result |= ADCH << 8; // get rest of the result
  float batVolt =
      (iREF / result) * 1024; // Use the known iRef to calculate battery voltage
  analogReference(INTERNAL);  // set the ADC reference back to internal
  burn8Readings(); // make 8 readings but don't use them to ensure good reading
                   // after ADC reference change
  return batVolt;
}

// INIT FUNCTIONS //////////////////////////////////////////////////////////////

void initNode() {
  // Set pinmodes for config mode
  pinMode(configButton, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  // Get ID and Address
  getNodeId();
  getNodeAdress();
  // Set voltage reference
  analogReference(INTERNAL); // set the ADC reference to internal 1.1V reference
  burn8Readings(); // make 8 readings but don't use them to ensure good reading
                   // after ADC reference change
}

void initTempSensors() {
  loadDS18Addresses();
  sensors.begin();
  sensors.setResolution(10);
  printDS18AddressesArray();
}

////////////////////////////////////////////////////////////////////////////////
// SETUP AND LOOP //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(38400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  initNode();
  initTempSensors();
  Serial.println("-addresses in eeprom---------------------------------------");
  printDS18Addresses();
  Serial.println("-addressess in array---------------------------------------");
  printDS18AddressesArray();
  Serial.println("----------------------------------------");
}

void loop() {
  // put your main code here, to run repeatedly:
  checkConfigButton();

  if (!config) {
    // do stuf
    Serial.print("battery voltage: ");
    Serial.println(fReadVcc());
    // float a = checkBatteryVolt();
    getTemperatures();
    delay(2000);
  } else {
    runConfig();
  }
}
