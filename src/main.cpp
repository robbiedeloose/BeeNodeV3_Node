
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

#include "BeeNode.h"

#include <RF24.h> //Library for nRF24L01, using version https://github.com/TMRh20/RF24
#include <RF24Network.h> //Library for networking nRF24L01s, using version https://github.com/TMRh20/RF24Network
#include <SPI.h>         //nRF24L01 uses SPI communication

// node specific declarations
#define ledPin 13
#define configButton 2
NodeData_t beeNodeData;

// EEPROM address locations
#include <EEPROM.h>                   //EEPROM functions
#define EEPRomDeviceId 1              // 1 byte for #, 4 bytes for ID
#define EEPRomDs18Ids 6               // 48 bytes for 6 addresses, 8 bytes each
bool firstTimeThroughSaveLoop = true; // remove line above
uint8_t nodeAddress = 1;

// own libraries //
#include "RandomNodeId.h"
RandomNodeId beeNodeId;
uint8_t nodeId[4];
#include "MesureVoltageInternal.h"
float iREF = 1.1;
MesureVoltageInternal battery(iREF);

// One wire definitions //
#include <DallasTemperature.h>
#include <OneWire.h>
#define ONE_WIRE_BUS A0
#define TEMPERATURE_PRECISION 10 // 9, 10 - adjust wait time accordingly
int waitForConversion = 200;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress; // temporary addresses to save found sensor to
int const numberOfSensors = 3;
uint8_t deviceAddress[numberOfSensors][8]; // address aray
int foundDevices = 0; // Number of temperature devices found
float temperatures[numberOfSensors];

// NODE ADDRESS from pin headers ///////////////////////////////////////////////
uint8_t getNodeAdress() {
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  uint8_t address = 0;

  if (digitalRead(5) == LOW) {
    address = address + 1;
  }
  if (digitalRead(6) == LOW) {
    address = address + 2;
  }
  if (digitalRead(7) == LOW) {
    address = address + 3;
  }
  if (nodeAddress == 0)
    address = 1;

  Serial.print("Node Address: ");
  Serial.println(nodeAddress);
  return address;
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

void printLoadedDS18Addresses() {
  for (int a = 0; a < numberOfSensors; a++) {
    for (int i = 0; i < 8; i++) {
      Serial.print(deviceAddress[a][i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// function to print a device address
void printSavedDS18Addresses() {
  int startByte = 6;
  for (int a = 0; a < numberOfSensors; a++) {
    for (int i = 0; i < 8; i++) {
      Serial.print(EEPROM.read(i + startByte + (8 * a)), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void getTemperatures(NodeData_t *nodeData) {
  sensors.requestTemperatures(); // Send the command to get temperatures
  delay(waitForConversion);
  for (int a = 0; a < numberOfSensors; a++) {
    // delay(200);
    temperatures[a] = sensors.getTempC(deviceAddress[a]);
    nodeData->hiveTemps[a] = temperatures[a];
    Serial.print("Sensor ");
    Serial.print(a + 1);
    Serial.print(": ");
    Serial.println(temperatures[a]);
  }
}

bool checkAndSaveDS18Address(DeviceAddress tempAddress) {
  // This function gets an address that was found. It checks every used EEPRom
  // slot if this address allready exists and saves it in the first free slot if
  // not.
  // should start address in eeprom be passedto function? Is this the only place
  // it exists? --> No also exists in loas addresses and clear addresses.
  int startByte = EEPRomDs18Ids;
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

// config mode enables saving of addresses to eeprom
void lookForNewDS18Sensors() {
  digitalWrite(ledPin, HIGH); // keep led high while we look for new sensors
  // clear the addresses if you go through scan for first time
  if (firstTimeThroughSaveLoop)
    clearDS18Addresses();

  sensors.begin();
  foundDevices = sensors.getDeviceCount();
  for (int i = 0; i < foundDevices; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      if (checkAndSaveDS18Address(tempDeviceAddress)) // if found address is
                                                      // saved, print the
                                                      // addresses that are
                                                      // allready saved
        // Address is saved
        // Possible to return true here as wel to put print the saved data from
        // the calling function --
        printSavedDS18Addresses();
    }
  }
  delay(500);
  firstTimeThroughSaveLoop = false;
  digitalWrite(ledPin, LOW); // turn led low if we leave the function
}

// INIT FUNCTIONS //////////////////////////////////////////////////////////////

void initNode() {
  // Set pinmodes for config mode
  pinMode(configButton, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  // Get ID and Address
  getNodeAdress();
  // Set voltage reference
  analogReference(INTERNAL); // set the ADC reference to internal 1.1V reference
  // burn8Readings(); // make 8 readings but don't use them to ensure good
  // reading
  // after ADC reference change
}

void initTempSensors() {
  loadDS18Addresses();
  sensors.begin();
  sensors.setResolution(10);
  printLoadedDS18Addresses();
}

void set_temp_struct(NodeData_t *nodeData) {
  nodeData->hiveTemps[0] = 1;
  nodeData->hiveTemps[1] = 2;
  nodeData->hiveTemps[2] = 3;
}

void print_temp_struct(NodeData_t *nodeData) {
  Serial.print("struct temp 1:");
  Serial.println(nodeData->hiveTemps[0]);
  Serial.print("struct temp 2:");
  Serial.println(nodeData->hiveTemps[1]);
  Serial.print("struct temp 3:");
  Serial.println(nodeData->hiveTemps[2]);
}
////////////////////////////////////////////////////////////////////////////////
// SETUP AND LOOP //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(38400);
#ifdef DEBUG
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  /*
  This data should be printed when started or if we go in and out of config mode

  BeeNode v3.0.1a Node (or gateway or coordinator)
  -----------------------------------------
  NodeId: 56F7E790
  NodeAddress: 1
  -----------------------------------------
  DS18B20 sensors
  number of sensors: 3
  id 1: 56F7E79056F7E790
  id 2: 56F7E79056F7E790
  id 3: 56F7E79056F7E790
  -----------------------------------------
  humidity sensor
  present: false
  -----------------------------------------
  Scale sensor
  present: FALSE
  -----------------------------------------
  Vref: 1.1
  -----------------------------------------
  change settings? y/n

  */
  // byte someByte[4];
  beeNodeId.getId(nodeId);
  Serial.println("DeviceId:");
  for (byte b : nodeId)
    Serial.print(b, HEX);
  Serial.println();
  delay(5000);

  initNode();
  initTempSensors();
  Serial.println("-addresses in eeprom---------------------------------------");
  printSavedDS18Addresses();
  Serial.println("-addressess in array---------------------------------------");
  printLoadedDS18Addresses();
  Serial.println("----------------------------------------");
}

void loop() {

  // check if button is pressed. if so loop tgrough lookForNewDS18Sensors
  while (digitalRead(configButton) == LOW)
    lookForNewDS18Sensors();
  // load sensors if we went through search loop and quit
  if (!firstTimeThroughSaveLoop)
    loadDS18Addresses();
  firstTimeThroughSaveLoop = true;

  // measure and send
  Serial.print("battery voltage: ");
  Serial.println(battery.getVoltage());
  getTemperatures(&beeNodeData);
  Serial.println();
  Serial.println("---------------------------------------------------------");
  print_temp_struct(&beeNodeData);
  Serial.println("---------------------------------------------------------");
  delay(2000);
}
