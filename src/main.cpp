
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

// node specific declarations //
#define ledPin 5
#define configButton 4
NodeData_t beeNodeData;
uint8_t nodeAddress = 1;

// EEPROM address locations //
#include <EEPROM.h>      //EEPROM functions
#define EEPRomDeviceId 1 // 1 byte for #, 4 bytes for ID
#define EEPRomDs18Ids 6  // 48 bytes for 6 addresses, 8 bytes each
uint8_t nodeId[4];

// own libraries //
#include "RandomNodeId.h"
RandomNodeId beeNodeId;

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
#define numberOfSensors 3
uint8_t deviceAddresses[numberOfSensors][8]; // address aray
int foundDevices = 0; // Number of temperature devices found
float temperatures[numberOfSensors];
bool firstTimeThroughSaveLoop = true; // remove line above

// RF CODE /////////////////////////////////////////////////////////////////////
RF24 radio(7, 8); // create object to control and communicate with nRF24L01
RF24Network network(radio); // Create object to use nRF24L01 in mesh network
const uint16_t rXNode = 00; // address of coordinator
// structure used to hold the payload that is sent to the coordinator.
// Currently setup to send temp value as float, ADC value as int, and battery
// state as bool
// Can be adjusted to accomodate the exact data you want node to send
// Keep in mind the more data you send the more power is used and the more
// likely
// there is some kind of data error in the transmit receive process
struct payload_t {
  float aDCTemp; // temperature from onboard sensor
  bool batState; // bool to communicate battery power level, true is good and
                 // false means battery needs to be replaced
};
struct payload2_t {
  uint8_t id[4];
  float temp[numberOfSensors]; // temperature from onboard sensor
  float bat;
};

void fillPayload(payload2_t *payload) {
  for (uint8_t i = 0; i < 4; i++)
    payload->id[i] = nodeId[i];
  for (uint8_t i = 0; i < numberOfSensors; i++)
    payload->temp[i] = temperatures[i];
  payload->bat = 1.2345; // battery.getVoltage();
}

void setupRadio() {
  SPI.begin();   // Start SPI communication
  radio.begin(); // start nRF24L01 communication and control
  // radio.setRetries(0,0);
  // setup network communication, first argument is channel which determines
  // frequency band module communicates on. Second argument is address of this
  // module
  network.begin(90, nodeAddress);
}

void sendStuff() {
  network.update(); // check to see if there is any network traffic that needs
                    // to be passed on, technically an end device does not need
                    // this
  payload_t payload = {temperatures[0], true};
  payload2_t payload2;
  fillPayload(&payload2);
  //

  Serial.print("Battery: ");
  Serial.println(payload2.bat);
  Serial.print("temp 1: ");
  Serial.println(payload2.temp[0]);

  //
  RF24NetworkHeader header(rXNode); // Create transmit header. This goes in
                                    // transmit packet to help route it where it
                                    // needs to go, in this case it is the
                                    // coordinator
  // send data onto network and make sure it gets there1
  if (network.write(header, &payload2, sizeof(payload2)))
    Serial.println("Succes!");
  else
    Serial.println("Failed!");
}

// NODE ADDRESS from pin headers ///////////////////////////////////////////////
uint8_t getNodeAdress() {
  // pinMode(5, INPUT_PULLUP);
  // pinMode(6, INPUT_PULLUP);
  // pinMode(7, INPUT_PULLUP);

  uint8_t address = 0;
  /*
    if (digitalRead(5) == LOW) {
      address = address + 1;
    }
    if (digitalRead(6) == LOW) {
      address = address + 2;
    }
    if (digitalRead(7) == LOW) {
      address = address + 3;
    }*/
  if (address == 0)
    address = 1;

  Serial.print("Node Address: ");
  Serial.println(nodeAddress);
  return address;
}

// DS28B20 ADDRESS FUNCTIONS ///////////////////////////////////////////////////
void loadDS18Addresses() {
  int startByte = EEPRomDs18Ids;
  for (uint8_t a = 0; a < numberOfSensors; a++) {
    for (uint8_t i = 0; i < 8; i++) {
      deviceAddresses[a][i] = EEPROM.read(i + startByte + (8 * a));
    }
  }
}

void clearDS18Addresses() {
  int startByte = EEPRomDs18Ids;
  for (int a = 0; a < numberOfSensors; a++) {
    for (int i = 0; i < 8; i++) {
      EEPROM.write(i + startByte + (8 * a), 0xFF);
    }
  }
}

void printLoadedDS18Addresses() {
  for (int a = 0; a < numberOfSensors; a++) {
    for (int i = 0; i < 8; i++) {
      Serial.print(deviceAddresses[a][i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// function to print a device address
void printSavedDS18Addresses() {
  int startByte = EEPRomDs18Ids;
  for (int a = 0; a < numberOfSensors; a++) {
    for (int i = 0; i < 8; i++) {
      Serial.print(EEPROM.read(i + startByte + (8 * a)), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void getTemperatures(/*NodeData_t *nodeData*/) {
  sensors.requestTemperatures(); // Send the command to get temperatures
  delay(waitForConversion);
  for (int a = 0; a < numberOfSensors; a++) {
    // delay(200);
    temperatures[a] = sensors.getTempC(deviceAddresses[a]);
    // nodeData->hiveTemps[a] = temperatures[a];
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

void initTempSensors() {
  loadDS18Addresses();
  sensors.begin();
  sensors.setResolution(10);
  printLoadedDS18Addresses();
}

void initNode() {
  // Set pinmodes for config mode
  pinMode(configButton, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  // Get ID and Address
  beeNodeId.getId(nodeId); // send array to fill as parameter
  nodeAddress = getNodeAdress();
  // Set voltage reference
  battery.setRefInternal();
  //
  initTempSensors();
  loadDS18Addresses(); // send array to fill as parameter
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
void printConfig() {
  Serial.println("DeviceId:");
  for (byte b : nodeId)
    Serial.print(b, HEX);
  Serial.println();
  delay(5000);
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
}

////////////////////////////////////////////////////////////////////////////////
// SETUP AND LOOP //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(38400);
#ifdef DEBUG
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    // leave out when debug is not defined
  }
#endif

  initNode();
  initTempSensors();
  // Serial.println("-addresses in
  // eeprom---------------------------------------");
  printSavedDS18Addresses();
  // Serial.println("-addressess in
  // array---------------------------------------");
  printLoadedDS18Addresses();
  Serial.println("----------------------------------------");
  setupRadio();
}

void loop() {
  // check if button is pressed. if so loop tgrough lookForNewDS18Sensors
  while (digitalRead(configButton) == LOW)
    lookForNewDS18Sensors();
  // load sensors if we went through search loop and quit
  if (!firstTimeThroughSaveLoop) {
    loadDS18Addresses();
    printConfig();
    firstTimeThroughSaveLoop = true;
  }

  // measure and send
  Serial.print("battery voltage: ");
  Serial.println(battery.getVoltage());
  getTemperatures();
  /*  getTemperatures(&beeNodeData); // Struct?
  Serial.println();
  Serial.println("---------------------------------------------------------");
  print_temp_struct(&beeNodeData);
  Serial.println("---------------------------------------------------------");
  */
  sendStuff();
  delay(2000);
}
