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

// Pins
#define ledPin 6 // might change to buzzerPin
#define configButton 4
#define radioPin1 7
#define radioPin2 8
#define addressPin1 A1
#define addressPin2 A2
#define addressPin3 A3
#define alarmPin 3
#define seedRef A0
#define oneWirePin 5

// node specific declarations //
// NodeData_t beeNodeData; --------------------
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

// Low Power
#include <LowPower.h>
int sleepInterval = 10;
int sleepcycle = 2;

// One wire definitions //
#include <DallasTemperature.h>
#include <OneWire.h>
// #define ONE_WIRE_BUS A0
int tempResolution = 10; // 9, 10 - adjust wait time accordingly
int waitForConversion = 200;
OneWire oneWire(oneWirePin);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress; // temporary addresses to save found sensor to
#define numberOfSensors 3
uint8_t deviceAddresses[numberOfSensors][8]; // address aray
int foundDevices = 0; // Number of temperature devices found
float temperatures[numberOfSensors];
bool firstTimeThroughSaveLoop = true; // remove line above

// RF24 declarations //
RF24 radio(7, 8); // create object to control and communicate with nRF24L01
RF24Network network(radio); // Create object to use nRF24L01 in mesh network
const uint16_t rXNode = 00; // address of coordinator

// structure used to hold the payload that is sent to the coordinator.
struct payload_t {
  uint8_t id[4];
  float temp[numberOfSensors];
  float bat;
};
payload_t payload; // Payload to send

// RF25 Radio CODE
// /////////////////////////////////////////////////////////////////////
void collectData(payload_t *payloadAddress) {
  for (uint8_t i = 0; i < 4; i++) // fill nodeId
    payloadAddress->id[i] = nodeId[i];
  sensors.requestTemperatures(); // Send the command to get temperatures
  delay(waitForConversion);
  for (uint8_t a = 0; a < numberOfSensors; a++) // temperature values
    payloadAddress->temp[a] = sensors.getTempC(deviceAddresses[a]);
  payloadAddress->bat = battery.getVoltage(); // Battery
}

void showData(payload_t *payloadAddress) {
  Serial.print("Device Id: "); // Show devide id
  for (uint8_t i = 0; i < 4; i++)
    Serial.print(nodeId[i]);
  Serial.println();
  Serial.print("Device Address: ");
  Serial.println(nodeAddress); // Show device address
  Serial.println("DS18b20 temp sensors");
  for (int i; i < numberOfSensors; i++) { // Show DS sensors data
    Serial.print("temp : ");
    Serial.print(i + 1);
    Serial.print(": ");
    if (payloadAddress->temp[i] != -127.00)
      Serial.println(payloadAddress->temp[i]);
    else
      Serial.println("not present");
  }
  Serial.print("Battery: ");
  Serial.println(payloadAddress->bat); // Show battery info
  Serial.println();
}

bool sendData(payload_t *payloadAddress,
              int sizeOfPayload) { //<---------------------------- should edit
                                   // it to accept
  // pointer to struct as well, probably ad it with & in function
  // header en remove the & within the function
  network.update(); // check to see if there is any network traffic that needs
                    // to be passed on, technically an end device does not need
                    // this
  RF24NetworkHeader header(rXNode); // Create transmit header. This goes in
                                    // transmit packet to help route it where it
                                    // needs to go, in this case it is the
                                    // coordinator
  // send data onto network and make sure it gets there1
  if (network.write(header, payloadAddress, sizeOfPayload))
    return true;
  else
    return false;
}

// NODE ADDRESS from pin headers ///////////////////////////////////////////////
uint8_t getNodeAdress() {
  pinMode(addressPin1, INPUT_PULLUP);
  pinMode(addressPin2, INPUT_PULLUP);
  pinMode(addressPin3, INPUT_PULLUP);
  uint8_t address = 0;
  if (digitalRead(addressPin1) == LOW) {
    address = address + 1;
  }
  if (digitalRead(addressPin2) == LOW) {
    address = address + 2;
  }
  if (digitalRead(addressPin3) == LOW) {
    address = address + 3;
  }
  if (address == 0)
    address = 1;
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
    Serial.print("Id ");
    Serial.print(a + 1);
    Serial.print(": ");
    for (int i = 0; i < 8; i++) {
      Serial.print(EEPROM.read(i + startByte + (8 * a)), HEX);
      Serial.print(" ");
    }
    Serial.println();
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
  beeNodeId.getId(nodeId); // send array to fill as parameter
  nodeAddress = getNodeAdress();
  // Set voltage reference
  battery.setRefInternal();
}

void initTempSensors() {
  loadDS18Addresses();
  sensors.begin();
  sensors.setResolution(tempResolution);
}

void initRadio() { //
  SPI.begin();     // Start SPI communication
  radio.begin();   // start nRF24L01 communication and control
  // radio.setRetries(0,0);
  // setup network communication, first argument is channel which determines
  // frequency band module communicates on. Second argument is address of this
  // module
  network.begin(90, nodeAddress);
}

bool checkResponse() {
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == 'y') {
      Serial.println("YES");
      return true;
    } else {
      Serial.println("NO");
      return false;
    }
  }
  return false;
}

bool getAnswer() {
   while (!Serial.available()) { }
   if(Serial.read() == 'Y') return true;
   else return false;
}

void getNewSettings()
{
  Serial.println(F("Change DeviceAddress? y/n"));
  if(getAnswer()){;}
}
  ////////////////////////////////////////////////////////////////////////////////
  void printConfig() {
    Serial.println("BeeNode v3.0.1a Node");
    Serial.println("-----------------------------------------");
    Serial.print("Node Id: ");
    for (byte b : nodeId)
      Serial.print(b, HEX);
    Serial.println();
    Serial.print("Node Address: ");
    Serial.println(nodeAddress);
    Serial.println("-----------------------------------------");
    Serial.println("DS18B20 Sensors");
    Serial.print("Number of sensors: ");
    Serial.println(numberOfSensors);
    printSavedDS18Addresses();
    Serial.println("-----------------------------------------");
    Serial.print("Vref: ");
    Serial.println(iREF);
    Serial.println("-----------------------------------------");
    Serial.println("Interval: not set");
    Serial.println("-----------------------------------------");
    Serial.println("Humidity sensor");
    Serial.println("Present: false");
    Serial.println("-----------------------------------------");
    Serial.println("Scale sensor");
    Serial.println("Present: false");
    Serial.println("-----------------------------------------");
    Serial.print("Change setting? y/n");
    for (int i = 0; i < 20; i++) {
      Serial.print(".");
      if (checkResponse() == true){
        getNewSettings();
        break;
        }
      delay(500);
    }
    Serial.println();
    Serial.println();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // SETUP AND LOOP
  // //////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  void setup() {
    Serial.begin(38400);
#ifdef DEBUG
    while (!Serial) {
      ; // wait for serial port not to miss data when starting serial monitor
      // leave out when debug is not defined
    }
#endif
    initNode();        // start node
    initTempSensors(); // load and start temp sensors
    // initOtherThins - I2C sensors,scale, ...
    initRadio(); // start radio
    // load all setting if needed
    // - not for now
    // print settings data
    printConfig();
  }

  void loop() {
    // Config //////////////////////////////////////////////////////////////
    // check if button is pressed. if so loop tgrough lookForNewDS18Sensors
    while (digitalRead(configButton) == LOW)
      lookForNewDS18Sensors();
    // load sensors if we went through search loop and quit
    if (!firstTimeThroughSaveLoop) {
      loadDS18Addresses();
      printConfig();
      firstTimeThroughSaveLoop = true;
    }

    // measure and send ////////////////////////////////////////////////////
    // fill struct
    collectData(&payload);
    // display data in struct
    showData(&payload);
    // send struct
    radio.powerUp();
    bool sendSuccesfull = sendData(&payload, sizeof(payload));
    radio.powerDown();
// display send result
#ifdef DEBUG
    if (sendSuccesfull)
      DEBUG_PRINTLN("Send succesfull");
    else
      DEBUG_PRINTLN("Send error!");
    DEBUG_PRINTLN();
#endif
    // sleep to be implemented
    for (int i; i < sleepcycle; i++)
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
