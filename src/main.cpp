
#include <DallasTemperature.h> // for ds18
#include <EEPROM.h>            //EEPROM functions
#include <OneWire.h>           //for ds18
#include <RF24.h> //Library for nRF24L01, using version https://github.com/TMRh20/RF24
#include <RF24Network.h> //Library for networking nRF24L01s, using version https://github.com/TMRh20/RF24Network
#include <SPI.h>         //nRF24L01 uses SPI communication

byte nodeId[4];
byte nodeAddress = 1;
float iREF = 1.1;

void getNodeId() {
  // char firstByte = EEPROM.read(1);
  Serial.print("First byte: ");
  Serial.println(EEPROM.read(1));
  if (EEPROM.read(1) == '#') {
    Serial.print("Node id: ");
    for (int i = 0; i < 4; i++) {
      nodeId[i] = EEPROM.read(i + 2);
      Serial.print(nodeId[i], HEX);
    }
    Serial.println();
  } else {
    Serial.println("Address not set. Generating code...");
    randomSeed(analogRead(A0));
    for (int i = 0; i < 4; i++) {
      nodeId[i] = random(255);
      EEPROM.write(i + 2, nodeId[i]);
      Serial.print(nodeId[i], HEX);
    }
    Serial.println();
    EEPROM.write(1, '#');
  }
}

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
  Serial.print("Node Address: ");
  Serial.println(nodeAddress);
}

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

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  getNodeId();
  getNodeAdress();

  analogReference(INTERNAL); // set the ADC reference to internal 1.1V reference
  burn8Readings(); // make 8 readings but don't use them to ensure good reading
                   // after ADC reference change
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("battery voltage: ");
  Serial.println(fReadVcc());
  // float a = checkBatteryVolt();
  delay(2000);
}
