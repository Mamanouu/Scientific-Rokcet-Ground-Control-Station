#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Create an RF24 object
RF24 radio(27, 5); // CE, CSN pins
#define LEDPIN 2

// Address for communication between modules
const byte address[6] = "00001";

// Buffers for storing the incoming messages
char receivedText[32] = "";
char receivedText2[32] = "";
char receivedText3[32] = "";

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX); // Power level
  radio.startListening(); // Set as receiver
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
}

void loop() {
  // Check if data is available to read
  if (radio.available()) {
    // Buffer for the current incoming message
    char currentMessage[32] = "";
    radio.read(&currentMessage, sizeof(currentMessage));

    // Process based on the first character to identify which message was received
    if (currentMessage[0] == '@') {
      memmove(currentMessage, currentMessage + 1, strlen(currentMessage));
      strcpy(receivedText, currentMessage);  // Store message 1
    } 
    
    else if (currentMessage[0] == '&') {
      memmove(currentMessage, currentMessage + 1, strlen(currentMessage));
      strcpy(receivedText2, currentMessage); // Store message 2
    } 
    
    else if (currentMessage[0] == '#') {
      memmove(currentMessage, currentMessage + 1, strlen(currentMessage));
      strcpy(receivedText3, currentMessage); // Store message 3
    }

    // Once all messages are received, print them in order
    if (strlen(receivedText) > 0 && strlen(receivedText2) > 0 && strlen(receivedText3) > 0) {
      Serial.print(receivedText);
      Serial.print(receivedText2);
      Serial.println(receivedText3);

      // Reset the messages for the next set of transmissions
      memset(receivedText, 0, sizeof(receivedText));
      memset(receivedText2, 0, sizeof(receivedText2));
      memset(receivedText3, 0, sizeof(receivedText3));
    }
 }
}
