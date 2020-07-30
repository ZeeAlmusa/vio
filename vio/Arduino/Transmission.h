//This module is relay the state to a stationary arduino connected PC
//This will help plot the trajectory
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


namespace Transmission{
  RF24 radio(53, 48); // CE, CSN
  
  
  const byte address[6] = "00001";
  int text = 0;


void transmission_setup() {
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}



void relay() {
  char text[32] = {};
  Odometer::string_raw().toCharArray(text, 31);
  //Serial.println(text);
  //const char text[] = "Hello World";
  

  radio.write(&text, sizeof(text));
  

}

}
