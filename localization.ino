#include "IMU.h"
#include "Motors.h"
#include "odometer.h"
#include "Controller.h"
#include "Trajectory.h"
#include "Transmission.h"

bool segment_done = false;
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  Serial.begin(19200);
  IMU::IMU_setup();
  Motors::motors_setup();
  Odometer::encoder_setup();
  Transmission::transmission_setup();
  
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
  IMU::find_orientation();
  Odometer::find_state();
  
  //segment_done = Trajectory::forward(155, Odometer::get_milage(), 300);
  
  //Odometer::print_state();
  Transmission::relay();

}
