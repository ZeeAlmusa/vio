#include "IMU.h"
#include "Motors.h"
#include "odometer.h"
#include "Controller.h"
#include "Trajectory.h"
#include "Transmission.h"

bool segment_done = false;
float IMU_acc = 0;
float odo_acc = 0;
float vel, pos;
float prev_pos = 0;

float old_time = 0;
float new_time, dt;
String message;
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  Serial.begin(115200);
  IMU::IMU_setup();
  Motors::motors_setup();
  Odometer::encoder_setup();
  Transmission::transmission_setup();
  old_time = millis();
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  new_time = millis();
  dt = new_time - old_time;
  IMU::find_orientation();
  Odometer::find_state();
 
  //segment_done = Trajectory::forward(155, Odometer::get_milage(), 200);
  
  //Odometer::print_state();
  Odometer::print_state();
  Transmission::relay();
  
  
  //IMU_acc = IMU::getAy();
  //odo_acc = Odometer::get_a();
  //pos = prev_pos + IMU_acc*dt - (0.02)*dt + (0.5*IMU_acc*dt*dt - (0.02)*dt*dt);
  
//
//  Serial.print("IMU = ");
//  Serial.print(IMU_acc);
//  Serial.print("\t");
//  Serial.print("Odometer = ");
//  Serial.println(-Odometer::get_a());
   
    old_time = new_time;
  
}
