//Odometer assumes a robot with 2 wheels with motors (right and left)
//Naming convention: anything appended with 1 is for the right wheel, 2 is for the left wheel
//Note that cm is the output of the odometer. The states are express in cm and radians
//The states are (x, y, theta)
//The board is assumed to be an Arduino Mega 
namespace Odometer {


  //Define encoder interrupt pins
  //Right wheel
  const int encoder1A = 18;
  const int encoder1B = 19;

  //Left wheel
  const int encoder2A = 2;
  const int encoder2B = 3;
  
  
  const int rev_ticks = 64; //motor ticks per revolution before gear down
  const int out_ticks = 1920; //output ticks per revolution
  
  const float wheel = 123.825; //wheel diameter in mm
  
  //time management variables
  float current_time = 0;
  float old_time = 0;
  float dt = 0;
  
  volatile long int currentPosition1 = 0; //motor ticks of the right wheel
  
  //both these variables are used to get the differential movement for the right wheel
  long int currentMove1 = 0;
  long int oldMove1 = 0;
  float RPM1 = 0;
  
  
  volatile long int currentPosition2 = 0; //motor ticks of the left wheel
  
  //both these variables are used to get the differential movement for the left wheel
  long int currentMove2 = 0;
  long int oldMove2 = 0;
  float RPM2 = 0;
  
  
  //path length traveled by each wheel
  float speed1 = 0;
  float speed2 = 0;
  
  //path length traveled by each wheel, these are differential at each loop update
  float d_center = 0;
  float d_left = 0;
  float d_right = 0;
  
  //total traveled distance by the center of the robot
  float milage = 0;
  
  //phi is the differental yaw moved, theta is the current yaw
  float phi = 0;
  
  //the state variables
  float x = 0;
  float y = 0;
  float theta = 0;

  //auxilliary variables
  float v = 0; //robot speed
  float a = 0; //robot acceleration
  float d_center_old = 0;
  float v_old = 0;
  
  //distance from wheel to wheel
  const float D_BASE = 55; //in cm
  
  // ================================================================
  // ===                      Encoder Counters                    ===
  // ================================================================
  
  
  //These four functions count the encoder ticks
  //forward/backward for each wheel respectively
  void doEncoder1A() {
  
    if (digitalRead(encoder1A) != digitalRead(encoder1B)) {
      currentPosition1++;
    }
    else {
      currentPosition1--;
    }
  
  }
  
  void doEncoder1B() {
  
    if (digitalRead(encoder1A) == digitalRead(encoder1B)) {
      currentPosition1++;
    }
    else {
      currentPosition1--;
    }
  
  }
  
  void doEncoder2A() {
  
    if (digitalRead(encoder2A) != digitalRead(encoder2B)) {
      currentPosition2++;
    }
    else {
      currentPosition2--;
    }
  
  }
  
  
  void doEncoder2B() {
  
    if (digitalRead(encoder2A) == digitalRead(encoder2B)) {
      currentPosition2++;
    }
    else {
      currentPosition2--;
    }
  
  }
  
  
  // ================================================================
  // ===                      Encoder Setup                       ===
  // ================================================================
  
  
  
  void encoder_setup() {
    // put your setup code here, to run once:
    pinMode(encoder1A, INPUT_PULLUP);
    pinMode(encoder1B, INPUT_PULLUP);
    pinMode(encoder2A, INPUT_PULLUP);
    pinMode(encoder2B, INPUT_PULLUP);
  
    attachInterrupt(digitalPinToInterrupt(encoder1A), doEncoder1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder1B), doEncoder1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2A), doEncoder2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2B), doEncoder2B, CHANGE);
  
  
    old_time = millis();
  }
  
  
  // ================================================================
  // ===                      State Prediction                    ===
  // ================================================================

  //This finds the states (x, y, theta) of the robot using the odometry motion model
  //The output is in (cm, cm, rad)
  //This function is called in the main program function to toggle the odometry tracking
  //The states are updated only when the robot wheel(s) are in motion
  void find_state() {

    //get the time increment in seconds
    current_time = millis();
    dt = (current_time - old_time) / 1000;

    //get the current total ticks for each
    currentMove1 = currentPosition1;
    currentMove2 = currentPosition2;

    //calculate the RPM for each wheel by dividing differential ticks over dt. Convert to RPM.
    RPM1 = (currentMove1 - oldMove1) / (out_ticks * dt) * 60 ;
    RPM2 = (currentMove2 - oldMove2) / (out_ticks * dt) * 60 ;

    //Odometery motion model for differential steering
    //1. Differential rotation in radian multiply by diameter dS = dTheta * Diameter
    //2. Wheel diameter is in mm, convert to cm. Output is cm
    d_right = (float) (currentMove1 - oldMove1) / (float)out_ticks * PI * wheel / 10;
    d_left = (float) (currentMove2 - oldMove2) / (float)out_ticks * PI * wheel / 10;

    
    d_center = (d_left + d_right) / 2.0; //Robot differential distance is the average of right and left
    milage += d_center; //Keep track of total distance
    phi = (d_right - d_left) / D_BASE; //differential rotation 

    v = (d_center - d_center_old)/ dt; // find the speed of the robot (cm/s) 
    a = (v - v_old) / dt; //find the acceleration of the robot (cm/s^2)
      
    //Only update when the robot is moving
    if (!(RPM1 == 0.0 && RPM2 == 0.0)) {
  
      theta = theta + phi; //update heading
   
      x = x + d_center * cos(theta); //update x
      y = y + d_center * sin(theta); //update y
  
  
    }
  
    //update the variables for the next iteration calculations
    old_time = current_time;
    oldMove1 = currentMove1;
    oldMove2 = currentMove2;
    d_center_old = d_center;
    v_old = v;
  
  }
  
  // ================================================================
  // ===                      Getter Methods                      ===
  // ================================================================
  
  //Right wheel RPM
  float get_RPM1() {
    return RPM1;
  
  }
  
  //Left wheel RPM
  float get_RPM2() {
    return RPM2;
  
  }
  
  //Right wheel encoder counts
  float get_counts1() {
    return currentPosition1;
  }
  
  //Left wheel encoder counts
  float get_counts2() {
    return currentPosition2;
  }
  
  //center of robot differential distance
  float get_distance() {
    return d_center;
  }
  
  //right wheel differential distance
  float get_dright() {
    return d_right;
  }
  
  //left wheel differential distance
  float get_dleft() {
    return d_left;
  }
  
  //differential heading
  float get_phi() {
    return phi;
  }
  
  //differential heading
  float get_x() {
    return x;
  }
  
  //differential heading
  float get_y() {
    return y;
  }
  
  //heading of the robot
  float get_heading() {
  
    return theta;
  }
  
  //total distance of the robot
  float get_milage() {
    return milage;
  }

  float get_v(){
    return v;
  }

  float get_a(){
    return a;
  }
  
  // ================================================================
  // ===                      Print  Methods                      ===
  // ================================================================
  
  void print_state() {
  
    Serial.print(x);
    Serial.print('/');
    Serial.print(y);
    Serial.print('/');
    Serial.print(theta);
    Serial.print('/');
    Serial.println(-IMU::get_yaw()/180 * PI);
    
  
  }

   String string_state() {
     
    //TODO: convert state to string and write it maybe?
    String state = String(x) + "/" + String(y) + "/" + String(theta) + "/" + String(-IMU::get_yaw()/180 * PI);
    
    return state;
  }

}
