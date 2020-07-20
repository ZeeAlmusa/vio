//Controller is responsible to provide controlled outputs for driving forward and turning
//The control scheme is PID, tuned manually.

namespace Controller {
  //I is the integral error accumulator
  //Each controller needs its own to keep track of its respective error
  float I1 = 0;
  float I2 = 0;
  float I3 = 0;

  //Old error used for D compensation
  //Need to retain this out of scope to for the next iteration
  float old_error1 = 0;
  float old_error2 = 0;
  float old_error3 = 0;

  //time variables for each controller need to get dt = new_time - old_time
  float old_time1 = 0;
  float old_time2 = 0;
  float old_time3 = 0;
  
  
  //Speed control for the wheel based on a given RPM setpoint
  //Can be an arbitrary RPM or can ideally pass the other wheels RPM 
  //This can be doing through passing the setpoint = Odometer::get_RPMx()
  //returns the controlled PWM signal to the motor
  float controlled_forward_vel(float setpoint, float RPM) {

    //Manually tuned gains
    const float Kp = 10; //10 //10 // 10
    const float Kd = 5; //3  //8 //  5
    const float Ki = 0.05; //0.007 //0.002 // 0.05

    // This limits how much the integral controller can contribute to output
    // Output is limited to Max_Output / Ki_thresh
    // In this case the maximum PWM is 255
    const float Ki_thresh = 2.1; 
  
    //find time incremenet
    float t = millis();
    float dt = t - old_time1;

    //find error
    float error = setpoint - RPM;

    //calculate error integral
    float I1 = I1 + error * dt;

    //Check the integral compensator against the threshold
    if (I1 >= (255 / Ki) / Ki_thresh) {
  
      I1 = (255 / Ki) / Ki_thresh;
    }
  
    else if (I1 <= (-255 / Ki) / Ki_thresh) {
      I1 = (-255 / Ki) / Ki_thresh;
    }

    //Find the error derivative
    float D1 = (error - old_error1) / dt;

    //Get the output
    int PWM = Kp * error + Ki * I1 + Kd * D1;

    //Constrain it to PWM range
    PWM = constrain(PWM, -255, 255);

    //Update time and error variables
    old_time1 = t;
    old_error1 = error;
    
    return PWM;
  
  
  
  }
  
  
  //Position controller that takes the setpoint as one wheels encoder ticks
  //While the other tries to match it
  //Can be done through: control_forward_pos(Odometer::get_counts1(), Odometer::get_counts2())
  float control_forward_pos(float setpoint, float encoder_ticks) {
  
    const float Kp = 2.2; //10 //10 //8.9
    const float Kd = 10; //3  //8 //5
    const float Ki =  0.3; //0.007 //0.002 //0.05
    const float Ki_thresh = 2.1;
  
  
    float t = millis();
    float dt = t - old_time2;
    float error = setpoint - encoder_ticks;
  
    float I2 = I2 + error * dt;
  
    if (I2 >= (255 / Ki) / Ki_thresh) {
  
      I2 = (255 / Ki) / Ki_thresh;
    }
  
    else if (I2 <= (-255 / Ki) / Ki_thresh) {
      I2 = (-255 / Ki) / Ki_thresh;
    }
  
    float D2 = (error - old_error2) / dt;
  
    int PWM = Kp * error + Ki * I2 + Kd * D2;
    PWM = constrain(PWM, -255, 255);
  
    old_time2 = t;
    old_error2 = error;
  
    return PWM;
  
  }
  
  //Heading controller
  //setpoint is the desired heading, heading is the current heading
  //the setpoint can be taken from the IMU, odometer or a fused estimate
  //If the error is small enough, it stops to avoid oscillating
  float point_turn(float setpoint, float heading) {
  
    const float Kp = 20;
    const float Kd = 32;
    const float Ki =  0.000013;
    const float Ki_thresh = 1.9;
  
    float t = millis();
  
  
    float dt = t - old_time3;
  
    float error = setpoint - heading;

    
    I3 = I3 + error * dt;
  
    if (I3 >= (255 / Ki) / Ki_thresh) {
  
      I3 = (255 / Ki) / Ki_thresh;
    }
  
    else if (I3 <= (-255 / Ki) / Ki_thresh) {
      I3 = (-255 / Ki) / Ki_thresh;
    }
  
    float D3 = (error - old_error3) / dt;
  
    float PWM = Kp * error + Ki * I3 + Kd * D3;
    PWM = constrain(PWM, -255, 255);

    //If the error is small enough, stop
    if (abs(error) <= 0.01) {
      PWM = 0;
    }
    old_time3 = t;
    old_error3 = error;
  
    return PWM;
  
  
  }
  
  



}
