

namespace Trajectory{
  

  
  float previous_milage = 0;
  bool initiated_path = false;
  
  bool forward(int pwm, float current_milage, float distance){
    
    int control_signal;
    float traveled_dist;
    bool done = false;
    
    if (!initiated_path){
      previous_milage = current_milage;
      initiated_path = true;
    }
    
    traveled_dist = current_milage - previous_milage;
   
    if (traveled_dist >= distance) done = true;
    if (!done){
      Motors::right_motor(pwm); 
      control_signal = Controller::control_forward_pos(Odometer::get_counts1(), Odometer::get_counts2());
      Motors::left_motor(control_signal);
    }

    else{
      Motors::brake();
    }

  

    return done;
    
  }


  void pointTurn(float deg){
    
    int control_signal;
    
    control_signal = Controller::point_turn(deg, Odometer::get_heading());
    
    Motors::right_motor(control_signal);
    Motors::left_motor(-control_signal);
    
  }

  
}
