//Motor methods

namespace Motors {
  
  //right motor pins
  int InA1 = 9;
  int InB1 = 10;
  int PWM1 = 8;
  
  
  //left motor pins
  int InA2 = 4;
  int InB2 = 5;
  int PWM2 = 6;
  
  
  
  void motors_setup() {
  
    pinMode(InA1, OUTPUT);
    pinMode(InB1, OUTPUT);
    pinMode(PWM1, OUTPUT);
  
    pinMode(InA2, OUTPUT);
    pinMode(InB2, OUTPUT);
    pinMode(PWM2, OUTPUT);
  
  }
  
  
  //Open loop forward method
  void forward(int pwm) {
  
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, LOW);
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, LOW);
  
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);
    analogWrite(PWM1, pwm);
    analogWrite(PWM2, pwm);
  
  }
  
  //Open loop backward method
  void backward(int pwm) {
  
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, LOW);
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, LOW);
  
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, HIGH);
    analogWrite(PWM1, pwm);
    analogWrite(PWM2, pwm);
  
  }
  //forward/backward method for right motor
  //params: pwm is can be from -255 to 255
  void right_motor(int pwm) {
  
    if (pwm >= 0) {
      digitalWrite(InA1, LOW);
      digitalWrite(InB1, HIGH);
      analogWrite(PWM1, pwm);
    }
  
    else {
      digitalWrite(InA1, HIGH);
      digitalWrite(InB1, LOW);
      analogWrite(PWM1, -pwm);
  
    }
  
  }
  
  //forward/backward method for left motor
  //params: pwm is can be from -255 to 255
  void left_motor(int pwm) {
    if (pwm >= 0) {
      digitalWrite(InA2, LOW);
      digitalWrite(InB2, HIGH);
      analogWrite(PWM2, pwm);
    }
  
    else {
      digitalWrite(InA2, HIGH);
      digitalWrite(InB2, LOW);
      analogWrite(PWM2, -pwm);
  
    }
  
  }
  //Stop motors
  void brake() {
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, LOW);
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, LOW);
  
  }

}
