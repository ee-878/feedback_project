#include <Wire.h>
#include <Arduino.h>
#include <Math.h>
#include <util/atomic.h>

#define ENCODER_1_A 3  // pin 2 of the Arduino
#define ENCODER_1_B 7  // pin 9 of the Arduino

#define MOTOR_1_PIN_1 9
#define MOTOR_1_PIN_2 10
#define MOTOR_1_CONTROL_PIN 6

long prev_time = 0;
long current_time = 0;
long loop_time;

double currentAngle = 180.0;
double prevAngle = 180.0;
double currentAngleDer = 0.0;
double prevAngleDer = 0.0;
double currentAngleInt = 0.0;
int direction = 0;
int prevDirection = 0;

bool begin_control = false;

double prev_control;
double K = 1.5;
double a = 0.95;
double b = 0.5;

double error = 0.0;
double prev_error = 0.0;
double error_derivative = 0.0;
double error_int = 0.0;

volatile long pulses;
static volatile long prev_pulses;
volatile double currentSpeed;
volatile long prevTime;
volatile long currentTime;
volatile double dt;

class Motor
{
  /* Class for controlling a motor with an encoder. One encoder can be used from this class */
  public: 

    int motor_pin_1;
    int motor_pin_2;
    int motor_control_pin;

  public:

    void initialize(int pin_1, int pin_2, int control_pin) {
      motor_pin_1 = pin_1;
      motor_pin_2 = pin_2;
      motor_control_pin = control_pin;
      pinMode(motor_pin_1, OUTPUT);
      pinMode(motor_pin_2, OUTPUT);
      digitalWrite(motor_pin_1, HIGH);
      digitalWrite(motor_pin_2, LOW);
      analogWrite(motor_control_pin, 0); //start in forward with 0 pwm
    };
    
    void forward() {
      digitalWrite(motor_pin_1, HIGH);
      digitalWrite(motor_pin_2, LOW);
    };

    void reverse() {
      digitalWrite(motor_pin_1, LOW);
      digitalWrite(motor_pin_2, HIGH);
    };

    void setPWM(int pwmInput) {
      analogWrite(motor_control_pin, pwmInput);
    };

    double getSpeed() {
      return currentSpeed;
    };
};

Motor motor1;


void setup() {
  Serial.begin(115200);                     // activates the serial communication

  motor1.initialize(MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_1_CONTROL_PIN);

  pinMode(ENCODER_1_A, INPUT_PULLUP);  // sets the Encoder_output_A pin as the input
  pinMode(ENCODER_1_B, INPUT_PULLUP);  // sets the Encoder_output_B pin as the input
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_A), encoder_1_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_B), encoder_quad_interrupt, FALLING);
  Serial.println("Press enter to begin control...");
}

void loop() {
  
  if (!begin_control) {
    if (Serial.available())
        begin_control = true;
  }

  double t = millis()/1000.0;
  double loop_period = 0.025;



  prev_time = current_time;
  current_time = micros();
  dt = (double)(current_time - prev_time)/(1.0*pow(10, 6)); // microseconds
  //Serial.print("dt: ");
  //Serial.println(angleDt);

  double ref = 5.0;
  prev_error = error;
  error = ref*(PI/180.0) - currentAngle;
  error_derivative = (error - prev_error)/dt;
  error_int = error_int + error*dt;
  currentAngleInt = (prevAngle + currentAngle)*dt;

  double controlOutput = K*error - K*a*prev_error + b*prev_control; // in Volts

  Serial.print(">error:");
  Serial.println(error);

  Serial.print(">Voltage:");
  Serial.println(controlOutput);

  controlOutput = round((controlOutput/5.0)*255); //map to 0 to 255 for pwm, allow for over/under that as wells
  if (controlOutput >= 0) {
    motor1.reverse();
  }
  if (controlOutput < 0) {
    motor1.forward();
    controlOutput = abs(controlOutput);
  }
  if (controlOutput > 255) {
    controlOutput = 255;
  }  

  prev_control = controlOutput*5.0/255.0;

  motor1.setPWM(controlOutput);
  double motor1Speed = motor1.getSpeed();

  loop_time = micros();
  if ((loop_time - current_time)/1000.0 < loop_period)
    delay((loop_time - current_time)/1000.0);
}

void encoder_1_interrupt() {
  prevTime = currentTime;
  currentTime = micros();
  dt = (double)(currentTime - prevTime)/(1.0*pow(10, 6)); // microseconds
  prev_pulses = pulses;
  int b = digitalRead(ENCODER_1_B);
  //double dTheta = (360.0)/(245.0*2.0); //degrees (pulses)
  double dTheta = 360.0/(488.0);
  if (b > 0) {
    pulses++;
    prevAngle = currentAngle;
    currentAngle = prevAngle + dTheta;
  } else {
    pulses--;
    prevAngle = currentAngle;
    currentAngle = prevAngle - dTheta;
  }
  currentSpeed = calculateMotorSpeed(dTheta, dt);
  Serial.print(">Angle:");
  Serial.println(currentAngle);
  Serial.print("Pulses:");
  Serial.println(pulses);
  Serial.print(">Speed:");
  Serial.println(currentSpeed);
};

void encoder_quad_interrupt() {
  prevTime = currentTime;
  currentTime = micros();
  dt = (double)(currentTime - prevTime)/(1.0*pow(10, 6)); // microseconds
  prev_pulses = pulses;
  int b = digitalRead(ENCODER_1_A);
  //double dTheta = (360.0)/(245.0*2.0); //degrees (pulses)
  double dTheta = 360.0/(488.0);
  if (b > 0) {
    pulses++;
    prevAngle = currentAngle;
    currentAngle = prevAngle + dTheta;
  } else {
    pulses--;
    prevAngle = currentAngle;
    currentAngle = prevAngle - dTheta;
  }
  currentSpeed = calculateMotorSpeed(dTheta, dt); // TODO: fix this function, only reports 0 currently
  Serial.print(">Angle:");
  Serial.println(currentAngle);
  Serial.print(">Pulses:");
  Serial.println(pulses);
  Serial.print(">Speed:");
  Serial.println(currentSpeed);
};

double calculateMotorSpeed(double dTheta, long double dt) {
  // 360 pulses per rev, so 1 pulse = 1 deg **TODO: FIGURE OUT WHAT THE PPR IS FOR THIS MOTOR!**
  if (dt <= 0.0) {
    return 0.0;
  }
  double speed = dTheta / dt;
  return speed;
};