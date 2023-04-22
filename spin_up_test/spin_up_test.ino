#include <Wire.h>
#include <Arduino.h>
#include <Math.h>
#include <util/atomic.h>

#define ENCODER_1_A 3  // pin 2 of the Arduino
#define ENCODER_1_B 7  // pin 9 of the Arduino

#define MOTOR_1_PIN_1 9
#define MOTOR_1_PIN_2 10
#define MOTOR_1_CONTROL_PIN 6

long anglePrevTime = 0;
long angleCurrentTime = 0;
double angleDt;

double currentAngle = 180.0;
double prevAngle = 180.0;
double currentAngleDer = 0.0;
double prevAngleDer = 0.0;
double currentAngleInt = 0.0;
int direction = 0;
int prevDirection = 0;

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
}

void loop() {
  motor1.forward();
  double max_voltage = 12.0;
  double voltage = 3.0;
  double pwm_input = round((voltage/max_voltage)*255.0);
  motor1.setPWM(pwm_input);
  delay(500);
}

void encoder_1_interrupt() {
  prevTime = currentTime;
  currentTime = micros();
  dt = (double)(currentTime - prevTime)/(1.0*pow(10, 6)); // microseconds
  prev_pulses = pulses;
  int b = digitalRead(ENCODER_1_B);
  double dTheta = 360.0/245.0; //degrees (pulses)
  if (b > 0) {
    pulses++;
    prevAngle = currentAngle;
    currentAngle = prevAngle + dTheta;
  } else {
    pulses--;
    prevAngle = currentAngle;
    currentAngle = prevAngle - dTheta;
  }
  if (abs(currentAngle) > 360.0)
  {
    if (signbit(currentAngle))
      currentAngle = (currentAngle - 360.0)*-1.0;
    else
      currentAngle = currentAngle - 360.0;
  }
  currentSpeed = calculateMotorSpeed(dTheta, dt);
  Serial.print(">Angle:");
  Serial.println(currentAngle);
  Serial.print(">Pulses:");
  Serial.println(pulses);
  Serial.print(">Speed:");
  Serial.println(currentSpeed);
  Serial.print(">Time:");
  Serial.println(currentTime);
};

double calculateMotorSpeed(int dTheta, long double dt) {
  // 360 pulses per rev, so 1 pulse = 1 deg **TODO: FIGURE OUT WHAT THE PPR IS FOR THIS MOTOR!**
  if (dt <= 0.0) {
    return 0.0;
  }
  double speed = dTheta / dt;
  return speed;
};