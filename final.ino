#include <ECE3.h>

uint16_t sensorValues[8];

// Pin Assignments
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

bool hasTurned = false;
bool finished = false;

double error = 0;
double prevError = 0;

double derivative = 0;

double output = 0;

int forwardSpeed = 110;
double kP = 0.03;
double kD = 0.156; 
double kI;

void setup() {  
  // Pin Settings
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  // Setting Initial Values
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  
  ECE3_Init();
  Serial.begin(9600);
  delay(2000);
}

void loop() {
  ECE3_read_IR(sensorValues);

  if(finished == true || (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] > 2200 * 8)){
    if(hasTurned == false){
      turn();
      hasTurned = true;
    } else{
        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);
        finished = true;
        return;
    }
  }
  
  sensorValues[0] = (sensorValues[0] - 676) * (1000.0 / 2500);
  sensorValues[1] = (sensorValues[1] - 562) * (1000.0 / 2500);
  sensorValues[2] = (sensorValues[2] - 676) * (1000.0 / 2500);
  sensorValues[3] = (sensorValues[3] - 585) * (1000.0 / 2500);
  sensorValues[4] = (sensorValues[4] - 653) * (1000.0 / 2500);
  sensorValues[5] = (sensorValues[5] - 676) * (1000.0 / 2500);
  sensorValues[6] = (sensorValues[6] - 630) * (1000.0 / 2500);
  sensorValues[7] = (sensorValues[7] - 747) * (1000.0 / 2500);


  for(int i = 0; i < 8; i++){
    if(sensorValues[i] < 0){
      sensorValues[i] = 0;
    }
  }

  double sensorFusion = -1 * (8 * sensorValues[0] + 4 * sensorValues[1] + 2 * sensorValues[2] + 1 * sensorValues[3]) + 
  (1 * sensorValues[4] + 2 * sensorValues[5] + 4 * sensorValues[6] + 8 * sensorValues[7]);

  // pid time 

  error = sensorFusion;
  derivative = error - prevError;
  output = kP * error + kD * derivative; 
  prevError = error;
 
  int leftSpeed = maxPWM(forwardSpeed - output);  
  int rightSpeed = maxPWM(forwardSpeed + output);

  analogWrite(left_pwm_pin, leftSpeed);
  analogWrite(right_pwm_pin, rightSpeed);
  
}

int maxPWM(int pwm){
  if(pwm >= 255){
    return 255;
  } else if(pwm <= 0){
    return 0;
  }
  return pwm;
}

void turn(){
  digitalWrite(left_dir_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  analogWrite(left_pwm_pin, 200);
  analogWrite(right_pwm_pin, 200);
  delay(280);
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);
}
