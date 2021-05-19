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

double error = 0;
double prevError = 0;

double derivative = 0;

double output = 0;
double kP = 7;
double kD = 1; 
double kI;

void setup() {
  // put your setup code here, to run once:
  
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
  // put your main code here, to run repeatedly:
  ECE3_read_IR(sensorValues);

  /*
  sensorValues[0] = (sensorValues[0] - 676) * (1000.0 / 2494);
  sensorValues[1] = (sensorValues[1] - 585) * (1000.0 / 2026);
  sensorValues[2] = (sensorValues[2] - 676) * (1000.0 / 2447);
  sensorValues[3] = (sensorValues[3] - 607) * (1000.0 / 1794);
  sensorValues[4] = (sensorValues[4] - 653) * (1000.0 / 1795);
  sensorValues[5] = (sensorValues[5] - 676) * (1000.0 / 2215);
  sensorValues[6] = (sensorValues[6] - 630) * (1000.0 / 2096);
  sensorValues[7] = (sensorValues[7] - 723) * (1000.0 / 2500);
  */
  if(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] > 2200 * 8){
    if(hasTurned == false){
      turn();
      hasTurned = true;
    } else{
        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);
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

  

  double sensorFusion = (-1 * (15 * sensorValues[0] + 14 * sensorValues[1] + 12 * sensorValues[2] + 8 * sensorValues[3]) + 
  (8 * sensorValues[4] + 12 * sensorValues[5] + 15 * sensorValues[6] + 15 * sensorValues[7])) / 4.0;

/*
  for (unsigned char i = 0; i < 8; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.print(sensorFusion);
  
  Serial.println();
`
  delay(1000);
  */

  // pid time 

  error = (sensorFusion - 0)/100.0;
  //integral = integral_prior + error * iteration_time
  derivative = error - prevError;
  //output = KP*error + KI*integral + KD*derivative + bias
  output = kP * error + kD * derivative; 
  prevError = error;
  //integral_prior = integral
 // sleep(iteration_time)
 
  int leftSpeed = maxPWM(50 - error);
  int rightSpeed = maxPWM(50 + error);
  
  /*Serial.print("error: ");
  Serial.print(error);
  Serial.print(", porportional: ");
  Serial.print(kP * error);
  Serial.print(", derivative: ");
  Serial.print(kD * derivative);
  Serial.print(", leftSpeed: ");
  Serial.print(leftSpeed);
  Serial.print(", rightSpeed: ");
  Serial.print(rightSpeed);
  Serial.println(); */
  
/*
  if(leftSpeed < 0){
    digitalWrite(left_dir_pin,HIGH);
  }
  else{
    digitalWrite(left_dir_pin,LOW);
  }
  if(rightSpeed < 0){
    digitalWrite(right_dir_pin,HIGH);
  }
  else{
    digitalWrite(right_dir_pin,LOW);
  }
  */
  analogWrite(left_pwm_pin, leftSpeed);
  analogWrite(right_pwm_pin, rightSpeed);
  
  delay(5);
}

int maxPWM(int pwm){
  if(pwm >= 255){
    return 255;
  } else if(pwm <= -255){
    return -255;
  }
  return pwm;
}

void turn(){
  digitalWrite(left_dir_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  analogWrite(left_pwm_pin, 255);
  analogWrite(right_pwm_pin, 255);
  delay(250);
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);
}
