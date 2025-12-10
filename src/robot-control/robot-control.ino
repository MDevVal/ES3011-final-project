#include <Wire.h>
#include <smartmotor.h>

SmartMotor left_motor(0x0A); 
SmartMotor right_motor(0x0B);

const float WHEEL_DIAMETER_CM = 6.5;  // Adjust to your wheel size
const float WHEEL_BASE_CM = 15.0;      // Distance between wheels
const int16_t DRIVE_SPEED = 40;        // RPM for driving
const int16_t TURN_SPEED = 30;         // RPM for turning

const float CM_PER_ROTATION = WHEEL_DIAMETER_CM * 3.14159;
const float ROTATIONS_FOR_1M = 100.0 / CM_PER_ROTATION; //We should offset this durring testing to hit the actual finish line
const float ROTATIONS_FOR_180 = (WHEEL_BASE_CM * 3.14159) / CM_PER_ROTATION;

enum State {
  DRIVE_FORWARD,
  TURN_180,
  DRIVE_BACK,
  STOP
};

State current_state = DRIVE_FORWARD;
int32_t start_angle_left = 0;
int32_t start_angle_right = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Wire.begin();

  left_motor.tune_vel_pid(1.0, 0.65, 0.060, 0.065);
  right_motor.tune_vel_pid(1.0, 0.65, 0.060, 0.065);


  delay(100);

  start_angle_left = left_motor.read_angle();
  start_angle_right = right_motor.read_angle();
  
  Serial.println("Starting mission: Drive 1m -> Turn 180 -> Drive back");
  delay(1000);
}

void loop() {
  int32_t current_left = left_motor.read_angle();
  int32_t current_right = right_motor.read_angle();
  
  float rotations_left = abs(current_left - start_angle_left) / 360.0;
  float rotations_right = abs(current_right - start_angle_right) / 360.0;
  
  switch(current_state) {
    
    case DRIVE_FORWARD:
      left_motor.write_rpm(DRIVE_SPEED);
      right_motor.write_rpm(-DRIVE_SPEED);
      
      Serial.print("FORWARD - Distance: ");
      Serial.print(rotations_right * CM_PER_ROTATION);
      Serial.println(" cm");
      
      if (rotations_right >= ROTATIONS_FOR_1M) {
        Serial.println("Reached 1m, starting turn...");
        current_state = TURN_180;
        start_angle_left = current_left;
        start_angle_right = current_right;
        delay(500);
      }
      break;
      
    case TURN_180:
      left_motor.write_rpm(TURN_SPEED);
      right_motor.write_rpm(TURN_SPEED);
      
      Serial.print("TURNING - Angle: ");
      Serial.print((rotations_right / ROTATIONS_FOR_180) * 180.0);
      Serial.println(" deg");
      
      if (rotations_right >= ROTATIONS_FOR_180) {
        Serial.println("Completed 180 turn, driving back...");
        current_state = DRIVE_BACK;
        start_angle_left = current_left;
        start_angle_right = current_right;
        delay(500); 
      }
      break;
      
    case DRIVE_BACK:
      left_motor.write_rpm(DRIVE_SPEED);
      right_motor.write_rpm(-DRIVE_SPEED);
      
      Serial.print("RETURN - Distance: ");
      Serial.print(rotations_right * CM_PER_ROTATION);
      Serial.println(" cm");
      
      if (rotations_right >= ROTATIONS_FOR_1M) {
        Serial.println("Mission complete!");
        current_state = STOP;
      }
      break;
      
    case STOP:
      left_motor.write_rpm(0);
      right_motor.write_rpm(0);
      Serial.println("STOPPED");
      delay(5000);
      break;
  }
  
  delay(50);  
}
