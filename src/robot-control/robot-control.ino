#include <Wire.h>
#include <smartmotor.h>

SmartMotor left_motor(0x0A); 
SmartMotor right_motor(0x0B);

const float WHEEL_DIAMETER_CM = 5.7;
const float WHEEL_BASE_CM = 8.8+1;
const int16_t MAX_DRIVE_SPEED = 50;
const int16_t MAX_TURN_SPEED = 25;
const float CM_PER_ROTATION = WHEEL_DIAMETER_CM * 3.14159;
const float ROTATIONS_FOR_1M = 100.0 / CM_PER_ROTATION;
const float ROTATIONS_FOR_180 = (WHEEL_BASE_CM * 3.14159 / 2.0) / CM_PER_ROTATION;

// Trapezoidal profile parameters
const float ACCEL_TIME_MS = 500.0;        // Time to reach max speed (ms)
const float DECEL_TIME_MS = 500.0;        // Time to decelerate to stop (ms)
const float ACCEL_DISTANCE_PERCENT = 0.1; // Accelerate for first 20% of distance
const float DECEL_DISTANCE_PERCENT = 0.1; // Decelerate for last 20% of distance

enum State {
  DRIVE_FORWARD,
  TURN_180,
  DRIVE_BACK,
  STOP
};

State current_state = DRIVE_FORWARD;
int32_t start_angle_left = 0;
int32_t start_angle_right = 0;
unsigned long state_start_time = 0;

// Trapezoidal motion profile calculator
float calculateTrapezoidalSpeed(float progress, float max_speed, 
                                 float accel_fraction, float decel_fraction) {
  // progress: 0.0 to 1.0 (0% to 100% of motion)
  // Returns speed from 0 to max_speed with smooth ramps
  
  if (progress < accel_fraction) {
    // Acceleration phase: linear ramp up
    return max(max_speed * (progress / accel_fraction), (0.3*max_speed));
  } 
  else if (progress > (1.0 - decel_fraction)) {
    // Deceleration phase: linear ramp down
    float decel_progress = (1.0 - progress) / decel_fraction;
    return max(max_speed * decel_progress, (.3*max_speed));
  } 
  else {
    // Constant velocity phase
    return max_speed;
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Wire.begin();
  
  left_motor.tune_vel_pid(0, 10, 2, 1.1);
  right_motor.tune_vel_pid(0, 13, 2, 1.1);
  delay(100);
  
  start_angle_left = left_motor.read_angle();
  start_angle_right = right_motor.read_angle();
  state_start_time = millis();
  
  Serial.println("Starting mission with trapezoidal motion profiling");
  Serial.println("Drive 1m -> Turn 180 -> Drive back");
  delay(1000);
}

void loop() {
  int32_t current_left = left_motor.read_angle();
  int32_t current_right = right_motor.read_angle();
  
  float rotations_left = abs(current_left - start_angle_left) / 360.0;
  float rotations_right = abs(current_right - start_angle_right) / 360.0;
  
  switch(current_state) {
    
    case DRIVE_FORWARD: {
      // Calculate progress (0.0 to 1.0)
      float progress = rotations_right / ROTATIONS_FOR_1M;
      progress = constrain(progress, 0.0, 1.0);
      
      // Calculate speed using trapezoidal profile
      float target_speed = calculateTrapezoidalSpeed(progress, MAX_DRIVE_SPEED, 
                                                     ACCEL_DISTANCE_PERCENT, 
                                                     DECEL_DISTANCE_PERCENT);
      
      // Apply speed with proper signs for differential drive
      left_motor.write_rpm(-target_speed);
      right_motor.write_rpm(target_speed);
      
      Serial.print("FORWARD - Dist: ");
      Serial.print(rotations_right * CM_PER_ROTATION);
      Serial.print(" cm | Progress: ");
      Serial.print(progress * 100.0);
      Serial.print("% | Speed: ");
      Serial.print(target_speed);
      Serial.print(" rpm | L: ");
      Serial.print(left_motor.read_rpm());
      Serial.print(" R: ");
      Serial.println(right_motor.read_rpm());
      
      if (rotations_right >= ROTATIONS_FOR_1M) {
        Serial.println("Reached 1m, starting turn...");
        current_state = TURN_180;
        start_angle_left = current_left;
        start_angle_right = current_right;
        state_start_time = millis();
        delay(300);
      }
      break;
    }
      
    case TURN_180: {
      // Calculate progress (0.0 to 1.0)
      float progress = rotations_right / ROTATIONS_FOR_180;
      progress = constrain(progress, 0.0, 1.0);
      
      // Calculate speed using trapezoidal profile
      float target_speed =MAX_TURN_SPEED; //calculateTrapezoidalSpeed(progress, MAX_TURN_SPEED, 
                                                     //ACCEL_DISTANCE_PERCENT, 
                                                     //DECEL_DISTANCE_PERCENT);
      
      // Both motors same direction for turning in place
      left_motor.write_rpm(target_speed);
      right_motor.write_rpm(target_speed);
      
      Serial.print("TURNING - Angle: ");
      Serial.print(progress * 180.0);
      Serial.print(" deg | Progress: ");
      Serial.print(progress * 100.0);
      Serial.print("% | Speed: ");
      Serial.print(target_speed);
      Serial.print(" rpm | L: ");
      Serial.print(left_motor.read_rpm());
      Serial.print(" R: ");
      Serial.println(right_motor.read_rpm());
      
      if (rotations_right >= ROTATIONS_FOR_180) {
        Serial.println("Completed 180 turn, driving back...");
        current_state = DRIVE_BACK;
        start_angle_left = current_left;
        start_angle_right = current_right;
        state_start_time = millis();
        delay(300); 
      }
      break;
    }
      
    case DRIVE_BACK: {
      // Calculate progress (0.0 to 1.0)
      float progress = rotations_right / ROTATIONS_FOR_1M;
      progress = constrain(progress, 0.0, 1.0);
      
      // Calculate speed using trapezoidal profile
      float target_speed = calculateTrapezoidalSpeed(progress, MAX_DRIVE_SPEED, 
                                                     ACCEL_DISTANCE_PERCENT, 
                                                     DECEL_DISTANCE_PERCENT);
      
      // Apply speed with proper signs for differential drive
      left_motor.write_rpm(-target_speed);
      right_motor.write_rpm(target_speed);
      
      Serial.print("RETURN - Dist: ");
      Serial.print(rotations_right * CM_PER_ROTATION);
      Serial.print(" cm | Progress: ");
      Serial.print(progress * 100.0);
      Serial.print("% | Speed: ");
      Serial.print(target_speed);
      Serial.print(" rpm | L: ");
      Serial.print(left_motor.read_rpm());
      Serial.print(" R: ");
      Serial.println(right_motor.read_rpm());
      
      if (rotations_right >= ROTATIONS_FOR_1M) {
        Serial.println("Mission complete!");
        current_state = STOP;
      }
      break;
    }
      
    case STOP:
      left_motor.write_rpm(0);
      right_motor.write_rpm(0);
      Serial.println("STOPPED - Mission Complete");
      delay(5000);
      break;
  }
  
  delay(50);  
}
