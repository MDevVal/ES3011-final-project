#include <Arduino.h>
#include <Wire.h>
#include <smartmotor.h>
#include <smc_gains.h>

void setup() {
    // INIT SERIAL
    Serial.begin(9600);
    while(!Serial);

    SmartMotor motor(0x0A); // INIT SMARTMOTOR WITH ADDRESS 0x0A
    Wire.begin(); // INIT ARDUINO UNO AS I2C CONTROLLER
    
    // TUNE POSITION PID
    float kp=0.65;
    float ki=0.060;
    float kd=0.065;
    motor.tune_pos_pid(kp,ki,kd); 
    delay(1000); // GIVE SMC A SECOND TO SAVE NEW VALUES

    // PRINT NEW PID GAINS
    Gains pos_gains=motor.get_pos_gains();
    Serial.println("Position PID Gains: ");
    Serial.println("------------------");
    Serial.print("Kp: ");
    Serial.println(pos_gains.kp);
    Serial.print("Ki: ");
    Serial.println(pos_gains.ki);
    Serial.print("Kd: ");
    Serial.println(pos_gains.kd);
    Serial.println();
    
    // TUNE VELOCITY PID
    float kv=1.0;
    kp=0.65;
    ki=0.060;
    kd=0.065;
    motor.tune_vel_pid(kv,kp,ki,kd); 
    delay(1000); // GIVE SMC A SECOND TO SAVE NEW VALUES

    // PRINT NEW PID GAINS
    Gains vel_gains=motor.get_vel_gains();
    Serial.println("Velocity PID Gains: ");
    Serial.println("------------------");
    Serial.print("Kv: ");
    Serial.println(vel_gains.kv);
    Serial.print("Kp: ");
    Serial.println(vel_gains.kp);
    Serial.print("Ki: ");
    Serial.println(vel_gains.ki);
    Serial.print("Kd: ");
    Serial.println(vel_gains.kd);
    Serial.println();
}

void loop() {

} 