#include <Servo.h>
Servo front;
Servo back;

// A2 is rear, B1 is front

// Constants used to set the gait characteristics
int gait_size = 2;

float amp_front;
float amp_back;
float offset_front;
float offset_back;
float freq_weight;
float freq_offset_back;
float freq_offset_front;


/* Servo Mapping
    Map between real-world angles (radians) and motor contol angles
    Back:
      0 degrees = 159
      180 degrees = 33

    Front
      0 degrees = 27
      180 degrees = 152
*/

void front_set(int true_angle) {
  float motor_angle = map(true_angle, 0, 180, 27, 152);
  front.write(motor_angle);
}

void back_set(int true_angle) {
  float motor_angle = map(true_angle, 0, 180, 159, 33);
  back.write(motor_angle);
}

/*  Motor Control Functions
    Arguments is current simulation time
    parameters (amplitude and frequency (0 to 1)
    output is motor control angle in degrees
*/

float get_back_angle(float t) {
  float output = amp_back * sin(20 * freq_weight * 0.35625 * t + freq_offset_back) + offset_back; // calculates the current desired angle in radians
  return -output * 180 / PI; // returns the desired angle in degrees
}

float get_front_angle(float t) {
  float output = amp_front * sin(20 * freq_weight * 0.35625 * t + freq_offset_front) + offset_front; // calculates the current desired angle in radians
  return -output * 180 / PI; // returns the desired angle in degrees
}

float currentTime;
float previousTime;

//********************************************************************//

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  front.attach(6);
  back.attach(5);

  switch (gait_size) {
    case 2: // 1/2 gait size
      amp_back = -0.72247829;
      amp_front = -0.60836185;
      offset_back = -1.95;
      offset_front = -1.25;
      freq_offset_back = -0.4;
      freq_offset_front =  -0.05;
      freq_weight = 2;
      break;
    case 3: // 1/3 gait size
      amp_back = -0.43562765;
      amp_front = -0.44156115;
      offset_back = -2.1;
      offset_front = -1.3;
      freq_offset_back = -0.55;
      freq_offset_front = -0.05 ;
      freq_weight = 3;
      break;
    case 4: // 1/4 gait size
      amp_back = -0.2692595;
      amp_front = -0.35929823;
      offset_back = -2.1;
      offset_front = -1.3;
      freq_offset_back = -0.7;
      freq_offset_front = -0.05 ;
      freq_weight = 4;
      break;
    case 5: // 4/7 gait size
      amp_back = -0.79126349;
      amp_front = -0.6768732;
      offset_back = -2.0;
      offset_front = -1.25;
      freq_offset_back = -0.45;
      freq_offset_front = -0.05;
      freq_weight = 7/4;
      break;
  }

  float ini_back = get_back_angle(0);
  float ini_front = get_front_angle(0);
  back_set(ini_back);
  front_set(ini_front);
  Serial.print("back_motor_angle");
  Serial.print(",");
  Serial.println("front_motor_angle");

  Serial.print(ini_back);
  Serial.print(",");
  Serial.println(ini_front);

  delay(5000);

}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis() - 5000; // current time in ms
  if (abs(currentTime - previousTime) >=  10) {
    float back_angle = get_back_angle(currentTime / 1000); // functions expect time in s
    float front_angle = get_front_angle(currentTime / 1000);;

    back_set(back_angle);
    front_set(front_angle);
    previousTime = currentTime;

    Serial.print(back_angle);
    Serial.print(",");
    Serial.println(front_angle);
  }
}
