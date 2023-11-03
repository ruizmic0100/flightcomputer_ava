#include <Arduino_LSM6DSOX.h>

float Ax, Ay, Az;
float Gx, Gy, Gz;


int flaps_pwm = 0; // flat: 100, Start to move: 130, Tops out at 250
int left_aileron_pwm = 200;
int right_aileron_pwm = 200;

void setup() {
  Serial.begin(9600);


  // Pin Inits
  pinMode(21, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);

  while(!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
  Serial.println();

  Serial.print("Gyroscope sample rate = ");  
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println("Hz");
  Serial.println();

}

void loop() {

  // PWM GPIO5
  if (flaps_pwm >= 255) {
    flaps_pwm = 0;
  }
  if (left_aileron_pwm >= 255) {
    left_aileron_pwm = 200;
    right_aileron_pwm = 200;
  }
  Serial.println(flaps_pwm);


  analogWrite(9, flaps_pwm);
  analogWrite(10, left_aileron_pwm);
  analogWrite(8, right_aileron_pwm);
  
  //analogWrite(9, val);
  //delay(100);
  // analogWrite(8, val);
  flaps_pwm = flaps_pwm + 1;
  left_aileron_pwm = left_aileron_pwm + 1;
  right_aileron_pwm = right_aileron_pwm + 1;
  delay(100);

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);

    // Serial.println("Accelerometer data: ");
    //Serial.print(Ax);
    //Serial.print(',');
    //Serial.print(Ay);
    //Serial.print(',');
    //Serial.print(Az);
    //Serial.print('\n');
  }

  // if (IMU.gyroscopeAvailable()) {
  //   IMU.readGyroscope(Gx, Gy, Gz);
    
  //   Serial.println("Gyroscope data: ");
  //   Serial.print(Gx);
  //   Serial.print('\t');
  //   Serial.print(Gy);
  //   Serial.print('\t');
  //   Serial.println(Gz);
  //   Serial.println();
  // }

//delay(200);

}#include <Arduino_LSM6DSOX.h>

float Ax, Ay, Az;
float Gx, Gy, Gz;

int val = 130;

void setup() {
  Serial.begin(9600);


  // Pin Inits
  pinMode(10, OUTPUT);

  while(!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
  Serial.println();

  Serial.print("Gyroscope sample rate = ");  
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println("Hz");
  Serial.println();

}

void loop() {

  // PWM GPIO5
  if (val >= 240) {
    val = 130;
  }
  analogWrite(10, val);
  delay(100);
  analogWrite(9, val);
  delay(100);
  analogWrite(8, val);
  val = val + 5;
  delay(300);



  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);

    // Serial.println("Accelerometer data: ");
    Serial.print(Ax);
    Serial.print(',');
    Serial.print(Ay);
    Serial.print(',');
    Serial.print(Az);
    Serial.print('\n');
  }

  // if (IMU.gyroscopeAvailable()) {
  //   IMU.readGyroscope(Gx, Gy, Gz);
    
  //   Serial.println("Gyroscope data: ");
  //   Serial.print(Gx);
  //   Serial.print('\t');
  //   Serial.print(Gy);
  //   Serial.print('\t');
  //   Serial.println(Gz);
  //   Serial.println();
  // }

//delay(200);

}