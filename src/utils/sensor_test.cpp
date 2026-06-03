#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"


static SPISettings myMT6701SPISettings(4000000, MT6701_BITORDER, SPI_MODE2);
#define SPI_CS_PIN 48
MagneticSensorMT6701SSI sensor(SPI_CS_PIN, myMT6701SPISettings);

//  BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional))
// BLDCDriver3PWM driver = BLDCDriver3PWM(19, 18, 5, 17);

//  BLDCMotor( pole_pairs , ( phase_resistance, KV_rating  optional) )
// BLDCMotor motor = BLDCMotor(7, 11.2, 80, 0.003);

uint32_t Freq = 0;
 
void setup() 
{
  Serial.begin(115200);
  Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");

  
  // pinMode(12, INPUT_PULLUP);

  // initialize magnetic sensor hardware
  // (mosi, miso, sclk)
  //SPI.begin(33, 35, 32);
  // SCK_PIN, MISO_PIN, MOSI_PIN
  SPI.begin(9, 10, 11);
  sensor.init();
  Serial.begin(115200);


  // // pwm frequency to be used [Hz]
  // driver.pwm_frequency = 40000;
  // // power supply voltage [V]
  // driver.voltage_power_supply = 12;
  // // Max DC voltage allowed - default voltage_power_supply
  // driver.voltage_limit = 12;
  // // driver init
  // driver.init();


  // // pwm frequency to be used [Hz]
  // driver.pwm_frequency = 50000;
  // // power supply voltage [V]
  // driver.voltage_power_supply = 20;
  // // Max DC voltage allowed - default voltage_power_supply
  // driver.voltage_limit = 20;


  // // link the motor to the sensor
  // motor.linkSensor(&sensor);

  // // init driver
  // // link the motor to the driver
  // motor.linkDriver(&driver);

  // // set control loop type to be used
  // motor.controller = MotionControlType::angle;




  // // velocity PID controller parameters
  // // default P=0.5 I = 10 D =0
  // motor.PID_velocity.P = 0.01;
  // motor.PID_velocity.I = 2;
  // motor.PID_velocity.D = 0.00;
  // // jerk control using voltage voltage ramp
  // // default value is 300 volts per sec  ~ 0.3V per millisecond
  // motor.PID_velocity.output_ramp = 0;

  // // velocity low pass filtering
  // // default 5ms - try different values to see what is the best. m,
  // // the lower the less filtered
  // motor.LPF_velocity.Tf = 1/(6.28*200); 

  // setting the limits
  // either voltage
  // motor.voltage_limit = 10; // Volts - default driver.voltage_limit
  // // of current 
  // motor.current_limit = 2; // Amps - default 0.2Amps



  // // angle PID controller 
  // // default P=20
  // motor.P_angle.P = 20; 
  // motor.P_angle.I = 0;  // usually only P controller is enough 
  // motor.P_angle.D = 0;  // usually only P controller is enough 
  // // acceleration control using output ramp
  // // this variable is in rad/s^2 and sets the limit of acceleration
  // motor.P_angle.output_ramp = 0; // default 1e6 rad/s^2

  // // angle low pass filtering
  // // default 0 - disabled  
  // // use only for very noisy position sensors - try to avoid and keep the values very small
  // motor.LPF_angle.Tf = 0; // default 0

  // // setting the limits
  // //  maximal velocity of the position control
  // motor.velocity_limit = 10000; // rad/s - default 20

  // // motor.PID_velocity.limit = 1;
  // // motor.P_angle.limit = 200;


  
  // // driver init
  // driver.init();
  // // initialize motor
  // motor.init();

  // // init current sense
  // if (motor.initFOC())  Serial.println("FOC init success!");
  // else{
  //   Serial.println("FOC init failed!");
  //   return;
  // }


}



float home_pos = 2.9;

int counter = 0;
long prev_millis = 0;

void loop() {
  counter++;
  sensor.update();
  float angle = sensor.getAngle();
  Serial.println(angle);
  delay(100);
//   long cur_millis = millis();
//   if(cur_millis-prev_millis >= 1000){
//     prev_millis = cur_millis;
//     Serial.println(sensor.getAngle());
//     Serial.println(counter);
//     counter = 0;
//   }

//   // driver.setPwm(0.2*12, 0.5*12, 0.66*12);
  
//   // float target_angle = home_pos;
//   float target_angle = home_pos + 0.1*PI * _sin(float(millis())/500*PI);
//   if (digitalRead(12) == LOW){
//     target_angle += 1.8;
//   }


//   motor.loopFOC();
//   if (counter %2 == 0)
//   {
//     motor.move(target_angle);
//   }
  
}

