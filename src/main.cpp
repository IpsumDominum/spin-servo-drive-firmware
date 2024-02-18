#include <Arduino.h>
#include <SimpleFOC.h>

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number, phase resistance (optional) );
// BLDCMotor motor = BLDCMotor(4);
BLDCMotor motor = BLDCMotor(4, 2.0, 300.0);

BLDCDriver3PWM driver = BLDCDriver3PWM(PA0, PA1, PA2);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, 46, PB0, PB1, PB12);

//Position Sensor
SPIClass SPI_2(PB15, PB14, PB13); //setup SPI2

#define SENSOR1_CS PB11 // some digital pin that you're using as the nCS pin

MagneticSensorSPI sensor = MagneticSensorSPI(MA730_SPI, SENSOR1_CS);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }


void setup() {
  Serial.begin(115200);
  while (!Serial);
  motor.useMonitoring(Serial);

  // Wire.setSCL(21);
  // Wire.setSDA(20);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 10;

  driver.init();
  sensor.init();

  // link driver
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // aligning voltage
  motor.voltage_sensor_align = 1;
  motor.current_limit = 2;   // [V] - if phase resistance not defined
  // motor.voltage_limit = 2;   // [V] - if phase resistance not defined
  motor.velocity_limit = 50; // [rad/s] cca 50rpm

  motor.PID_velocity.P = 0.05;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.LPF_velocity.Tf = 0.01;

  // current sense
  current_sense.init();
  // motor.linkCurrentSense(&current_sense);

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  // motor.controller = MotionControlType::torque;
  motor.controller = MotionControlType::velocity;
  // motor.controller = MotionControlType::velocity_openloop;

  // add current limit
  // motor.phase_resistance = 3.52 // [Ohm]
  // motor.current_limit = 2;   // [Amps] - if phase resistance defined

  // monitoring
  motor.monitor_downsample = 100; // set downsampling can be even more > 100
  motor.monitor_variables = _MON_VEL; // set monitoring of d and q current

  // initialize motor
  motor.init();

  // align sensor and start FOC
  // motor.initFOC(0, Direction::CW);
  Serial.print("Aligning...");
  motor.initFOC();

  // set the initial motor target
  // motor.target = 0.2; // Amps - if phase resistance defined
  // motor.target = 1; // Volts

  // add target command T
  // command.add('T', doTarget, "target current"); // - if phase resistance defined
  // command.add('T', doTarget, "target voltage");
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // motor.monitor();

  // user communication
  command.run();
}