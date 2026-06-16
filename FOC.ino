void FOC_SETUPL() {





  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  motor.voltage_sensor_align = 1;

  //motor.foc_modulation = FOCModulationType::SinePWM;
  //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;  // Recommended for efficiency
  // motor.foc_modulation = FOCModulationType::Trapezoid_120;
  // motor.foc_modulation = FOCModulationType::Trapezoid_150;
  motor.controller = MotionControlType::torque;
  // set FOC loop to be used
  // MotionControlType::torque
  // MotionControlType::velocity
  // MotionControlType::angle
  // MotionControlType::angle_nocascade

  motor.useMonitoring(Serial);
  motor.init();
  //commect out for auto detect
  motor.sensor_direction = Direction::CCW;  // CW or CCW
  motor.zero_electric_angle = 4.19;

  motor.initFOC();
}





void FOC_SETUPR() {
  sensor1.init();
  sensor1.enableInterrupts(doA1, doB1, doC1);
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 12;
  motor1.voltage_sensor_align = 1;
  // motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.controller = MotionControlType::torque;
  motor1.useMonitoring(Serial);
  motor1.init();

  //commect out for auto detect
  motor1.sensor_direction = Direction::CCW;  // CW or CCW
  motor1.zero_electric_angle = 4.19;


  motor1.initFOC();
}




/*




void FOC_SETUPR() {
  sensor1.init();
  sensor1.enableInterrupts(doA1, doB1, doC1);
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 36;
  motor1.voltage_sensor_align = 1;


  motor.controller = MotionControlType::velocity;

  // velocity PI controller parameters
  // default P=0.5 I = 10
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  //default voltage_power_supply
  motor.voltage_limit = 6;
  motor.LPF_velocity.Tf = 0.01;


  motor1.useMonitoring(Serial);
  motor1.init();
  motor1.sensor_direction = Direction::CCW;  // CW or CCW
  motor1.initFOC();
}






*/



float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void FOC_Speed() {


  if (leftoutputraw > (1500 + DZ)) { leftoutput = mapFloat(leftoutputraw, (1500 + DZ), 2000, 0, MAXVolts); }
  if (leftoutputraw < (1500 - DZ)) { leftoutput = mapFloat(leftoutputraw, (1500 - DZ), 1000, 0, -MAXVolts); }
  if (leftoutputraw < (1500 + DZ) && leftoutputraw > (1500 - DZ)) {
    leftoutput = 0;
  }

  if (rightoutputraw > (1500 + DZ)) { rightoutput = mapFloat(rightoutputraw, (1500 + DZ), 2000, 0, MAXVolts); }
  if (rightoutputraw < (1500 - DZ)) { rightoutput = mapFloat(rightoutputraw, (1500 - DZ), 1000, 0, -MAXVolts); }
  if (rightoutputraw < (1500 + DZ) && rightoutputraw > (1500 - DZ)) {
    rightoutput = 0;
  }
  motor.move(rightoutput);
  motor1.move(leftoutput);
}



void FOC_telemetry() {

  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();
  PhaseCurrent_s currents1 = current_sense1.getPhaseCurrents();
  float current_magnitude1 = current_sense1.getDCCurrent();

  count++;
  if (count >= 300) {

    Serial.print(currents.a * 1000);  // milli Amps
    Serial.print("\t");
    Serial.print(currents.b * 1000);  // milli Amps
    Serial.print("\t");
    Serial.println(current_magnitude * 1000);  // milli Amps
    count = 0;
  }
}
