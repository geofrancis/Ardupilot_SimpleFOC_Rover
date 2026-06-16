
#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_servo_output_raw.h"
#include <SimpleFOC.h>
#include <esp_task_wdt.h>
TaskHandle_t motorTaskHandle;

#define ESC1

float MAXVolts = 8;

int RCLpin = 16;
int RCRpin = 17;

float Vlimit = 3;

int polepairs = 15;

int PWMpinRA = 32;
int PWMpinRB = 33;
int PWMpinRC = 25;

int PWMpinLA = 27;
int PWMpinLB = 26;
int PWMpinLC = 14;

int HALLpinRA = 23;
int HALLpinRB = 5;
int HALLpinRC = 15;

int HALLpinLA = 13;
int HALLpinLB = 19;
int HALLpinLC = 18;

int CURsenseLA = 39;
int CURsenseLB = 36;
int CURsenseRA = 35;
int CURsenseRB = 34;

int DZ = 20;  // dead

float target_velocity = 0;



int leftoutputraw = 1500;
int rightoutputraw = 1500;

unsigned long previousMillis = 0;
const long telem = 1000;

float leftoutput = 0;
float rightoutput = 0;

int DI1O = 0;
int FCHB = 1;
int FCOK = 0;

int BASEMODE = 0;
int armed;
int active = 0;

float targetL = 0;
float velocityL = 0;
float voltageqL = 0;
float currentqL = 0;

float targetR = 0;
float velocityR = 0;
float voltageqR = 0;
float currentqR = 0;


uint8_t system_id = 1;
uint8_t component_id = 158;
uint8_t severity = 1;
uint16_t id = 0;
uint8_t chunk_seq = 0;

#define GRID_ROWS 19  // Odd number works best so there is an exact middle row
#define GRID_COLS 60  // Horizontal width (number of time samples)

// Buffers to hold sample history
float sample_history[GRID_COLS] = { 0 };
int sample_index = 0;
int count = 0;



HallSensor sensor = HallSensor(HALLpinRA, HALLpinRB, HALLpinRC, polepairs);
BLDCMotor motor = BLDCMotor(polepairs);
BLDCDriver3PWM driver = BLDCDriver3PWM(PWMpinRA, PWMpinRB, PWMpinRC, polepairs);
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, CURsenseLA, CURsenseLB);


HallSensor sensor1 = HallSensor(HALLpinLA, HALLpinLB, HALLpinLC, polepairs);
BLDCMotor motor1 = BLDCMotor(polepairs);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PWMpinLA, PWMpinLB, PWMpinLC, polepairs);
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01f, 50.0f, CURsenseRA, CURsenseRB);


#ifdef ESC1
int ESC = 140;  //board number
#endif

#ifdef ESC2
int ESC = 141;  //board number
#endif

#ifdef ESC3
int ESC = 142;  //board number
#endif



void doA() {
  sensor.handleA();
}
void doB() {
  sensor.handleB();
}
void doC() {
  sensor.handleC();
}

void doA1() {
  sensor1.handleA();
}
void doB1() {
  sensor1.handleB();
}
void doC1() {
  sensor1.handleC();
}


void PWM(void* pvParameters) {
  pinMode(RCLpin, INPUT);
  pinMode(RCRpin, INPUT);
  vTaskDelay(4000);

  for (;;) {
    leftoutputraw = pulseIn(RCLpin, HIGH);
    rightoutputraw = pulseIn(RCRpin, HIGH);
    if (leftoutputraw > 900 && leftoutputraw < 2100) {
      if (rightoutputraw > 900 && rightoutputraw < 2100) {
        FOC_Speed();
        //Serial.print(leftoutputraw);
        //Serial.print("   ");
        //Serial.println(rightoutputraw);
      }
    } else {
      // Serial.print("MISSING RC INPUT");
      motor.move(0);
      motor1.move(0);
      vTaskDelay(1000);
    }
    vTaskDelay(50);
  }
}




void Left_task(void* pvParameters) {
  FOC_SETUPL();
  for (;;) {
    // noInterrupts();
    motor.loopFOC();
    // interrupts();
    // vTaskDelay(1);
    //motor.move(1);
  }
}


void Right_task(void* pvParameters) {
  FOC_SETUPR();
  for (;;) {
    // noInterrupts();
    motor1.loopFOC();
    // interrupts();
    //vTaskDelay(1);
    //motor1.move(1);
  }
}




void setup() {
  Serial.begin(500000);  //Main serial port for console output
  //Serial1.begin(230400, SERIAL_8N1, 16, 17);
  //Serial2.begin(38400, SERIAL_8N1, 2, 4);  //GPS+AIS

  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 5000,
    .idle_core_mask = (1 << 1),  // Bitmask: Bit 1 is Core 1. We leave Bit 0 (Core 0) unset.
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&twdt_config);
  SimpleFOCDebug::enable(&Serial);
  current_sense.init();
  current_sense1.init();
  driver.init();
  driver1.init();
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);



  xTaskCreatePinnedToCore(
    PWM,               // Function to implement the task
    "RC pwm",          // Name of the task
    10000,             // Stack size in words (10k is usually plenty)
    NULL,              // Task input parameter
    2,                 // Priority of the task (1 is higher than 0)
    &motorTaskHandle,  // Task handle
    1                  // Core where the task should run (Core 0)
  );


  xTaskCreatePinnedToCore(
    Left_task,         // Function to implement the task
    "Left Motor",      // Name of the task
    10000,             // Stack size in words (10k is usually plenty)
    NULL,              // Task input parameter
    5,                 // Priority of the task (1 is higher than 0)
    &motorTaskHandle,  // Task handle
    0                  // Core where the task should run (Core 0)
  );


  xTaskCreatePinnedToCore(
    Right_task,        // Function to implement the task
    "Right Motor",     // Name of the task
    10000,             // Stack size in words (10k is usually plenty)
    NULL,              // Task input parameter
    5,                 // Priority of the task (1 is higher than 0)
    &motorTaskHandle,  // Task handle
    0                  // Core where the task should run (Core 0)
  );



  STARTUPMSG();
}


void loop() {
  vTaskDelay(1);
  // motor.monitor();
  //motor1.monitor();
  MavLink_IN();
  //FOC_Speed();
  //motor.move(1);
  //motor1.move(1);
  //FOC_telemetry();

  //MAVLINK_HB();


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= telem) {
    previousMillis = currentMillis;
    MAVLINK_HB();
    //Mavlink_Telemetry();
    // MAVLINK_ESC_1();

    if (DI1O == 1) { Mavlink_Telemetry(); }
    // if (DI1O == 2) { FCHBC(); }
    // if (DI1O == 3) { FOC_telemetry(); }
    // if (DI1O == 4) { sleepcheck(); }
    DI1O++;
    if (DI1O > 4) { DI1O = 1; }
    // Serial.print("                               DI1O  ");
    // Serial.println(DI1O);
  }
}
