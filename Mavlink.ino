
void STARTUPMSG() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const char* name;

  if (ESC == 140) { name = "Hoverboard 1 Start"; }
  if (ESC == 141) { name = "Hoverboard 2 Start"; }
  if (ESC == 142) { name = "Hoverboard 3 Start"; }
  mavlink_msg_statustext_pack(system_id, ESC, &msg, 2, name, id, chunk_seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}



void MavLink_IN() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial.available()) {
    uint8_t c = Serial1.read();
    // Serial.print(c);
    //Get new message
    
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {

            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            HBWATCH();
            if (BASEMODE == 193) {
              armed = 1;
              //Serial.println("------------------------------------------------------------------------------------ARMED");
            }

            if (BASEMODE == 65) {
              // Serial.println("------------------------------------------------------------------------------------DISARMED");
              armed = 0;
            }
          }
          break;

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:  // #35
          {
            mavlink_servo_output_raw_t SERVOCHANNEL;
            mavlink_msg_servo_output_raw_decode(&msg, &SERVOCHANNEL);
            //leftoutputraw = (SERVOCHANNEL.servo1_raw);
            //rightoutputraw = (SERVOCHANNEL.servo2_raw);
                    // Serial.print(rightoutputraw);
            //Serial.println(leftoutputraw);
          }
          break;
      }
    }
  }
}


void MAVLINK_HB() {
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
  uint32_t custom_mode = 1;                  ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int type = MAV_TYPE_SERVO;
  // Pack the message
  // Serial.print("mavhb1");
  mavlink_msg_heartbeat_pack(1, ESC, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}



void Mavlink_Telemetry() {

  mavlink_message_t msg;
  uint32_t time_boot_ms = millis();
  const char* name;
  float value;

  if (ESC == 140) { name = "RPM_reqL1"; }
  if (ESC == 141) { name = "RPM_reqL2"; }
  if (ESC == 142) { name = "RPM_reqL3"; }
  value = (targetL);
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);


  if (ESC == 140) { name = "RPML1"; }
  if (ESC == 141) { name = "RPML2"; }
  if (ESC == 142) { name = "RPML3"; }
  value = velocityL;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);


  if (ESC == 140) { name = "VoltL1"; }
  if (ESC == 141) { name = "VoltL2"; }
  if (ESC == 142) { name = "VoltL3"; }
  value = voltageqL;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);

  if (ESC == 140) { name = "AMPL1"; }
  if (ESC == 141) { name = "AMPL2"; }
  if (ESC == 142) { name = "AMPL3"; }
  value = currentqL;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);



  if (ESC == 140) { name = "RPM_reqR1"; }
  if (ESC == 141) { name = "RPM_reqR2"; }
  if (ESC == 142) { name = "RPM_reqR3"; }
  value = (targetR);
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);


  if (ESC == 140) { name = "RPMR1"; }
  if (ESC == 141) { name = "RPMR2"; }
  if (ESC == 142) { name = "RPMR3"; }
  value = velocityR;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);


  if (ESC == 140) { name = "VoltR1"; }
  if (ESC == 141) { name = "VoltR2"; }
  if (ESC == 142) { name = "VoltR3"; }
  value = voltageqR;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);

  if (ESC == 140) { name = "AMPR1"; }
  if (ESC == 141) { name = "AMPR2"; }
  if (ESC == 142) { name = "AMPR3"; }
  value = currentqR;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  //Serial.print("V1: ");
  //Serial.println(VOLT1);

  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

