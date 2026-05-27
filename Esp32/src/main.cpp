/**
 * @brief Test de commandes des moteurs CubeMars par le bus CAN
 *
 * Recup de la doc PDF de CubeMars V2.0 :
 *      AK Series Module Driver Manual V1.0.18
 *
 * 1er tentative : utilisation du mode MIT (le plus simple d'après la vidéo)
 *
 */

#include <Arduino.h>

#include <ESP32_TWAI.h>

#define BOOT_PUSHBUTTON D9

const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -30.0f;
const float V_MAX = 30.0f;
const float T_MIN = -18.0f;
const float T_MAX = 18.0f;
const float Kp_MIN = 0;
const float Kp_MAX = 500.0f;
const float Kd_MIN = 0;
const float Kd_MAX = 5.0f;
const float Test_Pos = 0.0f;

#define CanTxMsg_t CanMsg
#define CanRxMsg_t CanMsg
#define can_msg_t CanMsg

#define MIT_CAN_ID 99

/**
 * Recup page 36
 *
 */
typedef enum {
  CAN_PACKET_SET_DUTY = 0,      // Duty Cycle Mode
  CAN_PACKET_SET_CURRENT,       // Current Loop Mode
  CAN_PACKET_SET_CURRENT_BRAKE, // Current Brake Mode
  CAN_PACKET_SET_RPM,           // RPM Mode
  CAN_PACKET_SET_POS,           // Position Mode
  CAN_PACKET_SET_ORIGIN_HERE,   // Set Origin Mode
  CAN_PACKET_SET_POS_SPD,       // Position-Velocity Loop Mode
  CAN_PACKET_SET_MIT = 8,       // MIT Mode
} CAN_PACKET_ID;

/**
 * @brief float_to_uint16
 *
 * All the numbers in the packet are converted to integer numbers by the
 * following function before they are sent to the motor.int
 * Converts a float to an unsigned int, given range and number of bits
 *
 *
 */
uint16_t float_to_uint16(float x, float x_min, float x_max, uint16_t bits) {
  float span = x_max - x_min;
  if (x < x_min)
    x = x_min;
  else if (x > x_max)
    x = x_max;
  return (uint16_t)((x - x_min) * ((float)((1 << bits) / span)));
}

/**
 * @brief uint16_to_float
 *
 * All numbers in the packet are converted to floating point by the following
 * function. converts unsigned int to float, given range and number of bits
 *
 *
 */
float uint16_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief
 *
 */
void buffer_append_int16(uint8_t *buffer, int16_t number, int16_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

/**
 * @brief
 *
 */
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

/**
 * @brief  Extended mode format ???
 *
 * The servo motor protocol is the can protocol, which uses the extended frame
 * format recup page et adapté à la lib : eyr1n/ESP32_TWAI
 */
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
  uint8_t i = 0;
  CanTxMsg_t txMessage;

  if (len > 8) {
    len = 8;
  }
  txMessage.id = CanExtendedId(0);
  // TxMessage.IDE = CAN_ID_EXT;
  // TxMessage.ExtId = id;
  // TxMessage.RTR = CAN_RTR_DATA;
  txMessage.data_length = len;
  for (i = 0; i < len; i++)
    txMessage.data[i] = data[i];

  CAN.write(txMessage); // CAN send TxMessage data
}

/**
 * @brief
 *
 * recup page 38
 */
void comm_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8),
                        buffer, send_index);
}

/**
 * @brief
 * recup page 39
 */
void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
                        buffer, send_index);
}

/**
 * @brief
 * recup page 40
 */
void comm_can_set_cb(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8),
                        buffer, send_index);
}

/**
 * @brief
 * recup page 40
 */
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8),
                        buffer, send_index);
}

/**
 * @brief
 * recup page 41
 */
void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8),
                        buffer, send_index);
}

/**
 * @brief
 * recup page 42
 */
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
  int32_t send_index = 1;
  uint8_t buffer;
  buffer = set_origin_mode;
  comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8),
                        &buffer, send_index);
}

/**
 * @brief
 * recup page 43
 */
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd,
                          int16_t RPA) {
  int32_t send_index = 0;
  int16_t send_index1 = 4;
  uint8_t buffer[8];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer, spd / 10.0, &send_index1);
  buffer_append_int16(buffer, RPA / 10.0, &send_index1);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8),
                        buffer, send_index1);
}

/**
 * @brief
 * recup page 45
 */
void motor_receive(float *motor_pos, float *motor_spd, float *motor_cur,
                   int8_t *motor_temp, int8_t *motor_error,
                   CanRxMsg_t *RxMessage) {
  int16_t pos_int = ((RxMessage)->data[0] << 8 | (RxMessage)->data[1]);
  int16_t spd_int = ((RxMessage)->data[2] << 8 | (RxMessage)->data[3]);
  int16_t cur_int = ((RxMessage)->data[4] << 8 | (RxMessage)->data[5]);
  *motor_pos = (float)(pos_int * 0.1f);  // motor position
  *motor_spd = (float)(spd_int * 10.0f); // motor velocity
  *motor_cur = (float)(cur_int * 0.01f); // motor current
  *motor_temp = (RxMessage)->data[6];    // motor temperature
  *motor_error = (RxMessage)->data[7];   // motor error code
}

/**
 * @brief CAN_Send_Msg : MIT Control Mode Transceiver Code Routine
 *
 * Transmission routine code:
 *
 * recup de la doc AK Series Module Driver Manual V1.0.18
 *
 */
uint8_t CAN_Send_Msg(uint8_t stdId, uint8_t len, can_msg_t *msg) {
  can_msg_t tx_message;
  uint8_t mbox;

  tx_message.id = CanStandardId(0x01); // Standard identifier
  // TxMessage.ExtId = 0x00;
  // Set extended identifier
  // TxMessage.IDE = CAN_Id_Standard; // Standard frame
  // TxMessage.RTR = CAN_RTR_Data;
  // Data frame
  tx_message.data_length = 8;
  // Data length to be sent
  for (uint8_t i = 0; i < len; i++)
    tx_message.data[i] = msg->data[i];
  mbox = CAN.write(tx_message);

  uint16_t i = 0;
  // while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i <
  // 0XFFF)) {
  //     i++; // Wait for transmission to complete
  // }
  if (i >= 0XFFF)
    return 1;
  return 0;
}

/**
 * @brief
 *
 * recup page 64
 */
void MIT_pack_cmd(can_msg_t *tx_message, float p_des, float v_des, float kp, float kd, float t_ff) {
  uint16_t p_int;
  uint16_t v_int;
  uint16_t kp_int;
  uint16_t kd_int;
  uint16_t t_int;

  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
  p_int = float_to_uint16(p_des, P_MIN, P_MAX, 16);
  v_int = float_to_uint16(v_des, V_MIN, V_MAX, 12);
  kp_int = float_to_uint16(kp, Kp_MIN, Kp_MAX, 12);
  kd_int = float_to_uint16(kd, Kd_MIN, Kd_MAX, 12);
  t_int = float_to_uint16(t_ff, T_MIN, T_MAX, 12);

  tx_message->data[0] = p_int >> 8;   // Position high 8 bits
  tx_message->data[1] = p_int & 0xFF; // Position low 8 bits
  tx_message->data[2] = v_int >> 4;   // Speed high 8 bits
  tx_message->data[3] =
      ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed low 4 bits, KP high 4 bits
  tx_message->data[4] = kp_int & 0xFF;      // KP low 8 bits
  tx_message->data[5] = kd_int >> 4;        // Kd high 8 bits
  tx_message->data[6] =
      ((kd_int & 0xF) << 4) | (t_int >> 8); // Kd low 4 bits, torque high 4 bits
  tx_message->data[7] = t_int & 0xFF;       // Torque low 8 bits

  CAN_Send_Msg(MIT_CAN_ID, 8, tx_message);
}

// Receive routine code :
float position;
float speed;
float torque;
float temperature;

/**
 * @brief unpack_reply : unpack ints from can buffer
 *
 * page 67
 *
 */
void unpack_reply(can_msg_t *rxMessage) {
  int id;
  uint16_t p_int;
  uint16_t v_int;
  uint16_t i_int;
  uint8_t t_int;

  id = rxMessage->data[0]; // driver id number
  p_int = (rxMessage->data[1] << 8) | rxMessage->data[2];
  v_int = (rxMessage->data[3] << 4) | (rxMessage->data[4] >> 4);
  // motor position data
  // motor speed value
  i_int = ((rxMessage->data[4] & 0xF) << 8) | rxMessage->data[5];
  t_int = rxMessage->data[6];
  /// convert ints to floats ///
  // float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  // float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  // float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  // float T = T_int;
  // motor torque value
  if (id == 1) {
    position = uint16_to_float(p_int, P_MIN, P_MAX, 16);
    speed = uint16_to_float(v_int, V_MIN, V_MAX, 12);
    torque = uint16_to_float(i_int, -T_MAX, T_MAX, 12);
    temperature = ((float)t_int) - 40.0f; // temperature range-40~215
  }
}

/**
 *
 */
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BOOT_PUSHBUTTON, INPUT);

  Serial.begin(115200);
  while (!Serial) {
  }
  Serial.println("\nStart CAN Sender application");

  // start the CAN bus at 500 kbps
  CAN.begin(CanBitRate::BR_500k);
}

typedef enum { APPUYE, RELACHE } state_t;
/**
 *
 */
void loop() {
  int n = 0;
  state_t state = RELACHE;
  can_msg_t msg;

  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(900);
    Serial.print("Start CAN Sender application loop n°");
    Serial.println(n++);

    switch (state) {
    case APPUYE:
      if (digitalRead(BOOT_PUSHBUTTON) == HIGH) {
        state = RELACHE;
        Serial.println("Bouton relaché");
        MIT_pack_cmd (&msg, 3.14, 0, 1, 1, 0);
      }
      break;

    case RELACHE:
    default:
      if (digitalRead(BOOT_PUSHBUTTON) == LOW) {
        state = APPUYE;
        Serial.println("Bouton appuyé");
        MIT_pack_cmd (&msg, -1.1, 0, 1, 1, 0);
      }
      break;
    }

    if (CAN.available()) {
      CanMsg msg = CAN.read();
      Serial.println(msg.getStandardId());
    }
  }
}

// End of file
