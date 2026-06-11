#include <Arduino.h>
#include "driver/twai.h"

// ─── Pins & IDs ───────────────────────────────────────────────
#define BOOT_PUSHBUTTON D9
#define CAN_TX_PIN      21
#define CAN_RX_PIN      22
#define CAN_ID          69      // ID du moteur

// ─── Paramètres de mouvement ──────────────────────────────────
#define SPEED_ERPM      5000
#define ACCEL_ERPM_S    10000
#define STEP_DEG        45.0f

// ─── Enum des modes CAN (repris du prof, doc page 36) ─────────
typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_SET_ORIGIN_HERE,
  CAN_PACKET_SET_POS_SPD,
  CAN_PACKET_SET_MIT = 8,
} CAN_PACKET_ID;

// ─── Machine à états bouton (repris du prof) ──────────────────
typedef enum { APPUYE, RELACHE } state_t;
state_t state = RELACHE;

// ─── Variables globales ───────────────────────────────────────
static bool     driver_installed = false;
float           current_position = 0.0f;

// ─────────────────────────────────────────────────────────────
// Utilitaires d'encodage big-endian (repris du prof, doc page 37)
// ─────────────────────────────────────────────────────────────
void buffer_append_int16(uint8_t *buffer, int16_t number, int16_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

// ─────────────────────────────────────────────────────────────
// Envoi d'une trame CAN étendue (repris du prof, corrigé)
// Correction : suppression du flag RTR incompatible avec DATA
// ─────────────────────────────────────────────────────────────
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
  if (len > 8) len = 8;

  twai_message_t tx_message;
  tx_message.flags              = TWAI_MSG_FLAG_EXTD;  // ← corrigé (plus de RTR)
  tx_message.identifier         = id;
  tx_message.data_length_code   = len;
  for (uint8_t i = 0; i < len; i++)
    tx_message.data[i] = data[i];

  if (twai_transmit(&tx_message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("Trame transmise avec succès");
  } else {
    Serial.println("Échec de transmission");
  }
}

// ─────────────────────────────────────────────────────────────
// Commande Position-Velocity (doc page 43)
// pos_deg : position cible en degrés
// spd     : vitesse en ERPM
// acc     : accélération en ERPM/s²
// ─────────────────────────────────────────────────────────────
void comm_can_set_pos_spd(uint8_t controller_id, float pos_deg,
                          int16_t spd, int16_t acc) {
  int32_t send_index  = 0;
  int16_t send_index1 = 4;
  uint8_t buffer[8];

  // Position : int32 = degrés * 10000  (corrigé vs code prof : * 1000)
  buffer_append_int32(buffer, (int32_t)(pos_deg * 10000.0f), &send_index);

  // Vitesse : int16 = ERPM / 10
  buffer_append_int16(buffer, (int16_t)(spd / 10), &send_index1);

  // Accélération : int16 = ERPM/s² / 10
  buffer_append_int16(buffer, (int16_t)(acc / 10), &send_index1);

  comm_can_transmit_eid(
    controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8),
    buffer, send_index1
  );

  Serial.printf("Commande envoyée → pos=%.1f° | spd=%d ERPM | acc=%d ERPM/s²\n",
                pos_deg, spd, acc);
}

// ─────────────────────────────────────────────────────────────
// Lecture et affichage du feedback moteur (doc page 44-45)
// ─────────────────────────────────────────────────────────────
void handle_rx_message(twai_message_t &message) {

  // On extrait le Function ID et le Motor ID depuis l'identifiant étendu
  uint8_t  motor_id    = message.identifier & 0xFF;
  uint32_t function_id = (message.identifier >> 8) & 0xFFFF;

  // On ne traite que le feedback servo (0x29) de notre moteur
  if (function_id != 0x29 || motor_id != CAN_ID) {
    Serial.printf("Trame ignorée — function_id=0x%02X motor_id=%d\n",
                  function_id, motor_id);
    return;
  }

  // Décodage des 8 octets
  int16_t pos_int = ((int16_t)message.data[0] << 8) | message.data[1];
  int16_t spd_int = ((int16_t)message.data[2] << 8) | message.data[3];
  int16_t cur_int = ((int16_t)message.data[4] << 8) | message.data[5];
  int8_t  temp    = (int8_t)message.data[6];
  uint8_t error   = message.data[7];

  // Conversion en unités physiques
  float pos_deg  = pos_int * 0.1f;
  float spd_erpm = spd_int * 10.0f;
  float cur_A    = cur_int * 0.01f;

  Serial.println("─── Feedback moteur ───────────────────");
  Serial.printf("  Position    : %.1f °\n",    pos_deg);
  Serial.printf("  Vitesse     : %.0f ERPM\n", spd_erpm);
  Serial.printf("  Courant     : %.2f A\n",    cur_A);
  Serial.printf("  Température : %d °C\n",     temp);
  Serial.printf("  Erreur      : %d %s\n",     error,
    error == 0 ? "(OK)" :
    error == 1 ? "(Surchauffe moteur)" :
    error == 2 ? "(Surcourant)" :
    error == 3 ? "(Surtension)" :
    error == 4 ? "(Sous-tension)" :
    error == 5 ? "(Encodeur)" :
    error == 6 ? "(Surchauffe MOSFET)" :
    error == 7 ? "(Moteur bloqué)" : "(Inconnu)");
  Serial.println("───────────────────────────────────────");
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(BOOT_PUSHBUTTON, INPUT_PULLUP);  // ← corrigé (pull-up interne)

  Serial.println("Initialisation du bus CAN...");

  // Correction critique : 1 Mbps obligatoire (doc page 8)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
//  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS(); // ← corrigé (500k → 1M)
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS(); // ← corrigé (500k → 1M)
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("ERREUR : installation driver TWAI");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("ERREUR : démarrage driver TWAI");
    return;
  }

  // Alertes utiles pour le debug (repris du prof)
  uint32_t alerts =   TWAI_ALERT_RX_DATA       |
                      TWAI_ALERT_TX_SUCCESS     |
                      TWAI_ALERT_TX_FAILED      |
                      TWAI_ALERT_BUS_ERROR      |
                      TWAI_ALERT_ERR_PASS       |
                      TWAI_ALERT_RX_QUEUE_FULL;

  if (twai_reconfigure_alerts(alerts, NULL) != ESP_OK) {
    Serial.println("ERREUR : configuration alertes TWAI");
    return;
  }

  driver_installed = true;
  Serial.println("Bus CAN prêt à 1 Mbps");
  Serial.println("Appuie sur le bouton pour +45°");
}

// ─────────────────────────────────────────────────────────────
void loop() {
  if (!driver_installed) { delay(1000); return; }

  // ── Lecture des alertes TWAI (repris du prof) ──────────────
  uint32_t alerts;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(10));
  twai_status_info_t status;
  twai_get_status_info(&status);

  if (alerts & TWAI_ALERT_ERR_PASS)
    Serial.println("Alerte : contrôleur TWAI en mode passif erreur");

  if (alerts & TWAI_ALERT_BUS_ERROR)
    Serial.printf("Alerte : erreur bus CAN — count=%" PRIu32 "\n",
                  status.bus_error_count);

  if (alerts & TWAI_ALERT_TX_FAILED)
    Serial.printf("Alerte : échec TX — failed=%" PRIu32 "\n",
                  status.tx_failed_count);

  if (alerts & TWAI_ALERT_TX_SUCCESS)
    Serial.println("Alerte : TX succès");

  if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
    Serial.println("Alerte : queue RX pleine, trame perdue");

  // ── Réception des trames feedback ──────────────────────────
  if (alerts & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);  // ← décommenté et branché
    }
  }

  // ── Machine à états bouton (repris du prof) ─────────────────
  switch (state) {
    case RELACHE:
      if (digitalRead(BOOT_PUSHBUTTON) == LOW) {
        delay(50);  // anti-rebond
        if (digitalRead(BOOT_PUSHBUTTON) == LOW) {
          state = APPUYE;
          current_position += STEP_DEG;
          comm_can_set_pos_spd(CAN_ID, current_position,
                               SPEED_ERPM, ACCEL_ERPM_S);
          Serial.printf("Bouton appuyé → cible = %.1f°\n", current_position);
        }
      }
      break;

    case APPUYE:
      if (digitalRead(BOOT_PUSHBUTTON) == HIGH) {
        state = RELACHE;
        Serial.println("Bouton relâché");
      }
      break;
  }
}