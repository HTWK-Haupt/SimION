#include <EEPROM.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include "norm.h"
#include "LM_RTL_V0_4c2.h"
#include "r_s.h"

// EMG signal given to the neuromonitor
#include "emg.h"

// Sensor matrix defines
#define SENSOR_MAX  27
// Enable sensor no. 1...16 -> D23 AND NOT D14
#define ENABLE_SENSOR_1_16  digitalWrite(23, true); digitalWrite(14, false);
// Enable sensor no. 17...27 -> NOT D23 AND D14
#define ENABLE_SENSOR_17_27  digitalWrite(14, true); digitalWrite(23, false);

// Teaching point table
#define POINTS_LEN  255
#define POINTS_WIDTH  4

// I2C activity, blue onboard LED
#define BLUE_LED_PIN     2
#define TOGGLE_LED  digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN));

// Switching the stimulation path to low impedance causes a green signal on the neuromonitors screen
#define GREEN_SIGNAL_PIN     27

// Rectangular conditioned stimulation output from the neuromonitor
#define TRIGGER_PIN 26
// DAC output to the neuromonitor
#define DAC_PIN   25

// EMG related constants
#define INC_SIG_NOISE 2
#define INC_DELAY_US 300
#define INC_THRESHOLD 0.05E-3F

// Threshold to decide if magnet 1 or magnet 2 is used
#define MAGNETIC_THRESHOLD 25.0E-3F

// Measuring main loop period
uint32_t cycle_time_ms = 0;

// Filter parameter of Hall sensors
// df_burstsize = 0->1 sample; 7->128 samples
// df_bw = 0->1 measure/sample; 12->4096 measures/sample; first measure takes 11 us, each additional 8.8 us
// df_iir = 0->FIR mode; 1->IIR mode
uint8_t si72_param[] = {0, 9, 0};

// Triggers reinitalisation of the sensor matrix and thus a new offset compensation too
bool restart = false;

// Magnetic field density in T
uint16_t b_offset[SENSOR_MAX] = {0};
float b_measure[SENSOR_MAX] = {0.0F};

// Raw coordinates of the magnetic dipols center. x,y,z in meter, mx,my,mz in Am^2
float coord[6] = {0.001F, 0.001F, 0.001F, 0.0F, 0.0F, 0.0F};
float mm_abs = 0.0F; // Absolute value of the magnetic moment in Am^2
// Half length of the magnet in meter for the calculation of the tooltip position
const float magn_len_half_1 = 3.0E-3F; // about 43E-3 Am^2
const float magn_len_half_2 = 1.5E-3F; // about 10E-3 Am^2
// Actual position of the stimulation probes tip. x,y,z in meter
float tooltip[3] = {0.0F};

// Teaching point table, n times x, y, z, actual distance to tooltip, all in meter
float points[POINTS_LEN][POINTS_WIDTH] = {0.0F};
// Amount of valid entries of points[][]
uint8_t points_cnt = 0;
// Index of the closest entry of points[][] to the tooltip
uint8_t point_1_idx = 0;
// Index of the second closest entry of points[][] to the tooltip
uint8_t point_2_idx = 0;
// Distance in between tooltip and the line through p1 and p2 in meter
float distance = 0.005F;
float threshold = 0.6E-3F; // Default trigger threshold in meter
// Default EMG signal parameter
volatile int emg_signal_100 = 70;
volatile int emg_delay_us = 3000;
volatile int emg_noise_100 = 6;
volatile bool emg_trigger = false;
volatile bool emg_permanent = true;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Catching the stimulation impulse coming from the neuromonitor
void IRAM_ATTR trigger() {
  portENTER_CRITICAL_ISR(&mux);
  if (emg_trigger) {
    for (uint16_t i = 0; i < emg_delay_us / EMG_SAMPLING_RATE_US; i++) {
      dacWrite(DAC_PIN, random(255 * emg_noise_100 / 100));
      delayMicroseconds(EMG_SAMPLING_RATE_US);
    }
    for (uint16_t sig_idx = 0; sig_idx < EMG_LEN; sig_idx++) {
      dacWrite(DAC_PIN, emg[sig_idx] * emg_signal_100 / 100 + random(255 * emg_noise_100 / 100));
      delayMicroseconds(EMG_SAMPLING_RATE_US);
    }
    for (uint16_t i = 0; i < 43000 / EMG_SAMPLING_RATE_US; i++) {
      dacWrite(DAC_PIN, random(255 * emg_noise_100 / 100));
      delayMicroseconds(EMG_SAMPLING_RATE_US);
    }
    emg_trigger = false;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void setup()
{
  // Initialize EEPROM with predefined size
  // Layout: isValidMarker, points[][], points_cnt
  EEPROM.begin(POINTS_LEN * POINTS_WIDTH * sizeof(float) + 2);

  randomSeed(analogRead(0));

  pinMode(BLUE_LED_PIN, OUTPUT);

  pinMode(TRIGGER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), trigger, RISING);

  pinMode(GREEN_SIGNAL_PIN, OUTPUT);

  // Biasing DAC
  dacWrite(DAC_PIN, 0);

  Wire.begin();
  Serial.begin(115200);
  web_ui_init();

  TOGGLE_LED
  sensor_matrix_init();
  TOGGLE_LED

  collision_points_restore();
}

void loop()
{
  static uint32_t last_ms = 0;

  // Data aquisition ////////////////////////////////////////////////////

  uint32_t current_ms = millis();
  cycle_time_ms = current_ms - last_ms;
  last_ms = current_ms;

  TOGGLE_LED
  if (restart) {
    sensor_matrix_init();
    restart = false;
    coord[0] = 0.001;
    coord[1] = 0.001;
    coord[2] = 0.001;
    coord[3] = 0.0;
    coord[4] = 0.0;
    coord[5] = 0.0;
    //Serial.println(si72_param[1]);
  }
  sensor_matrix_measure();
  TOGGLE_LED

  // Data processing /////////////////////////////////////////////////

  // Run Levenberg-Marquardt algorithym, using the last coordinates as starting point
  LM_RTL_V0_4c2((const float*)coord, r_s, b_measure, coord);
  //Serial.println(String(coord[0] * 1.0E3F) + String(" ") + String(coord[1] * 1.0E3F) + String(" ") + String(coord[2] * 1.0E3F));

  // Calculate tooltip position

  // Create normalised direction vector from magnetic moment vector
  mm_abs = b_norm((const float*)&coord[3]) + 1.0E-12F; // Prevent division by zero
  if (MAGNETIC_THRESHOLD <= mm_abs) {
    tooltip[0] = -magn_len_half_1 * coord[3] / mm_abs + coord[0];
    tooltip[1] = -magn_len_half_1 * coord[4] / mm_abs + coord[1];
    tooltip[2] = -magn_len_half_1 * coord[5] / mm_abs + coord[2];
  }
  else {
    tooltip[0] = -magn_len_half_2 * coord[3] / mm_abs + coord[0];
    tooltip[1] = -magn_len_half_2 * coord[4] / mm_abs + coord[1];
    tooltip[2] = -magn_len_half_2 * coord[5] / mm_abs + coord[2];
  }
  //Serial.println(String(tooltip[0] * 1.0E3F) + String(" ") + String(tooltip[1] * 1.0E3F) + String(" ") + String(tooltip[2] * 1.0E3F));
  //Serial.println("");


  collision_calc_distance();

  if (emg_permanent || (threshold >= distance)) {
    digitalWrite(GREEN_SIGNAL_PIN, HIGH);
    portENTER_CRITICAL_ISR(&mux);
    emg_trigger = true;
    portEXIT_CRITICAL_ISR(&mux);
  }
  else {
    digitalWrite(GREEN_SIGNAL_PIN, LOW);
  }

  // Output ///////////////////////////////////////////////////////////

  web_ui_eval();
}
