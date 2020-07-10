#include <EEPROM.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include "norm.h"
#include "LM_RTL_V0_4c2.h"
#include "r_s.h"

// EMG signals given to the neuromonitor in dependency if the stimulation signal is below, equal or upper the threshold
#include "emg_below.h"
#include "emg_equal.h"
#include "emg_upper.h"
#define EMG_LEN 1024
#define EMG_TIME_MS 4
#define EMG_OFFSET 0x27




// Sensor matrix defines
#define SENSOR_MAX  27
// Enable sensor no. 1...16 -> D23 AND NOT D14
#define ENABLE_SENSOR_1_16  digitalWrite(23, true); digitalWrite(14, false);
// Enable sensor no. 17...27 -> NOT D23 AND D14
#define ENABLE_SENSOR_17_27  digitalWrite(14, true); digitalWrite(23, false);

// Teaching point table
#define POINTS_LEN  20
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
float b_measure[SENSOR_MAX] = {0.0};

// Raw coordinates of the magnetic dipols center. x,y,z in meter, mx,my,mz in Am^2
float coord[6] = {0.001, 0.001, 0.001, 0.0, 0.0, 0.0};

// Half length of the magnet in meter for the calculation of the tooltip position
float magn_len_half = 3.0E-3F;

// Actual position of the stimulation probes tip. x,y,z in meter
float tooltip[3] = {0.0};

// Teaching point table, n times x, y, z, actual distance to tooltip, all in meter
float points[POINTS_LEN][POINTS_WIDTH] = {0.0};
// Amount of valid entries of points[][]
uint8_t points_cnt = 0;
// Index of the closest entry of points[][] to the tooltip
uint8_t point_1_idx = 0;
// Index of the second closest entry of points[][] to the tooltip
uint8_t point_2_idx = 0;
// Distance in between tooltip and the line through p1 and p2 in meter
float distance = 0.005F;

// Catching the stimulation impulse coming from the neuromonitor
void IRAM_ATTR trigger() {
  // TODO: parametric threshold
  //if (0.001 > distance) {

  // TODO: parametric delay
  delayMicroseconds(3000);

  for (uint16_t sig_idx = 0; sig_idx < 446; sig_idx++) {
    // TODO: parametric gain
    dacWrite(DAC_PIN, 0.2 * (emg_upper[sig_idx] - emg_upper[0]));
    delayMicroseconds(EMG_TIME_MS * 1000 / EMG_LEN);
  }


  //}
}

void setup()
{
  // Initialize EEPROM with predefined size
  // Layout: isValidMarker, points[][], points_cnt
  EEPROM.begin(POINTS_LEN * POINTS_WIDTH * sizeof(float) + 2);

  pinMode(BLUE_LED_PIN, OUTPUT);

  pinMode(TRIGGER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), trigger, RISING);

  // TODO: depends on proper stimulation
  pinMode(GREEN_SIGNAL_PIN, OUTPUT);
  digitalWrite(GREEN_SIGNAL_PIN, HIGH);

  // Biasing DAC
  dacWrite(DAC_PIN, 0);

  Wire.begin();
  Serial.begin(115200);
  web_ui_init();

  TOGGLE_LED
  sensor_matrix_init();
  TOGGLE_LED

  //collision_points_restore();
}

void loop()
{
  uint32_t current_ms = 0;
  uint32_t last_ms = 0;

  // Data aquisition ////////////////////////////////////////////////////

  current_ms = millis();
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
  float mm_abs = b_norm((const float*)&coord[3]) + 1.0E-12F; // Prevent division by zero
  tooltip[0] = -magn_len_half * coord[3] / mm_abs + coord[0];
  tooltip[1] = -magn_len_half * coord[4] / mm_abs + coord[1];
  tooltip[2] = -magn_len_half * coord[5] / mm_abs + coord[2];
  //Serial.println(String(tooltip[0] * 1.0E3F) + String(" ") + String(tooltip[1] * 1.0E3F) + String(" ") + String(tooltip[2] * 1.0E3F));
  //Serial.println("");


  collision_calc_distance();

  // Output ///////////////////////////////////////////////////////////

  web_ui_eval();
}
