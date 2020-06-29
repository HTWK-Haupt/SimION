#include <EEPROM.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include "norm.h"
#include "LM_RTL_V0_4c2.h"

// Teaching point table
#define POINTS_LEN  10
#define POINTS_WIDTH  5

// I2C activity, blue onboard LED
#define BLUE_LED_PIN     2
#define TOGGLE_LED  digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN));

#define GREEN_SIGNAL_PIN     27


// Stimulation output of C2
const byte trig_pin = 26;
//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Measuring main loop period
uint32_t cycle_time_ms = 0;
uint32_t current_ms = 0;
uint32_t last_ms = 0;

// Filter parameter of Hall sensors
// df_burstsize = 0->1 sample; 7->128 samples
// df_bw = 0->1 measure/sample; 12->4096 measures/sample; first measure takes 11 us, each additional 8.8 us
// df_iir = 0->FIR mode; 1->IIR mode
uint8_t si72_param[] = {0, 9, 0};

// x,y,z,mx,my,mz
float coord[6] = {0.001, 0.001, 0.001, 0.0, 0.0, 0.0};

// x,y,z
float tooltip[3] = {0.0};

// Teaching point table, n times x, y, z, distance to tooltip, distance to last
float points[POINTS_LEN][POINTS_WIDTH] = {8.8};
// Amount of valid entries of points[][]
uint8_t points_cnt = 0;
// Index of the closest entry of points[][] to the tooltip
uint8_t point_1_idx = 0;
// Index of the second closest entry of points[][] to the tooltip
uint8_t point_2_idx = 0;
// Distance in between tooltip and the line through p1 and p2
float distance = 1.0F;

bool restart = false;

// DAC test
#define DAC_PIN   25
uint8_t  sine_wave[256] = {
  0x80, 0x83, 0x86, 0x89, 0x8C, 0x90, 0x93, 0x96,
  0x99, 0x9C, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAE,
  0xB1, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF, 0xC1, 0xC4,
  0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8,
  0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8,
  0xEA, 0xEB, 0xED, 0xEF, 0xF0, 0xF1, 0xF3, 0xF4,
  0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB, 0xFC,
  0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFD,
  0xFD, 0xFC, 0xFB, 0xFA, 0xFA, 0xF9, 0xF8, 0xF6,
  0xF5, 0xF4, 0xF3, 0xF1, 0xF0, 0xEF, 0xED, 0xEB,
  0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC,
  0xDA, 0xD8, 0xD5, 0xD3, 0xD1, 0xCE, 0xCC, 0xC9,
  0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9, 0xB6, 0xB3,
  0xB1, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C,
  0x99, 0x96, 0x93, 0x90, 0x8C, 0x89, 0x86, 0x83,
  0x80, 0x7D, 0x7A, 0x77, 0x74, 0x70, 0x6D, 0x6A,
  0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52,
  0x4F, 0x4D, 0x4A, 0x47, 0x44, 0x41, 0x3F, 0x3C,
  0x39, 0x37, 0x34, 0x32, 0x2F, 0x2D, 0x2B, 0x28,
  0x26, 0x24, 0x22, 0x20, 0x1E, 0x1C, 0x1A, 0x18,
  0x16, 0x15, 0x13, 0x11, 0x10, 0x0F, 0x0D, 0x0C,
  0x0B, 0x0A, 0x08, 0x07, 0x06, 0x06, 0x05, 0x04,
  0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03,
  0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A,
  0x0B, 0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x15,
  0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x24,
  0x26, 0x28, 0x2B, 0x2D, 0x2F, 0x32, 0x34, 0x37,
  0x39, 0x3C, 0x3F, 0x41, 0x44, 0x47, 0x4A, 0x4D,
  0x4F, 0x52, 0x55, 0x58, 0x5B, 0x5E, 0x61, 0x64,
  0x67, 0x6A, 0x6D, 0x70, 0x74, 0x77, 0x7A, 0x7D
};

void IRAM_ATTR trigger() {
  //portENTER_CRITICAL_ISR(&mux);
  //interruptCounter++;
  //portEXIT_CRITICAL_ISR(&mux);
  for (uint16_t sig_idx = 255; sig_idx > 127; sig_idx--) {
    dacWrite(DAC_PIN, sine_wave[sig_idx]);
    delayMicroseconds(25);
  }
  for (uint16_t sig_idx = 127; sig_idx > 0; sig_idx--) {
    dacWrite(DAC_PIN, sine_wave[sig_idx] / 2 + 64);
    delayMicroseconds(50);
  }
}

void setup()
{
  // Initialize EEPROM with predefined size
  // Layout: isValidMarker, points[][], points_cnt 
  EEPROM.begin(POINTS_LEN * POINTS_WIDTH * sizeof(float) + 2);

  pinMode(BLUE_LED_PIN, OUTPUT);

  pinMode(trig_pin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(trig_pin), trigger, RISING);

  pinMode(GREEN_SIGNAL_PIN, OUTPUT);
  digitalWrite(GREEN_SIGNAL_PIN, HIGH);

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

  collision_point_distances();
  collision_two_closest_points();
  collision_line_distance();

  web_ui_eval();



  current_ms = millis();
  cycle_time_ms = current_ms - last_ms;
  last_ms = current_ms;
}