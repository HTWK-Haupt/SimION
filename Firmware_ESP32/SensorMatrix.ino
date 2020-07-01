#define SENSOR_MAX  27

// Enable sensor no. 1...16 -> D23 AND NOT D14
#define ENABLE_SENSOR_1_16  digitalWrite(23, true); digitalWrite(14, false);

// Enable sensor no. 17...27 -> NOT D23 AND D14
#define ENABLE_SENSOR_17_27  digitalWrite(14, true); digitalWrite(23, false);

int16_t data = 0;
uint16_t b_offset[SENSOR_MAX] = {0};
float b_measure[SENSOR_MAX] = {0.0};
unsigned int measuring_time_us = 0;

void sensor_matrix_init()
{
  pinMode(14, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  // First measure takes 11 us, each additional 8.8 us
  measuring_time_us = 9 /*us*/ * (1 << 12) /*number of measures*/ + 20 /*us*/;
  //Serial.println(measuring_time_us);

  for (uint8_t i = 0; i < SENSOR_MAX; i++)
  {
    sensor_matrix_set(i);
    si72_wake_up();
    si72_write_register(SI72_ARAUTOINC, 0x01); // Activate autoincrement
    si72_write_register(SI72_CTRL4, (si72_param[0] << 5) | (12 << 1) | si72_param[2]); // Set FIR filter to max. (4096 measures per sample)
    si72_write_register(SI72_POWER_CTRL, SI72_ONEBURST_MASK); // Take an empty shot
    delayMicroseconds(measuring_time_us);
    si72_read_data(&data);
    si72_write_register(SI72_POWER_CTRL, SI72_ONEBURST_MASK); // Measure the offset value
    delayMicroseconds(measuring_time_us);
    si72_read_data(&data);
    b_offset[i] = data;
    si72_write_register(SI72_CTRL4, (si72_param[0] << 5) | (si72_param[1] << 1) | si72_param[2]); // Set user filter configuration
  }

  //  for(uint8_t i = 0; i < SENSOR_MAX; i++)
  //  {
  //    Serial.print(1.25e-3*(double)b_offset[i]);
  //    Serial.print(" ");
  //  }
  //  Serial.println("");

  // First measure takes 11 us, each additional 8.8 us
  measuring_time_us = 9 /*us*/ * (1 << si72_param[1]) /*number of measures*/ + 20 /*us*/;
  //Serial.println(measuring_time_us);
}

// Set Hall-Sensor no. 1...27 aktive
void sensor_matrix_set(uint8_t num)
{
  num++;
  // 4 bit address bus D16...19
  if (num >= 1 && num <= 16)
  {
    ENABLE_SENSOR_1_16
    for (uint8_t i = 0; i < 4; i++)
      digitalWrite(16 + i, 1 & ((num - 1) >> i));
  }
  else if (num >= 17 && num <= 27)
  {
    ENABLE_SENSOR_17_27
    for (uint8_t i = 0; i < 4; i++)
      digitalWrite(16 + i, 1 & ((num - 17) >> i));
  }
}

void sensor_matrix_measure()
{
  // Send a frame of raw binary measurement data, start with a special marker
  Serial.write(0x00);
  Serial.write(0x80);
  for (uint8_t i = 0; i < SENSOR_MAX; i++)
  {
    sensor_matrix_set(i);
    si72_wake_up();
    si72_write_register(SI72_POWER_CTRL, SI72_ONEBURST_MASK);
    delayMicroseconds(measuring_time_us);
    si72_read_data(&data);
    data -=  b_offset[i];
    Serial.write((uint8_t)data);
    Serial.write((uint8_t)(data >> 8));
    b_measure[i] = 1.25e-6F * (float)data;
  }

  // Run Levenberg-Marquardt algorithym, using the last coordinates as starting point
  LM_RTL_V0_4c2((const float*)coord, r_s, b_measure, coord);
  //Serial.println(String(coord[0] * 1.0E3F) + String(" ") + String(coord[1] * 1.0E3F) + String(" ") + String(coord[2] * 1.0E3F));

  // Calculate tooltip position

  // Create normalised direction vector from magnetic moment vector
  float magn_len_half = 3.0E-3F; // Meter
  float mm_abs = b_norm((const float*)&coord[3]) + 1.0E-12F; // Prevent division by zero
  tooltip[0] = -magn_len_half * coord[3] / mm_abs + coord[0];
  tooltip[1] = -magn_len_half * coord[4] / mm_abs + coord[1];
  tooltip[2] = -magn_len_half * coord[5] / mm_abs + coord[2];
  //Serial.println(String(tooltip[0] * 1.0E3F) + String(" ") + String(tooltip[1] * 1.0E3F) + String(" ") + String(tooltip[2] * 1.0E3F));
  //Serial.println("");
}
