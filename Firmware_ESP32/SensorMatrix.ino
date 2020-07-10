// Initialise the sensor matrix an measure the earth magnetic field for compensation
void sensor_matrix_init()
{
  int16_t data = 0;
  
  pinMode(14, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  // First measure takes 11 us, each additional 8.8 us
  const unsigned int measuring_time_us = 9 /*us*/ * (1 << 12) /*number of measures*/ + 20 /*us*/;
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

  
}

// Set Hall-Sensor no. 1...27 active
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

// Perform a measuring cycle with serial raw data output
void sensor_matrix_measure()
{
  int16_t data = 0;
    
  // Send a frame of raw binary measurement data, start with a special marker
  Serial.write(0x00);
  Serial.write(0x80);
  for (uint8_t i = 0; i < SENSOR_MAX; i++)
  {
    sensor_matrix_set(i);
    si72_wake_up();
    si72_write_register(SI72_POWER_CTRL, SI72_ONEBURST_MASK);
    // First measure takes 11 us, each additional 8.8 us
    delayMicroseconds(9 /*us*/ * (1 << si72_param[1]) /*number of measures*/ + 20 /*us*/);
    si72_read_data(&data);
    data -=  b_offset[i];
    // Output binary raw data for external  processing with MATLAB
    Serial.write((uint8_t)data);
    Serial.write((uint8_t)(data >> 8));
    b_measure[i] = 1.25e-6F * (float)data;
  }
}
