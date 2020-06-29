// Calculate the distances from the tooltip to all collision points and populate the 4. column of the points table
void collision_point_distances(void)
{
  float diff[3];
  for (uint8_t i = 0; i < points_cnt; i++)
  {
    diff[0] = tooltip[0] - points[i][0];
    diff[1] = tooltip[1] - points[i][1];
    diff[2] = tooltip[2] - points[i][2];
    points[i][3] = b_norm(diff);
  }
}

// Find the two closest points and save their indices
void collision_two_closest_points(void)
{
  float dist_1 = points[0][3];
  point_1_idx = 0;
  point_2_idx = 0;

  if (points_cnt >= 2) {
    // Search p1 from 0 to end
    for (uint8_t i = 1; i < points_cnt; i++)
    {
      if (points[i][3] < dist_1) {
        point_1_idx = i;
        dist_1 = points[i][3];
      }
    }
    // Search p2 in the neighborhood of p1
    if (point_1_idx == 0)
      point_2_idx = 1;
    else if (point_1_idx == points_cnt - 1)
      point_2_idx = point_1_idx - 1;
    else if (points[point_1_idx - 1][3] < points[point_1_idx + 1][3])
      point_2_idx = point_1_idx - 1;
    else
      point_2_idx = point_1_idx + 1;
  }
}

// Calculate the distance between the tooltip and the line p1-p2
void collision_line_distance(void)
{
  // Search the highest index of the closest points to get the right distance
  if (point_1_idx > point_2_idx) {
    // (a + b + c) / 2
    float s = (points[point_1_idx][3] + points[point_2_idx][3] + points[point_1_idx][4]) / 2.0F;
    // 2 / c * sqrt(s * (s - a) * (s - b) * (s - c))
    distance = 2.0F / points[point_1_idx][4] * sqrt(s * (s - points[point_1_idx][3]) * (s - points[point_2_idx][3]) * (s - points[point_1_idx][4]));
  }
  else if (point_2_idx > point_1_idx) {
    // (a + b + c) / 2
    float s = (points[point_1_idx][3] + points[point_2_idx][3] + points[point_2_idx][4]) / 2.0F;
    // 2 / c * sqrt(s * (s - a) * (s - b) * (s - c))
    distance = 2.0F / points[point_1_idx][4] * sqrt(s * (s - points[point_1_idx][3]) * (s - points[point_2_idx][3]) * (s - points[point_2_idx][4]));
  }
  else {
    distance = 0.1F;
  }
}

// Save points table to EEPROM
void collision_points_save()
{
  int addr = 0;
  EEPROM.write(addr, 0x01);
  EEPROM.commit();
  addr++;
  for (uint8_t row = 0; row < POINTS_LEN; row++) {
    for (uint8_t col = 0; col < POINTS_WIDTH; col++) {
      //Serial.print(points[row][col]);
      //Serial.print(" ");
      EEPROM.put(addr, points[row][col]);
      addr += sizeof(float);
    }
    //Serial.println("");
  }
  EEPROM.put(addr, points_cnt);
  //Serial.println(points_cnt);
  //Serial.println("----------------------------------------------------");
  EEPROM.commit();
}

// Restore points table from EEPROM
void collision_points_restore()
{
  int addr = 0;
  byte marker = 0x00;
  marker = EEPROM.read(addr);
  addr++;
  //Serial.println(byte(marker));
  if (marker == 0x01) {
    for (uint8_t row = 0; row < POINTS_LEN; row++) {
      for (uint8_t col = 0; col < POINTS_WIDTH; col++) {
        EEPROM.get(addr, points[row][col]);
        //Serial.print(points[row][col]);
        //Serial.print(" ");
        addr += sizeof(float);
      }
      //Serial.println("");
    }
    EEPROM.get(addr, points_cnt);
    //Serial.println(points_cnt);
    //Serial.println("----------------------------------------------------");
  }
}
