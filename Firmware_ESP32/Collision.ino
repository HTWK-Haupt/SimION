// Calculate the distances from the tooltip to all collision points and populate the 4. column of the points table (in meter)
void collision_calc_point_to_tooltip_distances(void)
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
void collision_find_two_closest_points(void)
{
  float m = 1.0;

  // Just one single point
  if (points_cnt == 1) {
    point_1_idx = 0;
    point_2_idx = 0;
    return;
  }

  // Two or more points, find the first
  for (uint8_t i = 0; i < points_cnt; i++)
  {
    if (points[i][3] < m) {
      m = points[i][3];
      point_1_idx = i;
    }
  }

  // Find the second, skipping the first
  m = 1.0;
  for (uint8_t i = 0; i < points_cnt; i++)
  {
    if (i == point_1_idx)
      continue;
    if (points[i][3] < m) {
      m = points[i][3];
      point_2_idx = i;
    }
  }
}

// Calculate the distance between the tooltip and the line p1-p2
void collision_calc_distance(void)
{
  // Set a default distance if there are no teached points
  if (0 == points_cnt) {
    portENTER_CRITICAL_ISR(&mux);
    distance = 0.005F;
    portEXIT_CRITICAL_ISR(&mux);
    return;
  }

  // We have at least one point, more different points or more identical points
  collision_calc_point_to_tooltip_distances();
  collision_find_two_closest_points();

  // Points are the same (just one point or n identical teach points)
  if (point_1_idx == point_2_idx || (
        points[point_1_idx][0] == points[point_2_idx][0] &&
        points[point_1_idx][1] == points[point_2_idx][1] &&
        points[point_1_idx][2] == points[point_2_idx][2])) {
    // Distance is a point-to-point problem and already solved
    portENTER_CRITICAL_ISR(&mux);
    distance = points[point_1_idx][3];
    portEXIT_CRITICAL_ISR(&mux);
    return;
  }

  // We found two different points, calculating:
  //
  //            |vec(r_12) x ( vec(r_tt) - vec(r_1) )|
  // distance = --------------------------------------
  //                       |vec(r_12)|
  //
  // r_12 := directional vector of the straight line through point 1 and 2
  // r_tt := position vector of the tooltip
  // r_1 := point 1 of the straight line

  float r_12[3];
  r_12[0] = points[point_2_idx][0] - points[point_1_idx][0];
  r_12[1] = points[point_2_idx][1] - points[point_1_idx][1];
  r_12[2] = points[point_2_idx][2] - points[point_1_idx][2];

  float diff[3];
  diff[0] = tooltip[0] - points[point_1_idx][0];
  diff[1] = tooltip[1] - points[point_1_idx][1];
  diff[2] = tooltip[2] - points[point_1_idx][2];

  float cross[3];
  cross[0] = r_12[1] * diff[2] - r_12[2] * diff[1];
  cross[1] = r_12[2] * diff[0] - r_12[0] * diff[2];
  cross[2] = r_12[0] * diff[1] - r_12[1] * diff[0];

  portENTER_CRITICAL_ISR(&mux);
  distance = b_norm(cross) / b_norm(r_12);
  portEXIT_CRITICAL_ISR(&mux);
}

// Save points table to EEPROM
void collision_points_save()
{
  int addr = 0;
  EEPROM.write(addr, 0x01);
  //EEPROM.commit();
  addr++;
  EEPROM.put(addr, points_cnt);
  //Serial.println(points_cnt);
  addr++;
  for (uint8_t row = 0; row < points_cnt; row++) {
    for (uint8_t col = 0; col < POINTS_WIDTH; col++) {
      //Serial.print(points[row][col]);
      //Serial.print(" ");
      EEPROM.put(addr, points[row][col]);
      addr += sizeof(float);
    }
    //Serial.println("");
  }
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
    EEPROM.get(addr, points_cnt);
    //Serial.println(points_cnt);
    addr++;
    for (uint8_t row = 0; row < points_cnt; row++) {
      for (uint8_t col = 0; col < POINTS_WIDTH; col++) {
        EEPROM.get(addr, points[row][col]);
        //Serial.print(points[row][col]);
        //Serial.print(" ");
        addr += sizeof(float);
      }
      //Serial.println("");
    }
    //Serial.println("----------------------------------------------------");
  }
}
