// SI7210 device address
#define SI72_ADDR    0x30

// Bit masks
#define SI72_OTP_BUSY_MASK    1
#define SI72_OTP_READ_EN_MASK 2
#define SI72_NUM_HALL_DEVICES 3
#define SI72_STOP_MASK      2
#define SI72_SLTIMEENA_MASK   1
#define SI72_SW_TAMPER_MASK   0xFC
#define SI72_SL_FAST_MASK   2
#define SI72_SLEEP_MASK     1
#define SI72_ONEBURST_MASK    4

// I2C registers for Si72xx
#define SI72_HREVID   0xC0
#define SI72_DSPSIGM  0xC1
#define SI72_DSPSIGL  0xC2
#define SI72_DSPSIGSEL  0xC3
#define SI72_POWER_CTRL 0xC4
#define SI72_ARAUTOINC  0xC5
#define SI72_CTRL1    0xC6
#define SI72_CTRL2    0xC7
#define SI72_SLTIME   0xC8
#define SI72_CTRL3    0xC9
#define SI72_A0     0xCA
#define SI72_A1     0xCB
#define SI72_A2     0xCC
#define SI72_CTRL4    0xCD
#define SI72_A3     0xCE
#define SI72_A4     0xCF
#define SI72_A5     0xD0
#define SI72_OTP_ADDR 0xE1
#define SI72_OTP_DATA 0xE2
#define SI72_OTP_CTRL 0xE3
#define SI72_TM_FG    0xE4

void si72_wake_up()
{
  Wire.beginTransmission(SI72_ADDR);
  Wire.write(SI72_HREVID);
  Wire.endTransmission();
}

uint8_t si72_read_register(uint8_t reg)
{
  Wire.beginTransmission(SI72_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(SI72_ADDR, 1);
  return Wire.read();
}

void si72_write_register(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(SI72_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t si72_read_data(int16_t *data)
{
	uint8_t read;
	// Autoincrement is required
  Wire.beginTransmission(SI72_ADDR);
  Wire.write(SI72_DSPSIGM);
  Wire.endTransmission();
  Wire.requestFrom(SI72_ADDR, 2);
  read = Wire.read();
  *data = (((uint16_t)read) & 0x007F) << 8;
  *data |= Wire.read();
  *data -= 16384;
  // Return most significant bit of m.s. byte -> 1: fresh data
  return (read & 0b10000000) >> 7;
}
