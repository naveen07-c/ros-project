// Tier S biped — ESP32 firmware (HARDWARE_SELECTION.md §7)
// =========================================================
// Bridges the ROS 2 control PC and the robot's peripherals:
//
//   UART0 (USB, 115200, from ros2_control pwm_servo_hw plugin):
//     "S<ch>,<us>\n"      -> servo channel ch (0..9) to pulse us (500..2500)
//     "P,<us0>,...,<us9>\n" -> all ten channels (neutral/stand frame)
//
//   UART0 out (telemetry, ~50 Hz, parsed by esp32_sensor_bridge):
//     "T,<ms>,<ax_mm>,<ay_mm>,<az_mm>,<gx_md>,<gy_md>,<gz_md>,<fl>,<fr>\n"
//     accel mm/s^2, gyro mdeg/s, FSR 12-bit ADC counts (heel+toe summed)
//
//   I2C: MPU6050 IMU (0x68), PCA9685 PWM driver (0x40)
//   ADC1: FSR heel/toe per foot on GPIO 34/35 (left) 32/33 (right)

#include <Wire.h>

static const uint8_t PIN_SDA = 21, PIN_SCL = 22;
static const uint8_t PIN_FSR_L_HEEL = 34, PIN_FSR_L_TOE = 35;
static const uint8_t PIN_FSR_R_HEEL = 32, PIN_FSR_R_TOE = 33;

static const uint8_t MPU_ADDR = 0x68;
static const uint8_t PCA_ADDR = 0x40;
static const uint8_t N_SERVOS = 10;

// PCA9685 registers
static const uint8_t PCA_MODE1 = 0x00;
static const uint8_t PCA_MODE2 = 0x01;
static const uint8_t PCA_LED0_ON_L = 0x06;
static const uint8_t PCA_PRE_SCALE = 0xFE;

static uint16_t servo_us[N_SERVOS];

static void i2c_w(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static void pca_set_us(uint8_t ch, uint16_t us)
{
  if (ch >= N_SERVOS) return;
  us = constrain(us, 500, 2500);
  servo_us[ch] = us;
  // 50 Hz frame = 20000 us over 4096 ticks -> 4.88 us/tick
  uint16_t off = (uint32_t)us * 4096UL / 20000UL;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(PCA_LED0_ON_L + 4 * ch);
  Wire.write(0x00);            // on_l
  Wire.write(0x00);            // on_h
  Wire.write(off & 0xFF);      // off_l
  Wire.write(off >> 8);        // off_h
  Wire.endTransmission();
}

static void pca_init()
{
  i2c_w(PCA_MODE1, 0x10);      // sleep
  i2c_w(PCA_PRE_SCALE, 121);   // 25 MHz / (4096 * 50 Hz) - 1 = 121
  i2c_w(PCA_MODE2, 0x04);      // totem-pole outputs
  i2c_w(PCA_MODE1, 0x01);      // wake, no auto-increment needed (per-ch writes)
  delay(5);
  for (uint8_t ch = 0; ch < N_SERVOS; ++ch) {
    pca_set_us(ch, 1500);      // neutral centers = spawn hold analog
  }
}

static void mpu_init()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);            // PWR_MGMT_1: wake, internal clock
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);            // CONFIG: DLPF 44 Hz — matches sim IMU smoothness
  Wire.write(0x03);
  Wire.endTransmission();
}

static bool mpu_read(int16_t * ax, int16_t * ay, int16_t * az,
                     int16_t * gx, int16_t * gy, int16_t * gz)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);            // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  auto rd = []() { int16_t v = (int16_t)(Wire.read() << 8); v |= Wire.read(); return v; };
  *ax = rd(); *ay = rd(); *az = rd();
  int16_t t = rd(); (void)t;
  *gx = rd(); *gy = rd(); *gz = rd();
  return true;
}

// ---- command line assembly from the control PC ----
static char rx_line[128];
static uint8_t rx_len = 0;

static void handle_line(char * s)
{
  if (s[0] == 'S') {
    int ch = atoi(s + 1);
    char * c = strchr(s + 1, ',');
    if (c && ch >= 0 && ch < N_SERVOS) {
      pca_set_us((uint8_t)ch, (uint16_t)atoi(c + 1));
    }
  } else if (s[0] == 'P') {
    // "P,us0,...,us9" -> all ten channels (pwm_servo_hw on_deactivate)
    int ch = 0;
    char * c = strchr(s, ',');
    while (c && ch < N_SERVOS) {
      pca_set_us((uint8_t)ch++, (uint16_t)atoi(c + 1));
      c = strchr(c + 1, ',');
    }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_FSR_L_HEEL, INPUT);
  pinMode(PIN_FSR_L_TOE, INPUT);
  pinMode(PIN_FSR_R_HEEL, INPUT);
  pinMode(PIN_FSR_R_TOE, INPUT);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  pca_init();
  mpu_init();
}

void loop()
{
  // ---- RX: servo command lines from the control PC ----
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      rx_line[rx_len] = 0;
      handle_line(rx_line);
      rx_len = 0;
    } else if (ch != '\r' && rx_len < sizeof(rx_line) - 1) {
      rx_line[rx_len++] = ch;
    } else {
      rx_len = 0;  // overrun: resync
    }
  }

  // ---- telemetry at 50 Hz ----
  static uint32_t last_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_ms < 20) {
    return;
  }
  last_ms = now_ms;

  int16_t ax, ay, az, gx, gy, gz;
  long ax_mm = 0, ay_mm = 0, az_mm = 9810, gx_md = 0, gy_md = 0, gz_md = 0;
  if (mpu_read(&ax, &ay, &az, &gx, &gy, &gz)) {
    // +/-2g, 16384 LSB/g -> mm/s^2; +/-250 dps, 131 LSB/dps -> mdeg/s
    ax_mm = (long)ax * 9810L / 16384L;
    ay_mm = (long)ay * 9810L / 16384L;
    az_mm = (long)az * 9810L / 16384L;
    gx_md = (long)gx * 1000L / 131L;
    gy_md = (long)gy * 1000L / 131L;
    gz_md = (long)gz * 1000L / 131L;
  }

  // FSR: heel + toe summed per sole (12-bit ADC, bridge thresholds ~250)
  int fl = analogRead(PIN_FSR_L_HEEL) + analogRead(PIN_FSR_L_TOE);
  int fr = analogRead(PIN_FSR_R_HEEL) + analogRead(PIN_FSR_R_TOE);

  char out[128];
  snprintf(out, sizeof(out), "T,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d",
           (unsigned long)now_ms, ax_mm, ay_mm, az_mm,
           gx_md, gy_md, gz_md, fl, fr);
  Serial.println(out);
}
