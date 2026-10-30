/*
 * esp32_triple_flow_slave.ino - one ESP32: 3 flow sensors + 2 isolation valves
 *
 * Flash this on the ESP32 attached to Master Pi 2 (MasterPi2.c).
 *
 * Pipe layout (flow left -> right):
 *
 *   pump -- F1 --[V1]-- F2 --[V2]-- F3 -- demand
 *           \________ this board, I2C 0x32 ________/
 *
 *   Leak in seg1 (F1 > F2) -> master closes V1 (nearest to that segment)
 *   Leak in seg2 (F2 > F3) -> master closes V2 (nearest to that segment)
 *
 * Protocol (7-bit address 0x32)
 *   Master READ  10 bytes :
 *     [0xA5][F1 L][F1 H][F2 L][F2 H][F3 L][F3 H][V1][V2][XOR of bytes 0..8]
 *     V1/V2: 1 = open, 0 = closed
 *   Master WRITE 4 bytes :
 *     [0x10][valve 0=V1 1=V2][arg 0=close 1=open][0x10 ^ valve ^ arg ^ 0x5A]
 *
 * Flow: YF-S201-style pulse sensors, 2 s sliding window (4 x 500 ms).
 * Board package: esp32 by Espressif (Arduino core 2.x or 3.x).
 */

#include <Wire.h>

// ============================== CONFIG ==============================
#define I2C_ADDR          0x32

#define PIN_SDA           21
#define PIN_SCL           22
#define PIN_FLOW_1        27        // F1 (after pump) — 3.3 V-safe signal only
#define PIN_FLOW_2        25        // F2 (between V1 and V2)
#define PIN_FLOW_3        33        // F3 (after V2)
#define PIN_VALVE_1       26        // V1, between F1 and F2
#define PIN_VALVE_2       32        // V2, between F2 and F3

#define VALVE_OPEN_LEVEL  HIGH      // HIGH = energised = open (NC solenoid)
#define PULSES_PER_LITER  450.0f    // YF-S201 ~450; calibrate with a jug
#define I2C_FREQ          100000
// ====================================================================

#define PKT_LEN        10
#define PKT_HDR        0xA5
#define CMD_VALVE      0x10
#define CMD_XOR_SEED   0x5A
#define N_VALVE        2

#define SLICE_MS       500
#define SLICES         4

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrPulses1 = 0;
volatile uint32_t isrPulses2 = 0;
volatile uint32_t isrPulses3 = 0;
volatile int8_t   pendingValve[N_VALVE] = { -1, -1 };

uint8_t  txPacket[PKT_LEN] = { PKT_HDR, 0, 0, 0, 0, 0, 0, 1, 1, 0 };

bool     valveOpen[N_VALVE] = { true, true };
uint32_t ring1[SLICES] = { 0 };
uint32_t ring2[SLICES] = { 0 };
uint32_t ring3[SLICES] = { 0 };
uint8_t  ringIdx = 0;
uint32_t lastSlice = 0;
uint32_t lastPrint = 0;
uint16_t lastFlow1 = 0;
uint16_t lastFlow2 = 0;
uint16_t lastFlow3 = 0;

void IRAM_ATTR onPulse1()
{
  portENTER_CRITICAL_ISR(&mux);
  isrPulses1++;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR onPulse2()
{
  portENTER_CRITICAL_ISR(&mux);
  isrPulses2++;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR onPulse3()
{
  portENTER_CRITICAL_ISR(&mux);
  isrPulses3++;
  portEXIT_CRITICAL_ISR(&mux);
}

void buildPacket(uint16_t f1, uint16_t f2, uint16_t f3, bool v1, bool v2)
{
  uint8_t p[PKT_LEN];
  p[0] = PKT_HDR;
  p[1] = f1 & 0xFF;
  p[2] = f1 >> 8;
  p[3] = f2 & 0xFF;
  p[4] = f2 >> 8;
  p[5] = f3 & 0xFF;
  p[6] = f3 >> 8;
  p[7] = v1 ? 1 : 0;
  p[8] = v2 ? 1 : 0;
  p[9] = p[0] ^ p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5] ^ p[6] ^ p[7] ^ p[8];

  portENTER_CRITICAL(&mux);
  memcpy(txPacket, p, PKT_LEN);
  portEXIT_CRITICAL(&mux);
}

void setValve(int which, bool open)
{
  int pin = (which == 0) ? PIN_VALVE_1 : PIN_VALVE_2;
  digitalWrite(pin, open ? VALVE_OPEN_LEVEL : !VALVE_OPEN_LEVEL);
  valveOpen[which] = open;
  Serial.printf("Valve V%d -> %s\n", which + 1, open ? "OPEN" : "CLOSED");
}

void onI2CRequest()
{
  uint8_t copy[PKT_LEN];
  portENTER_CRITICAL(&mux);
  memcpy(copy, txPacket, PKT_LEN);
  portEXIT_CRITICAL(&mux);
  Wire.write(copy, PKT_LEN);
}

void onI2CReceive(int n)
{
  uint8_t buf[8];
  int cnt = 0;
  while (Wire.available()) {
    int b = Wire.read();
    if (cnt < (int)sizeof(buf)) buf[cnt++] = (uint8_t)b;
  }
  if (cnt == 4 && buf[0] == CMD_VALVE && buf[1] < N_VALVE && buf[2] <= 1 &&
      buf[3] == (uint8_t)(CMD_VALVE ^ buf[1] ^ buf[2] ^ CMD_XOR_SEED)) {
    portENTER_CRITICAL(&mux);
    pendingValve[buf[1]] = (int8_t)buf[2];
    portEXIT_CRITICAL(&mux);
  }
  (void)n;
}

static uint16_t flowFromPulses(uint32_t sum)
{
  float windowSec = (SLICES * SLICE_MS) / 1000.0f;
  float mlPerMin  = ((float)sum / windowSec) * 60000.0f / PULSES_PER_LITER;
  if (mlPerMin > 65535.0f) mlPerMin = 65535.0f;
  return (uint16_t)mlPerMin;
}

void setup()
{
  Serial.begin(115200);

  pinMode(PIN_FLOW_1, INPUT_PULLUP);
  pinMode(PIN_FLOW_2, INPUT_PULLUP);
  pinMode(PIN_FLOW_3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_1), onPulse1, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_2), onPulse2, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_3), onPulse3, FALLING);

  pinMode(PIN_VALVE_1, OUTPUT);
  pinMode(PIN_VALVE_2, OUTPUT);
  setValve(0, true);
  setValve(1, true);

  buildPacket(0, 0, 0, valveOpen[0], valveOpen[1]);

  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  Wire.begin((uint8_t)I2C_ADDR, PIN_SDA, PIN_SCL, I2C_FREQ);

  Serial.printf("I2C triple-flow slave ready at 0x%02X (F1+F2+F3, V1+V2)\n", I2C_ADDR);
  lastSlice = millis();
}

void loop()
{
  int8_t c[N_VALVE];
  bool changed = false;

  portENTER_CRITICAL(&mux);
  c[0] = pendingValve[0]; pendingValve[0] = -1;
  c[1] = pendingValve[1]; pendingValve[1] = -1;
  portEXIT_CRITICAL(&mux);

  for (int i = 0; i < N_VALVE; i++) {
    if (c[i] >= 0) {
      setValve(i, c[i] == 1);
      changed = true;
    }
  }
  if (changed)
    buildPacket(lastFlow1, lastFlow2, lastFlow3, valveOpen[0], valveOpen[1]);

  uint32_t now = millis();
  if (now - lastSlice >= SLICE_MS) {
    lastSlice += SLICE_MS;

    uint32_t p1, p2, p3;
    portENTER_CRITICAL(&mux);
    p1 = isrPulses1; isrPulses1 = 0;
    p2 = isrPulses2; isrPulses2 = 0;
    p3 = isrPulses3; isrPulses3 = 0;
    portEXIT_CRITICAL(&mux);

    ring1[ringIdx] = p1;
    ring2[ringIdx] = p2;
    ring3[ringIdx] = p3;
    ringIdx = (ringIdx + 1) % SLICES;

    uint32_t sum1 = 0, sum2 = 0, sum3 = 0;
    for (uint8_t i = 0; i < SLICES; i++) {
      sum1 += ring1[i];
      sum2 += ring2[i];
      sum3 += ring3[i];
    }

    lastFlow1 = flowFromPulses(sum1);
    lastFlow2 = flowFromPulses(sum2);
    lastFlow3 = flowFromPulses(sum3);
    buildPacket(lastFlow1, lastFlow2, lastFlow3, valveOpen[0], valveOpen[1]);
  }

  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.printf("F1 = %.3f  F2 = %.3f  F3 = %.3f L/min  V1 = %s  V2 = %s\n",
                  lastFlow1 / 1000.0f, lastFlow2 / 1000.0f, lastFlow3 / 1000.0f,
                  valveOpen[0] ? "OPEN" : "CLOSED",
                  valveOpen[1] ? "OPEN" : "CLOSED");
  }
}
