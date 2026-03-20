// Define digital output pins for solenoids valves
int OV03 = 4;
int FV03 = 5;
int FV02 = 6;
int NV02 = 7;

//spark
int s1 = 3;

//load cell
#include "HX711.h"
#include "digitalWriteFast.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 10;
const int LOADCELL_SCK_PIN = 9;

HX711 scale;
float weight = 0;

// Analog input pins for pressure transducers
int FPD01 = A10;
int OPD01 = A12;
int OPD02 = A11;
int EPD01 = A9;
int FPD02 = A8;

// Raw analog values
int d1, d2, d3, d4, d5;

// Number of samples to average for pressure reading
int pres_samples = 1;  // Changed from 10 to 1 for 100Hz performance

// Pressure transducer full-scale pressure ratings
int Pmax = 500;     // PSI for voltage-output sensors
int Pmax1k = 1000;  // PSI for current-output sensors

// Transducer electrical characteristics
float R = 250;      // Load resistor for 4-20 mA sensors
float I0 = 0.004;   // 4 mA minimum
float Imax = 0.02;  // 20 mA maximum
float V0 = 0.5;     // Voltage transducer min output (V)
float Vmax = 4.5;   // Voltage transducer max output (V)

// Pressure smoothing and offset calibration variables
float pres_sum, pres_sum1, pres_sum2, pres_sum3, pres_sum4, pres_sum5;
float sum1, sum2, sum3, sum4, sum5;
float offset, offset1, offset2, offset3, offset4, offset5;

unsigned long startTime;

// Flags to track time-based sequence steps for test sequencing
bool did0s = false;
bool did20s = false;
bool did21s = false;
bool did29_95s = false;
bool did30s = false;
bool did30_5s = false;
bool did10s = false;
bool did15s = false;

bool didSpark = false;

// ------------------- 100 Hz TIMING -------------------
const unsigned long DT_US = 10000;  // 10 ms = 100 Hz
unsigned long next_t = 0;

// Spark state machine (non-blocking)
unsigned long spark_end_t = 0;
bool spark_active = false;

// Thrust cache (HX711 may not be ready at 100 Hz every tick)
float thrust_last = 0.0;

void setup() {
  // Set valve and coil pins as outputs
    pinMode(OV03, OUTPUT);
    pinMode(FV03, OUTPUT);
    pinMode(FV02, OUTPUT);
    pinMode(NV02, OUTPUT);
    pinMode(s1, OUTPUT);

  // Set pressure transducer pins as inputs
    pinMode(OPD02, INPUT);
    pinMode(FPD02, INPUT);
    pinMode(OPD01, INPUT);
    pinMode(FPD01, INPUT);
    pinMode(EPD01, INPUT);

  // Allow sensors to stabilize
    delay(3000);

  // Calibrate voltage-based sensors (pt3 and pt5)
    for (int i = 0; i < 100; i++) {
      double d1 = analogRead(EPD01);
      double d2 = analogRead(FPD02);
      float Vread1 = d1 * (5.0 / 1024.0);
      float Vread2 = d2 * (5.0 / 1024.0);
      float pressure1 = Pmax * ((Vread1 - V0) / (Vmax - V0));
      float pressure2 = Pmax * ((Vread2 - V0) / (Vmax - V0));
      sum1 += pressure1;
      sum2 += pressure2;
    }

    // Compute ambient offsets to convert gauge to absolute pressure
    offset1 = 14.7 - (sum1 / 100);
    offset2 = 14.7 - (sum2 / 100);

  // Calibrate current-based sensors (pt1, pt2, pt4)
    for (int i = 0; i < 100; i++) {
      float d3 = analogRead(OPD01);
      float d4 = analogRead(OPD02);
      float d5 = analogRead(FPD01);
      float Vread3 = d3 * (5.0 / 1024.0);
      float Vread4 = d4 * (5.0 / 1024.0);
      float Vread5 = d5 * (5.0 / 1024.0);
      float pressure3 = (Pmax1k * (Vread3 - (I0 * R))) / (R * (Imax - I0));
      float pressure4 = (Pmax1k * (Vread4 - (I0 * R))) / (R * (Imax - I0));
      float pressure5 = (Pmax1k * (Vread5 - (I0 * R))) / (R * (Imax - I0));
      sum3 += pressure3;
      sum4 += pressure4;
      sum5 += pressure5;
    }

    offset3 = 14.7 - (sum3 / 100);
    offset4 = 14.7 - (sum4 / 100);
    offset5 = 14.7 - (sum5 / 100);

  //have the pi match this
  Serial.begin(115200);

  //loadcell initialization
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.tare();
  scale.set_scale(2280.0*(4980/1800)/5*.97); // calibration factor

  // Initialize 100 Hz timing
  next_t = micros();
}

bool firstTime = true;

// ------------------- MAIN ------------------------
void loop() {
  // Valve Testing - disabled to maintain 100Hz strict timing
  if(firstTime){
    // Skip valve testing delays to maintain strict 100Hz from boot
  }

  // 1) Process incoming commands NON-BLOCKING (no delay())
  while (Serial.available() > 0) {
    char c = Serial.read();
    // NO delay() here - process immediately

    // Manual valve controls via serial commands
    if (c == '1') digitalWriteFast(FV02, HIGH);
    if (c == '!') digitalWriteFast(FV02, LOW);
    if (c == '2') digitalWriteFast(FV03, HIGH);
    if (c == '@') digitalWriteFast(FV03, LOW);
    if (c == '3') digitalWriteFast(OV03, HIGH);
    if (c == '#') digitalWriteFast(OV03, LOW);
    if (c == '4') digitalWriteFast(NV02, HIGH);
    if (c == '$') digitalWriteFast(NV02, LOW);

    // Spark - non-blocking state machine
    // Opens BOTH OV03 (oxidizer) and FV03 (fuel) + fires spark
    if (c == 'D') {
      digitalWriteFast(OV03, HIGH);  // Open OV03 (oxidizer)
      digitalWriteFast(FV03, HIGH);  // Open FV03 (fuel)
      digitalWriteFast(s1, HIGH);    // Fire spark immediately
      spark_active = true;
      spark_end_t = micros() + 100000; // 100ms duration
    }
  }

  // 2) Handle spark state machine (non-blocking)
  if (spark_active && (long)(micros() - spark_end_t) >= 0) {
    digitalWriteFast(s1, LOW);
    spark_active = false;
  }

  // 3) 100 Hz TICK - exact timing with micros()
  if ((long)(micros() - next_t) >= 0) {
    next_t += DT_US;  // Schedule next tick

    // Read all sensors (fast with pres_samples=1)
    float opd01 = ReadOPD01();
    float opd02 = ReadOPD02();
    float epd01 = ReadEPD01();
    float fpd01 = ReadFPD01();
    float fpd02 = ReadFPD02();

    // HX711 may not be ready every tick; cache last value
    if (scale.is_ready()) {
      thrust_last = ReadLoadCell();
    }
    float thrust = thrust_last;

    // Timestamp in milliseconds (use scheduled time for consistent timing)
    unsigned long t_ms = next_t / 1000UL;

    // 4) Output CSV format: t_ms,OPD01,OPD02,EPD01,FPD01,FPD02,THRUST
    Serial.print(t_ms);  Serial.print(',');
    Serial.print(opd01, 2); Serial.print(',');
    Serial.print(opd02, 2); Serial.print(',');
    Serial.print(epd01, 2); Serial.print(',');
    Serial.print(fpd01, 2); Serial.print(',');
    Serial.print(fpd02, 2); Serial.print(',');
    Serial.println(thrust, 2);
  }
}

// Load Cell Function
float ReadLoadCell() {
    weight = scale.get_units(1); // Read 1 sample
    return (weight + 50)/454;
}

// ------------------- PRESSURE SENSOR FUNCTIONS -------------------

// Read pressure from OPD (pt1)
float ReadOPD02() {
    pres_sum2 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(OPD02) * (5.0 / 1024.0);
        float pressure = (Pmax1k * (Vread - (I0 * R))) / (R * (Imax - I0)) + 7.7 + 25;
        pres_sum2 += pressure;
    }
    return pres_sum2 / pres_samples;
}

float ReadOPD01() {
    pres_sum1 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(OPD01) * (5.0 / 1024.0);
        float pressure = (Pmax1k * (Vread - (I0 * R))) / (R * (Imax - I0)) + 9.2 + 35 + 9.5;
        pres_sum1 += pressure;
    }
    return pres_sum1 / pres_samples;
}

// Read pressure from FPD (pt4)
float ReadFPD01() {
    pres_sum3 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(FPD01) * (5.0 / 1024.0);
        float pressure = (Pmax1k * (Vread - (I0 * R))) / (R * (Imax - I0)) + 39.7 - 6.0;
        pres_sum3 += pressure;
    }
    return pres_sum3 / pres_samples;
}

// Read pressure from EPD (pt5)
float ReadEPD01() {
    pres_sum4 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(EPD01) * (5.0 / 1024.0);
        float pressure = Pmax * ((Vread - V0) / (Vmax - V0)) + 15 + 3.5;
        pres_sum4 += pressure;
    }
    return pres_sum4 / pres_samples;
}

float ReadFPD02() {
    pres_sum5 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(FPD02) * (5.0 / 1024.0);
        float pressure = Pmax * ((Vread - V0) / (Vmax - V0)) + 15.0 + 47.5;
        pres_sum5 += pressure;
    }
    return pres_sum5 / pres_samples;
}

// ------------------- VALVE CONTROL -------------------
void OV_03_OPEN()  { digitalWriteFast(OV03, HIGH); }
void OV_03_CLOSE() { digitalWriteFast(OV03, LOW); }

void FV_03_OPEN()  { digitalWriteFast(FV03, HIGH); }
void FV_03_CLOSE() { digitalWriteFast(FV03, LOW); }

void NV_02_OPEN()  { digitalWriteFast(NV02, HIGH); }
void NV_02_CLOSE() { digitalWriteFast(NV02, LOW); }

void FV_02_OPEN()  { digitalWriteFast(FV02, HIGH); }
void FV_02_CLOSE() { digitalWriteFast(FV02, LOW); }

// ------------------- SPARK -------------------
void spark_open() { digitalWriteFast(s1, HIGH); }
void spark_close() { digitalWriteFast(s1, LOW); }

// ------------------- ABORT SEQUENCE -------------------
void BLP_Abort() {
    OV_03_CLOSE();
    delay(1);
    FV_03_CLOSE();
    delay(1);
    FV_02_OPEN();  // Unusual: opens instead of closing? Confirm logic
    delay(1);
    NV_02_CLOSE();
}
