// Define digital output pins for solenoids valves
int v1 = 13;
int v2 = 6;
int v3 = 14;
int v4 = 21;

// spark
int s1 = 4;

//load cell
#include "HX711.h"
#include "digitalWriteFast.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

// ematch
int EMATCH = A0;

HX711 scale;
float weight = 0;


// Analog input pins for pressure transducers
int OPD02  = A7;
int OPD01  = A10;
int EPD01  = A2;
int FPD01 = A14;
int FPD02 = A8; // Fix port num

// Raw analog values
int d1, d2, d3, d4, d5;

// Number of samples to average for pressure reading
int pres_samples = 10;

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


void setup() {
  // Set valve and coil pins as outputs
    pinMode(v1, OUTPUT);
    pinMode(v2, OUTPUT);
    pinMode(v3, OUTPUT);
    pinMode(v4, OUTPUT);
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
        double d1 = analogRead(OPD01);
        double d2 = analogRead(OPD02);
        double d3 = analogRead(FPD01);
        float Vread1 = d1 * (5.0 / 1024.0);
        float Vread2 = d2 * (5.0 / 1024.0);
        float Vread3 = d3 * (5.0 / 1024.0);
        float pressure1 = Pmax * ((Vread1 - V0) / (Vmax - V0));
        float pressure2 = Pmax * ((Vread2 - V0) / (Vmax - V0));
        float pressure3 = Pmax * ((Vread3 - V0) / (Vmax - V0));
        sum1 += pressure1;
        sum2 += pressure2;
        sum3 += pressure3;
    }

   // Compute ambient offsets to convert gauge to absolute pressure
    offset1 = 14.7 - (sum1 / 100);
    offset2 = 14.7 - (sum2 / 100);
    offset3 = 14.7 - (sum3 / 100);

   // Calibrate current-based sensors (pt1, pt2, pt4)
    for (int i = 0; i < 100; i++) {
        float d4 = analogRead(EPD01);
        float d5 = analogRead(FPD02);
        float Vread4 = d4 * (5.0 / 1024.0);
        float Vread5 = d5 * (5.0 / 1024.0);
        float pressure4 = (Pmax1k * (Vread4 - (I0 * R))) / (R * (Imax - I0));
        float pressure5 = (Pmax1k * (Vread5 - (I0 * R))) / (R * (Imax - I0));
        sum4 += pressure4;
        sum5 += pressure5;
    }

    offset4 = 14.7 - (sum4 / 100);
    offset5 = 14.7 - (sum5 / 100);

   //This probably should be changed to 115200 but then the pi
   //should also be changed
    Serial.begin(9600);

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    // Tare to zero the scale (remove any load)
    scale.tare();
    Serial.println("Scale tared.");
    // Set your own calibration factor here
    // current_factor * (actual weight / a bunch of factors)
    scale.set_scale(2280.0*(4980/1800)/5*.97);

    // EMatch
    pinMode(EMATCH, OUTPUT);
    digitalWrite(EMATCH, LOW);

}

void loop() {

  
  //checks for avaialable data from pi
  if (Serial.available()) 
  {
        //reads 1 character from pi
        char c = Serial.read();
        delay(1); // slight pause for command stability
        //Serial.print('Arduino recieved command'); // echo received command

        // Manual valve controls via serial commands
        if (c == '1') digitalWriteFast(v1, HIGH);
        if (c == '!') digitalWriteFast(v1, LOW);
        if (c == '2') digitalWriteFast(v2, HIGH);
        if (c == '@') digitalWriteFast(v2, LOW);
        if (c == '3') digitalWriteFast(v3, HIGH);
        if (c == '#') digitalWriteFast(v3, LOW);
        if (c == '4') digitalWriteFast(v4, HIGH);
        if (c == '$') digitalWriteFast(v4, LOW);

        if(c == '&') {
            weight = ReadLoadCell();
            Serial.println(weight);
        }

        if (c == '9') {
            unsigned long startMicros = micros();
            int count = 0;
            const unsigned long cycleTime = 16667;  // 60 Hz period in microseconds

            while (micros() - startMicros <= 300000) {  // 300 ms = 300,000 us
                unsigned long cycleStart = micros();

                digitalWriteFast(s1, HIGH);
                delayMicroseconds(15000);  // ON for 15 ms
                digitalWriteFast(s1, LOW);
                delayMicroseconds(1667);   // OFF for ~1.667 ms

                count++;

                // Optional precise wait until next full 16.667 ms
                while (micros() - cycleStart < cycleTime);  // Wait remainder of the cycle
            }

            Serial.print("0.3 seconds passed and sparked ");
            Serial.println(count);
        }

        // Read data from valvues
        if (c == '5') {
            float pressure = ReadOPD01();
            Serial.println(pressure);  // Send value back to Pi
        }
        if (c == '6') {
            float pressure = ReadOPD02();
            Serial.println(pressure);
        }
        if (c == '7') {
            float pressure = ReadEPD01();
            Serial.println(pressure);
        }
        if (c == '8') {
            float pressure = ReadFPD01();
            Serial.println(pressure);
        }
        if (c == 'A'){
            float pressure = ReadFPD02();
            Serial.println(pressure);
        }
        if (c == 'B' && !didSpark) {
            didSpark = true;
            for(int i=0; i<16; i++){
                spark_open();
                delayMicroseconds(9375);  
                spark_close();
                delayMicroseconds(9375);  
            }
            spark_close();
        }
        if (c == 'D'){
            digitalWrite(EMATCH, HIGH); // turns on MOSFET and fires match
            delay(500); // 1 second fire
            digitalWrite(EMATCH, LOW);
        }
   }

   
}

// ------------------- TEST SEQUENCES -------------------


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
        float pressure = (Pmax1k * (Vread - (I0 * R))) / (R * (Imax - I0)) + 7.7;
        pres_sum2 += pressure;
    }
    return pres_sum2 / pres_samples;
}



float ReadOPD01() {
    pres_sum1 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(OPD01) * (5.0 / 1024.0);
        float pressure = (Pmax1k * (Vread - (I0 * R))) / (R * (Imax - I0)) + 9.2;
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
        float pressure = (Pmax1k * (Vread - (I0 * R))) / (R * (Imax - I0)) + 9.7;
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
        float pressure = Pmax * ((Vread - V0) / (Vmax - V0));
        pres_sum4 += pressure;
    }
    return pres_sum4 / pres_samples;
}

float ReadFPD02() {
    pres_sum5 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(FPD02) * (5.0 / 1024.0);
        float pressure = Pmax * ((Vread - V0) / (Vmax - V0));
        pres_sum5 += pressure;
    }
    return pres_sum5 / pres_samples;
}



// ------------------- VALVE CONTROL -------------------

void OV_03_OPEN()  { digitalWriteFast(v3, HIGH); }
void FV_03_OPEN()  { digitalWriteFast(v2, HIGH); }
void NV_02_OPEN()  { digitalWriteFast(v4, HIGH); }
void FV_02_OPEN()  { digitalWriteFast(v1, HIGH); }
void FV_02_CLOSE() { digitalWriteFast(v1, LOW); }
void OV_03_CLOSE() { digitalWriteFast(v3, LOW); }
void FV_03_CLOSE() { digitalWriteFast(v2, LOW); }
void NV_02_CLOSE() { digitalWriteFast(v4, LOW); }

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
