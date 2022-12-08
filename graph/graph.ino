#include <VL53L0X.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

int timeout = 700;

#define PRELOAD_VALUE 170

#define PID_DISTANCE 200

#define XSHUT_F1 5 //5
#define XSHUT_F2 6 //4
#define XSHUT_SR 4 //4

#define TOF_F1_ADDR    0x2B      // this addr is set by default for each VL53L0X sensor     
#define TOF_F2_ADDR    0x2A
#define TOF_SR_ADDR    0x2C

#define SYSTEM_INTERRUPT_CLEAR       0x0B
#define RESULT_INTERRUPT_STATUS      0x13
#define RESULT_RANGE_STATUS          0x14

VL53L0X f1_sensor;
VL53L0X f2_sensor;
VL53L0X sr_sensor;
void setup() {
  pinMode(XSHUT_F1, OUTPUT);
  pinMode(XSHUT_F2, OUTPUT);
  pinMode(XSHUT_SR, OUTPUT);

  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initialization ...");
  
  pinMode(XSHUT_SR, INPUT);
  delay(10);
  sr_sensor.setAddress(TOF_SR_ADDR);

  sr_sensor.setTimeout(timeout);
  sr_sensor.init();
  sr_sensor.startContinuous();

  Serial.println("Setup done");

  cli();  // disable global interrupts
  TCCR2A = 0;
  TCCR2B = 0;

  OCR2A = PRELOAD_VALUE;
  TCCR2A |= (1 << WGM21);  // enable CTC mode
   
  // set 1024 as a frequency prescaller
  TCCR2B |= (1 << CS20);
  TCCR2B |= (1 << CS21);  
  TCCR2B |= (1 << CS22);  

  TIMSK2 = (1 << OCIE2A);   // enable Timer2 compare interrupt

  sei();
}

int sr;

float err = 0.0;
float deff = 0.0;
float deff_old = 0.0;
float err_old = 0.0;

volatile byte flag = 0;

int tof_base_value = 60;
float dt = 0.022;
float old_err = 0;
float P = 0.00;
float I = 0.00;
float D = 0.00;

float k1 = 1.2;
float k2 = 0.03;
float k3 = 0.02;

int value_motor = 0;
void loop() {
   if (flag == 1) {

     if (0 != (sr_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
        sr = sr_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
        sr_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
     }

     err = sr - PID_DISTANCE; 
     P = (err) * k1;                          // P
     I = (I + err * dt) * k2;                 // I
     D = ((err - err_old) * 22 / 1000) * k3;  // D
     err_old = err;
     Serial.println(D + I + D);
     flag = 0;
}
}

ISR(TIMER2_COMPA_vect)
{
    static int counter = 0;
    counter++;
    if (counter == 2) {
        flag = 1;
        counter = 0;
    }
 
}
