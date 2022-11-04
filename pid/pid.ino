#include <VL53L0X.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

int timeout = 700;


int SERVO_NORMOL = 90;
int SERVO_LEFT_LIMIT = 112; //112       101
int SERVO_RIGHT_LIMIT = 68; //68        79

#define PRELOAD_VALUE 170  // about 11 ms delay

#define XSHUT_F1 5 //5
#define XSHUT_F2 6 //4
#define XSHUT_SR 4 //4

#define TOF_F1_ADDR    0x2B      // this addr is set by default for each VL53L0X sensor     
#define TOF_F2_ADDR    0x2A
#define TOF_SR_ADDR    0x2C

#define SYSTEM_INTERRUPT_CLEAR       0x0B
#define RESULT_INTERRUPT_STATUS      0x13
#define RESULT_RANGE_STATUS          0x14
#define STEERING_CONTROL 9

#define SPEED_CONTROL    3
#define INA 7     // выходы arduino
#define INB 8
#define EN 12
#define PWM 11

// create vlx sensor objects
VL53L0X f1_sensor;
Servo my_servo;

void setup() {
  my_servo.attach(STEERING_CONTROL);
  my_servo.write(SERVO_NORMOL);

  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initialization ...");

  f1_sensor.setTimeout(timeout);
  f1_sensor.init();
  f1_sensor.startContinuous();

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

int f1, pl, sr;

float err = 0.0;
float deff = 0.0;
float deff_old = 0.0;
float err_old = 0.0;
int tm = 20;

volatile byte flag = 0;

int tof_base_value = 50;
float dt = 0.022;
float old_err = 0;
float P = 0.00;
float I = 0.00;
float D = 0.00;

float k1 = 0.02;
float k2 = 0.01;
float k3 = 0.03;

float value = 0.00;
void loop() {
   if (flag == 1) {
     if (0 != (f1_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
      f1 = f1_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
      f1_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
      }
     if (0 != (f2_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
      f2 = f2_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
      f2_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
      }
       
    err = f1 - f2; 
    P = err;                          // P
    I = I + err * dt;                 // I
    D = (err - old_err) * 22 / 1000;  // D
    old_err = err;

    value = P * k1 + I * k2 + D * k3;   // от -22 до 22
    flag = 0;

    Serial.println("MEASURED F1 Value " + String(f1) + "\t" + "PID SIGNAL " + String(value));
  }

 
// if ( err > SERVO_LEFT_LIMIT ) {
//    err = SERVO_LEFT_LIMIT;
//  }
//  if ( err < SERVO_RIGHT_LIMIT ) {
//     err = SERVO_RIGHT_LIMIT;
//  }
  
  my_servo.write(err); 
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
