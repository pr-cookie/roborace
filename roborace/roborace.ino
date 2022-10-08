#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <VL53L0X.h>
#include <Servo.h>

int timeout = 700;
float k1 = 0.05;
float k2 = 0.09;

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
VL53L0X f2_sensor;
VL53L0X sr_sensor;

Servo my_servo;

int motorSpeed = 50;

// try to implement PID

void setup() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(EN, OUTPUT);

  digitalWrite(EN, HIGH);

  pinMode(3, OUTPUT);
  my_servo.attach(STEERING_CONTROL);
  my_servo.write(SERVO_NORMOL);

//  delay(5000);

  pinMode(XSHUT_F1, OUTPUT);
  pinMode(XSHUT_F2, OUTPUT);
  pinMode(XSHUT_SR, OUTPUT);
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initialization ...");
      
  pinMode(XSHUT_F1, INPUT);
  delay(10);
  f1_sensor.setAddress(TOF_F1_ADDR);
  
  pinMode(XSHUT_F2, INPUT);
  delay(10);
  f2_sensor.setAddress(TOF_F2_ADDR);

  pinMode(XSHUT_SR, INPUT);
  delay(10);
  sr_sensor.setAddress(TOF_SR_ADDR);

  //Serial.println(58);

  f1_sensor.setTimeout(timeout);
  f2_sensor.setTimeout(timeout);
  sr_sensor.setTimeout(timeout);

  
  //Serial.println(56);
  f1_sensor.init();
  f2_sensor.init();
  sr_sensor.init();
  
  f1_sensor.startContinuous();
  f2_sensor.startContinuous();
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

int f1, f2, pl, sr;

float err = 0.0;
float deff = 0.0;
float deff_old = 0.0;

void(* resetFunc) (void) = 0;

volatile byte flag = 0;

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
    if (0 != (sr_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
      sr = sr_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
      sr_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
      }
   flag = 0;
  }
    if (f1 < 0 or f2 < 0 or sr < 0){

  digitalWrite(INB, HIGH);
  digitalWrite(INA, HIGH);
    resetFunc();
}
 if ( sr > 1300 or f1 < 800 or f2 < 800){
  motorSpeed = 70;
  k1 = 0.02;
  k2 = 0.02;
 }else if ( sr <= 1300 or f1 >= 800 or f2 >= 800 ){
  motorSpeed = 50;
  k1 = 1.4;
  k2 = 0.2;
 }
  deff = f1 - f2;
  err = SERVO_NORMOL - ((deff * k1 ) + k2 * (deff - deff_old));//
  deff_old = deff * k1 + k2 * (deff - deff_old);

  if ( err > SERVO_LEFT_LIMIT ) {
    err = SERVO_LEFT_LIMIT;
  }
  if ( err < SERVO_RIGHT_LIMIT ) {
     err = SERVO_RIGHT_LIMIT;
  }
  my_servo.write(err); 
  
  // Serial.print(f1);
  // Serial.print("    ");
  // Serial.print(f2);
  // Serial.print("    ");
  // Serial.print(sr);
  // Serial.print("    ");
  //  Serial.println(motorSpeed);
  
  if (f1 < 300 or f2 < 300 or sr < 350) {
    
    if (err > 0){
    my_servo.write(SERVO_LEFT_LIMIT); 
     delay(500);
      
    }
    if (err < 0){
    my_servo.write(SERVO_RIGHT_LIMIT);
     delay(500);
      
    }
    digitalWrite(INA, LOW);    // крутим мотор в                          противоположную сторону
    digitalWrite(INB, HIGH);
    analogWrite(PWM, 50);
    delay(1500);

    if (err > 0){
    my_servo.write(SERVO_LEFT_LIMIT); 
     delay(500);
      
    }
    if (err < 0){
    my_servo.write(SERVO_RIGHT_LIMIT);
     delay(500);
      
    }
    digitalWrite(INA, HIGH);    // крутим мотор в противоположную сторону
    digitalWrite(INB, LOW);
    analogWrite(PWM, 50);
    delay(500);
  }
//  if (f1 >= 250 or f2 >= 250 or sr >= 250) {
    digitalWrite(INA, HIGH);    // крутим мотор в противоположную сторону
    digitalWrite(INB, LOW);
    analogWrite(PWM, motorSpeed);
//  } 
}

// TM2 CTC interrupt handler
ISR(TIMER2_COMPA_vect)
{
    static int counter = 0;
    counter++;
    if (counter == 2) {
        flag = 1;
        counter = 0;
    }
 
}
