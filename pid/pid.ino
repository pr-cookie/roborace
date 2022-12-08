#include <VL53L0X.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

#define MOTOR_CONTROL    3
#define INA 7     // выходы arduino
#define INB 8
#define EN 12
#define PWM 11

#define START_DELAY 228  // it is 5s for 22 ms timer 228
#define PID_DISTANCE 100

#define INIT_STATE 0
#define MOTOR_STATE 1
#define SERVA_STATE 2


// create vlx sensor objects
VL53L0X f1_sensor;
VL53L0X f2_sensor;
VL53L0X sr_sensor;
Servo my_servo;

int motorMOTOR = 50;
void setup() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(EN, OUTPUT);
  
  digitalWrite(EN, HIGH);
  
  my_servo.attach(STEERING_CONTROL);
  my_servo.write(SERVO_NORMOL);

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
  
  f1_sensor.setTimeout(timeout);
  f1_sensor.init();
  f1_sensor.startContinuous();

  f2_sensor.setTimeout(timeout);
  f2_sensor.init();
  f2_sensor.startContinuous();

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

int f1, pl, sr, f2;

float err_motor = 0.0;
float deff_motor = 0.0;
float deff_old_motor = 0.0;
float err_old_motor = 0.0;
int tm = 20;

volatile byte flag = 0;

int tof_base_value = 60;
float dt = 0.022;
float old_err_motor = 0;
float P_motor = 0.00;
float I_motor = 0.00;
float D_motor = 0.00;

float k_motor1 = 1.2;
float k_motor2 = 0.003;
float k_motor3 = 0.2;

float k_serva1 = 1.5;
float k_serva2 = 0.005;
float k_serva3 = 0.04;

float err_serva = 0.0;
float deff_serva = 0.0;
float deff_old_serva = 0.0;
float err_old_serva = 0.0;

float old_err_serva = 0;
float P_serva = 0.00;
float I_serva = 0.00;
float D_serva = 0.00;

int mp = 0;
int value_motor = 0;
int value_serva = 0;

int state = INIT_STATE;

int start_counter = 0;

void loop() {
   if (flag == 1) {

     if (0 != (sr_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
        sr = sr_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
        sr_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
     }
     
     if (0 != (f2_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
        f2 = f2_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
        f2_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
     }

     if (0 != (f1_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
        f1 = f1_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
        f1_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
     }
      
    
       switch(state) {
           case INIT_STATE:            // Use 5s delay
               start_counter++;
               if (start_counter >= START_DELAY) {
                 start_counter = 0;
                 digitalWrite(INA, LOW);   
                 digitalWrite(INB, HIGH);
                 state = SERVA_STATE;
//                 Serial.println("STATE: " + String(state) + "\tSR " + String(sr)+ "\t" + "PID SIGNAL " + String(value));
               }
               
           break;

           case MOTOR_STATE:
              err_motor = sr - PID_DISTANCE; 
              P_motor = err_motor;                          // P
              I_motor = I_motor + err_motor * dt;                 // I
              D_motor = (err_motor - old_err_motor) * 22 / 1000;  // D
              old_err_motor = err_motor;
        
              value_motor = P_motor * k_motor1 + I_motor * k_motor2 + D_motor * k_motor3;   // от -22 до 22 
              if (value_motor > 100) {
                 value_motor = 100;
              }
              
              if (value_motor < 0) {
                 value_motor = 0;
              }                     
              state = SERVA_STATE;
           break;
                     
           case SERVA_STATE:
              err_serva = f1 - f2; 
              P_serva = err_serva;                          // P
              I_serva = I_serva + err_serva * dt;                 // I
              D_serva = (err_serva - old_err_serva) * 22 / 1000;  // D
              old_err_serva = err_serva;
        
              value_serva = P_serva * k_serva1 + I_serva * k_serva2 + D_serva * k_serva3;   // от -22 до 22 
              if (value_serva > 112) {
                 value_serva = 112;
              }
              
              if (value_serva < 68) {
                 value_serva = 68;
              }              
              state = MOTOR_STATE;
              
           break;
       }

       flag = 0;
   }
   Serial.print(String(f1)+ "\t" + "    " + String(f2) + "\t" + "    ");
   Serial.println(value_serva);
   my_servo.write(value_serva); 
   analogWrite(PWM, value_motor);
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
