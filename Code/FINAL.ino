
// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>

// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "CAR ROBOT"
#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 115 bytes
  { 255,6,0,0,0,108,0,17,0,0,0,139,1,108,200,1,1,5,0,2,
  8,8,93,29,0,134,31,31,31,65,117,116,111,0,74,111,121,115,116,105,
  99,107,0,129,2,49,106,6,24,76,101,110,103,104,116,32,32,87,105,100,
  116,104,32,32,68,105,115,116,97,110,99,101,32,32,71,111,32,32,83,116,
  111,112,32,32,83,101,101,100,0,3,3,59,104,19,134,36,31,7,4,88,
  99,18,53,24,31,36,5,16,117,76,76,32,36,31,31 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t mode; // =1 if switch ON and =0 if OFF
  uint8_t selectVar; // from 0 to 6
  int16_t Var; // -32768 .. +32767
  int8_t joystick_x; // from -100 to 100
  int8_t joystick_y; // from -100 to 100

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
#include <ESP32Servo.h>

// Pin definitions
#define ECHO_PIN 22    // Echo pin for ultrasonic sensor
#define TRIG_PIN 23    // Trigger pin for ultrasonic sensor
#define ENA_PIN 19     // Motor 1 enable pin (must be PWM)
#define IN1_PIN 18     // Motor 1 IN1 pin
#define IN2_PIN 5      // Motor 1 IN2 pin
#define IN3_PIN 17     // Motor 2 IN3 pin
#define IN4_PIN 16     // Motor 2 IN4 pin
#define ENB_PIN 4      // Motor 2 enable pin (must be PWM)
#define encoder_PIN 15 // digital encoder pin

// Ultrasonic sensor variables
long duration, distance=400;

// Encoder sensor variables
long step=0, stepLeft=0, stepRight=0 ;

// Servo motor variables
Servo ultrasonicServo;
Servo seedServo;  

//variables
int angle = 0;   
int direction = 90;  
int stopFlag=1;
int Length=0 , width=0; //cm
int distanceSeed=0 ; // المساغة بين كل بذرة
int numSeed=0; //عدد الخطوط الواجب زراعتها
int dir1, dir2 , rockFlag=0, runFlag=1, stepRock=0,stepRock1=0,flag=0;
int i=0; //عدد الخطوط المزروعة
int seedFlage=0;
int rockLeft = 0, rockRight = 0;

// PI control variables
float Kp = 0.5;  // Proportional gain
float Ki = 0.05;  // Integral gain
int integral = 0;  // Integral term for PI controller

// Timer objects
hw_timer_t *timerUltrasonic = NULL;

// Function prototypes
void rotateMotors(int targetStep, int currentStep, int dir1, int dir2);
void brakeMotors();
int  mapToPWM(int value);
void rockMode();
void seed();
int calculatePI(int targetStep, int currentStep);

// Interrupt handler for Encoder sensor reading
void IRAM_ATTR handleInterrupt() {
  if(flag == 0) {
    if(dir1 == 1 && dir2 == 1) {
      step++;
    }
    else if(dir1 == 0 && dir2 == 1) {
      stepLeft++;
    }
    else if(dir1 == 1 && dir2 == 0) {
      stepRight++;
    }
  }
  else {
    stepRock++;
  }
}

// Interrupt handler for ultrasonic sensor reading
void IRAM_ATTR onTimerUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration / 58.2;
}

void setup() {
  RemoteXY_Init (); 

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(encoder_PIN, INPUT_PULLUP); // تعيين الدبوس كمدخل مع مقاومة سحب لأعلى

  // Setup PWM pins for motor control
  ledcSetup(2, 5000, 8);
  ledcAttachPin(ENA_PIN, 2);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(ENB_PIN, 3);

  ultrasonicServo.setPeriodHertz(50);
  ultrasonicServo.attach(21);
  seedServo.setPeriodHertz(50);
  seedServo.attach(2, 1000, 2000);

  attachInterrupt(digitalPinToInterrupt(encoder_PIN), handleInterrupt, RISING); // تفعيل المقاطعة عند الجبهة الصاعدة

  // Setup ultrasonic sensor timer
  timerUltrasonic = timerBegin(1, 80, true);
  timerAttachInterrupt(timerUltrasonic, &onTimerUltrasonic, true);
  timerAlarmWrite(timerUltrasonic, 50000, true);
  timerAlarmEnable(timerUltrasonic);

  ultrasonicServo.write(90);
  seedServo.write(170);
  RemoteXY_delay(200);

}

void loop() {
  RemoteXY_Handler ();

  if(RemoteXY.connect_flag) {
    start:if(RemoteXY.mode==1 && RemoteXY.selectVar!=3 && RemoteXY.selectVar!=4 && RemoteXY.selectVar!=5) {
      if(RemoteXY.selectVar==0) Length=RemoteXY.Var*1.5;
      else if(RemoteXY.selectVar==1) width=RemoteXY.Var*1.5;
      else if(RemoteXY.selectVar==2) distanceSeed=RemoteXY.Var*1.5;
      numSeed=(width / 30)+1;
      runFlag=1;
      i=0;
      step=0;
      stepLeft=0;
      stepRight=0;
      stepRock=0;
    }
    if(RemoteXY.selectVar==3 && RemoteXY.mode==1) {
      while(runFlag == 1) {
        dir1=1;
        dir2=1;
        while (step < Length) { //سير للامام
          rotateMotors(Length,step, dir1, dir2);
          if((step+1)% distanceSeed ==0) { //شرط المسافة بين كل بذرة
            //brakeMotors();
            seed();
          }
          if(rockFlag == 0 && distance < 30) { //شرط وجود عائق
            rockFlag=1;
            rockMode();
          }
          if(RemoteXY.selectVar==4) {
            brakeMotors();
            goto start;
            }
        }
        brakeMotors();
        RemoteXY_delay(200);
        step=0;
        i++;
        if(i >= numSeed) { //شرط انتهاء مساحة الزراعة
          runFlag=0;
          break;
        }
        if(i % 2 == 1) { 
          dir1=1;
          dir2=0;
          while(stepRight < 44) {
            rotateMotors(160, 0, dir1, dir2);
            if(rockFlag == 0 && distance < 30) {
              rockFlag=1;
              rockMode();
            }
            if(RemoteXY.selectVar==4) {
              brakeMotors();
              goto start;
              }
          }
          stepRight=0;
        }
        else {
          dir1=0;
          dir2=1;
          while(stepLeft < 44) {
            rotateMotors(160, 0, dir1, dir2);
            if(rockFlag == 0 && distance < 30) {
              rockFlag=1;
              rockMode();
            }
            if(RemoteXY.selectVar==4) {
              brakeMotors();
              goto start;
              }
          }
          stepLeft=0;
        }
        brakeMotors();
        RemoteXY_delay(200);
        dir1=1;
        dir2=1;
        while(step < 30) {
          rotateMotors(30, step, dir1, dir2);
          if(rockFlag == 0 && distance < 30) {
            rockFlag=1;
            rockMode();
          }
          if(RemoteXY.selectVar==4) {
            brakeMotors();
            goto start;
            }
        }
        brakeMotors();
        RemoteXY_delay(200);
        step=0;
        if(i % 2 == 1) {
          dir1=1;
          dir2=0;
          while(stepRight < 44) {
            rotateMotors(160, 0, dir1, dir2);
            if(rockFlag == 0 && distance < 30) {
              rockFlag=1;
              rockMode();
            }
            if(RemoteXY.selectVar==4) {
              brakeMotors();
              goto start;
              }
          }
          stepRight=0;
        }
        else {
          dir1=0;
          dir2=1;
          while(stepLeft < 44) {
            rotateMotors(160, 0, dir1, dir2);
            if(rockFlag == 0 && distance < 30) {
              rockFlag=1;
              rockMode();
            }
            if(RemoteXY.selectVar==4) {
              brakeMotors();
              goto start;
              }
          }
          stepLeft=0;
        }
        brakeMotors();
        RemoteXY_delay(200);
      }
    }
    else if(RemoteXY.mode==0) {
      if(RemoteXY.joystick_y==0 && RemoteXY.joystick_x==0) brakeMotors();
      else if(RemoteXY.joystick_x >10 && RemoteXY.joystick_y<10  &&RemoteXY.joystick_y>-10) {
        dir1=1;
        dir2=0;
        rotateMotors(80, 0, dir1, dir2);
      }
      else if(RemoteXY.joystick_x <-10 && RemoteXY.joystick_y<10 && RemoteXY.joystick_y>-10) {
        dir1=0;
        dir2=1;
        rotateMotors(80, 0, dir1, dir2);
      }
      else if(RemoteXY.joystick_y >10 && RemoteXY.joystick_x >-10 && RemoteXY.joystick_x<10) {
        dir1=1;
        dir2=1;
        rotateMotors(RemoteXY.joystick_y, 0, dir1, dir2);
      }
      else if(RemoteXY.joystick_y <-10 && RemoteXY.joystick_x >-10 && RemoteXY.joystick_x<10) {
        dir1=0;
        dir2=0;
        rotateMotors(-1*RemoteXY.joystick_y, 0, dir1, dir2);
      }
      if(RemoteXY.selectVar==5 && seedFlage==0) {
        seedFlage=1;
        seed();
      }
      else if(RemoteXY.selectVar!=5) seedFlage=0;
    }
  }
}

void rotateMotors(int targetStep, int currentStep, int dir1, int dir2) {
  digitalWrite(IN1_PIN, dir1 == 1 ? LOW : HIGH);
  digitalWrite(IN2_PIN, dir1 == 1 ? HIGH : LOW);
  digitalWrite(IN3_PIN, dir2 == 1 ? LOW : HIGH);
  digitalWrite(IN4_PIN, dir2 == 1 ? HIGH : LOW);
  if(RemoteXY.mode==0) {
    ledcWrite(2,mapToPWM(targetStep < 50 ? 50 : targetStep));
    ledcWrite(3,mapToPWM(targetStep < 50 ? 50 : targetStep));
  }
  else {
  ledcWrite(2,calculatePI(targetStep, currentStep));
  ledcWrite(3,calculatePI(targetStep, currentStep));
  }
}

void brakeMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(ENB_PIN, LOW);
}

int mapToPWM(int value) {
  return map(value, 0, 100, 0, 255);
}

void rockMode() {
  scan:brakeMotors();
  RemoteXY_delay(200);
  ultrasonicServo.write(0);//Right
  RemoteXY_delay(1000);
  if(distance < 40) rockRight=1;
  ultrasonicServo.write(180);//Left
  RemoteXY_delay(1000);
  if(distance < 40) rockLeft=1;
  if(rockRight==0) {
    rockLeft=0;
    rockRight=0;
    ultrasonicServo.write(180);//left
    RemoteXY_delay(200);
    dir1=1;
    dir2=0;
    stepRight=0;
    while(stepRight < 44) rotateMotors(160, 0, dir1, dir2);
    stepRight=0;
    brakeMotors();
    RemoteXY_delay(200);
    dir1=1;
    dir2=1;
    rotateMotors(100, 0, dir1, dir2);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    flag=1;
    dir1=1;
    dir2=1;
    while(distance < 40) rotateMotors(100, 0, dir1, dir2);
    stepRock1=stepRock;
    stepRock=0;
    flag=0;
    RemoteXY_delay(300);
    brakeMotors();
    RemoteXY_delay(200);
    dir1=0;
    dir2=1;
    stepLeft=0;
    while(stepLeft < 44) rotateMotors(160, 0, dir1, dir2);
    stepLeft=0;
    brakeMotors();
    RemoteXY_delay(200);
    dir1=1;
    dir2=1;
    rotateMotors(100, 0, dir1, dir2);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    dir1=1;
    dir2=1;
    while(distance < 40) rotateMotors(100, 0, dir1, dir2);
    RemoteXY_delay(300);
    brakeMotors();
    RemoteXY_delay(200); 
    dir1=0;
    dir2=1;
    stepLeft=0;
    while(stepLeft < 44) rotateMotors(160, 0, dir1, dir2);
    stepLeft=0;
    brakeMotors();
    RemoteXY_delay(200);
    dir1=1;
    dir2=1;
    rotateMotors(100, 0, dir1, dir2);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    flag=1;
    dir1=1;
    dir2=1;
    stepRock=0;
    while(stepRock < stepRock1) rotateMotors(stepRock1, stepRock, dir1, dir2);
    stepRock=0;
    RemoteXY_delay(300);
    brakeMotors();
    RemoteXY_delay(200);
    flag=0;
    dir1=1;
    dir2=0;
    stepRight=0;
    while(stepRight < 44) rotateMotors(160, 0, dir1, dir2);
    stepRight=0;
    brakeMotors();
    ultrasonicServo.write(90);
    RemoteXY_delay(200);
    rockFlag=0;
  }
  else if(rockLeft==0) {
    rockLeft=0;
    rockRight=0;
    ultrasonicServo.write(0);//Right
    RemoteXY_delay(200);
    dir1=0;
    dir2=1;
    stepLeft=0;
    while(stepLeft < 44) rotateMotors(160, 0, dir1, dir2);
    stepLeft=0;
    brakeMotors();
    RemoteXY_delay(200);
    dir1=1;
    dir2=1;
    rotateMotors(100, 0, dir1, dir2);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    flag=1;
    dir1=1;
    dir2=1;
    stepRock=0;
    while(distance < 40) rotateMotors(100, 0, dir1, dir2);
    stepRock1=stepRock;
    stepRock=0;
    flag=0;
    RemoteXY_delay(300);
    brakeMotors();
    RemoteXY_delay(200);
    dir1=1;
    dir2=0;
    stepRight=0;
    while(stepRight < 44) rotateMotors(160, 0, dir1, dir2);
    stepRight=0;
    brakeMotors();
    RemoteXY_delay(200);
    dir1=1;
    dir2=1;
    rotateMotors(100, 0, dir1, dir2);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    dir1=1;
    dir2=1;
    while(distance < 40) rotateMotors(100, 0, dir1, dir2);
    RemoteXY_delay(300);
    brakeMotors();
    RemoteXY_delay(200); 
    dir1=1;
    dir2=0;
    stepRight=0;
    while(stepRight < 44) rotateMotors(160, 0, dir1, dir2);
    stepRight=0;
    brakeMotors();
    RemoteXY_delay(200);
    dir1=1;
    dir2=1;
    rotateMotors(100, 0, dir1, dir2);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    RemoteXY_delay(200);
    flag=1;
    dir1=1;
    dir2=1;
    stepRock=0;
    while(stepRock < stepRock1) rotateMotors(stepRock1, stepRock, dir1, dir2);
    stepRock=0;
    RemoteXY_delay(300);
    brakeMotors();
    RemoteXY_delay(200);
    flag=0;
    dir1=0;
    dir2=1;
    stepLeft=0;
    while(stepLeft < 44) rotateMotors(160, 0, dir1, dir2);
    stepLeft=0;
    brakeMotors();
    ultrasonicServo.write(90);
    RemoteXY_delay(200);
    rockFlag=0;    
  }
  else {
    rockLeft=0;
    rockRight=0;
    flag=1;
    dir1=0;
    dir2=0;
    while(stepRock < 70) rotateMotors(stepRock1, stepRock, dir1, dir2);
    flag=0;
    stepRock=0;
    goto scan;
  }
}

void seed() {
    seedServo.write(145);
    RemoteXY_delay(200);
    seedServo.write(170);
    RemoteXY_delay(200);
}

int calculatePI(int targetStep, int currentStep) {
    int error = targetStep - currentStep;
    integral = error;
    int controlValue = Kp * error + Ki * integral;
    controlValue = constrain(controlValue, 50, 100);  // Constrain output to 50% - 100%
    return mapToPWM(controlValue);
}
