#include "Timer.h"
#define L_1  26
#define L_2  28
#define R_1  30
#define R_2  32
#define L_pwm  6
#define R_pwm  7
#define ENCA1  2
#define ENCB1  3
#define ENCA2  18
#define ENCB2  19
#define LED  10

long pos_R = 0;
long pos_L = 0;
long posd_R = 0;
long posd_L = 0;
bool phase1_R = false;
bool phase1_L = false;
bool check1 = true;
long newtime;
long oldtime = 0;
int START_LAG_L = 0;
int START_LAG_R = 0;
int KEEP_LAG_L = 0;
int KEEP_LAG_R = 0;
int olde_L = 0;
int olde_R = 0;
float LAG_L = 0;
float LAG_R = 0;

long spd_L ;
long spd_R ;
int spd_oL = 0;
int spd_oR = 0;
int TR_R = 200;
int TR_L = 200;
int a_L;
int a_R;
float Kp = 0.1;

int TRGA;
int TRGB;

Timer T;

void readEncoder1(){
  int b = digitalRead(ENCB1);
  
  if(b>0){
    pos_R++;
  }else{
    pos_R++;
  }
}
void readEncoder2(){
  int c = digitalRead(ENCB2);
  
  if(c>0){
    pos_L++;
  }else{
    pos_L++;
  }
}

void motor_controll(int left_speed, int right_speed){
  if (left_speed < 0){ 
      digitalWrite(L_1, HIGH);
      digitalWrite(L_2, LOW); 
      analogWrite(L_pwm, abs(left_speed));
  }else {
      digitalWrite(L_1, LOW);
      digitalWrite(L_2, HIGH); 
      analogWrite(L_pwm, abs(left_speed));
  }

  if (right_speed < 0){
      digitalWrite(R_1, HIGH);
      digitalWrite(R_2, LOW);
      analogWrite(R_pwm, abs(right_speed));
  }else {
      digitalWrite(R_1, LOW);
      digitalWrite(R_2, HIGH);
      analogWrite(R_pwm, abs(right_speed));
  }
}
void PID_controller(){
  float VO1_R,VO1_L;
  float e_R,e_L,e_T;
  int Tspd_R,Tspd_L;
  e_R = TR_R - spd_R;
  e_L = TR_L - spd_L;
  if(e_R>10){
    LAG_R++;
  }else if(e_R<-10){
    LAG_R--;
  }
  if(e_L>10){
    LAG_L++;
  }else if(e_L<-10){
    LAG_L--;
  }
  VO1_R = e_R*0.065 - a_R*0.015 + LAG_R*0.01;
  VO1_L = e_L*0.065 - a_L*0.015 + LAG_L*0.01;
  motor_controll(VO1_L, VO1_R);
}

void speed(){
  newtime = millis();
  spd_R = (pos_R-posd_R) * 1000 /(newtime-oldtime);
  spd_L = (pos_L-posd_L) * 1000 /(newtime-oldtime);
  a_R = (spd_R - spd_oR) * 1000 /(newtime-oldtime);
  a_L = (spd_L - spd_oL) * 1000 /(newtime-oldtime);
  posd_R = pos_R;
  posd_L = pos_L;
  spd_oR = spd_R;
  spd_oL = spd_L;
  oldtime = newtime; 
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(LED,OUTPUT);  
  pinMode(L_1,OUTPUT);
  pinMode(L_2,OUTPUT);
  pinMode(R_1,OUTPUT);
  pinMode(R_2,OUTPUT);

  pinMode(L_pwm,OUTPUT);
  pinMode(R_pwm,OUTPUT);

  pinMode(ENCA1,INPUT);
  pinMode(ENCA2,INPUT);
  pinMode(ENCB1,INPUT);
  pinMode(ENCB2,INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder2,RISING);

  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);

  T.every(100,speed);
}

String message_1, message_2;
void loop() {
  if (Serial.available() > 0 ){
    message_1 = Serial.readStringUntil(':'); 
    message_2 = Serial.readStringUntil('!');
    digitalWrite(LED, HIGH);
    delay(10);
    digitalWrite(LED, LOW);
    Serial.println(message_1);
    Serial.println(message_2);
    Serial.flush();
  } 
  T.update();
  PID_controller();
}
