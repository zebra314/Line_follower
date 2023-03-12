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
unsigned long newtime;
unsigned long oldtime = 0;
int START_LAG_L = 0;
int START_LAG_R = 0;
int KEEP_LAG_L = 0;
int KEEP_LAG_R = 0;
int LAG_L = 0;
int LAG_R = 0;

int spd_L = 0;
int spd_R = 0;

int TR_R = 0;
int TR_L = 0;


String message;

int TRGA;
int TRGB;

Timer T;

void msg_process(String msg){
  String a, b;
  bool flag = false;
  int left, right;
  left = 0;
  right = 0;
  for(int i = 0;i<msg.length(); i++){
    if(msg[i] == ' '){
      flag = true;
      i++;
    }
    if(flag == false){
      a+=msg[i];
    }else {
      b+=msg[i];
    }
  }
  for(int i = a.length()-1, j=0; i>=0; i--, j++){
    left +=int(a[i]-'0')*pow(10, j);
  } 
  for(int i = b.length()-1, j=0; i>=0; i--, j++){
    right +=int(b[i]-'0')*pow(10, j);
  }
  TR_L = left;
  TR_R = right;
}

void readEncoder1(){
  int b = digitalRead(ENCB1);
  
  if(b>0){
    pos_R--;
  }else{
    pos_R++;
  }
}
void readEncoder2(){
  int c = digitalRead(ENCB2);
  
  if(c>0){
    pos_L--;
  }else{
    pos_L++;
  }
}

void motor_controll(int VO_L, int VO_R){
  
  int way_L=0;
  int way_R=0;
  int L_V;
  int R_V;
 
  L_V = abs(VO_L);
  R_V = abs(VO_R);
  way_L = VO_L/L_V;
  way_R = VO_R/R_V;
  
  switch(way_L){ //0後退 1左轉 2右轉 3前進 4停止
    case -1:
      digitalWrite(L_1, HIGH);
      digitalWrite(L_2, LOW);
      break;
    case 0:
      digitalWrite(L_1, LOW);
      digitalWrite(L_2, LOW);
      break;
    case 1:
      digitalWrite(L_1, LOW);
      digitalWrite(L_2, HIGH);
      break;
  }
  switch(way_R){ //0後退 1左轉 2右轉 3前進 4停止
    case -1:
      digitalWrite(R_1, HIGH);
      digitalWrite(R_2, LOW);
      break;
    case 0:
      digitalWrite(R_1, LOW);
      digitalWrite(R_2, LOW);
      break;
    case 1:
      digitalWrite(R_1, LOW);
      digitalWrite(R_2, HIGH);
      break;
  }
  analogWrite(L_pwm, L_V);
  analogWrite(R_pwm, R_V);
}

void controller(){
  int VO_R = 0;
  int VO_L = 0;
  VO_L = TR_L;
  VO_R = TR_R;
  motor_controll(VO_L, VO_R);
}
void PID_controller(){
  int VO_R,VO_L;
  int e_R,e_L;
  e_R = TR_R - pos_R;
  e_L = TR_L - pos_L;
  motor_controll(VO_L, VO_R);
}
void speed(){
  newtime = millis();
  spd_R = (pos_R-posd_R) * 1000 /(newtime-oldtime);
  spd_L = (pos_L-posd_L) * 1000 /(newtime-oldtime);
  posd_R = pos_R;
  posd_L = pos_L;
  oldtime = newtime;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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
  delay(1000);
  digitalWrite(LED, LOW);
}

void loop() {
  if (Serial.available() > 0){
    message = Serial.readString();
    msg_process(message);
    digitalWrite(LED, HIGH);
    controller(); 
    delay(150);
    motor_controll(0,0);
    Serial.println(message);
  }
  else
  {
    digitalWrite(LED, LOW);
    motor_controll(0,0);
  }
}