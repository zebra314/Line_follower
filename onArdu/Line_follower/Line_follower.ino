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

int pos_R = 0;
int pos_L = 0;
int spd_R = 0;
int spd_L = 0;
int V_R = 0;
int V_L = 0;
int TRspd_R = 0;
int TRspd_L = 0;
String message = "0 0";

Timer T;
void working_mode(){
  
}
void readEncoder1(){
  int b = digitalRead(ENCB1);
  
  if(b>0){
    pos_R++;
  }else{
    pos_R--;
  }
}
void readEncoder2(){
  int b = digitalRead(ENCB2);
  
  if(b>0){
    pos_L++;
  }else{
    pos_L--;
  }
}

void speedReader(){
  spd_R = pos_R;
  pos_R = 0;
  spd_R = pos_R;
  pos_R = 0;
  
}
void motor_turn(){
  digitalWrite(L_1, LOW);
  digitalWrite(L_2, HIGH);
  digitalWrite(R_1, LOW);
  digitalWrite(R_2, HIGH);

} 

void motor_speed(int L_spd, int R_spd){
  analogWrite(L_pwm, L_spd);
  analogWrite(R_pwm, R_spd);
}

void spd_controll(){
  //spd_1 , spd_2 , TRspd_1, TRspd_2
  int error_R;
  int error_L;

  error_R = TRspd_R - spd_R;
  error_L = TRspd_L - spd_L;

}
void display(){
  Serial.println("---------------------");
  Serial.print("ACT SPDR ");
  Serial.print(spd_R);
  Serial.print(" ==> ");
  Serial.print("TRA SPDR ");
  Serial.println(TRspd_R);

  Serial.print("ACT SPDL ");
  Serial.print(spd_L);
  Serial.print(" ==> ");
  Serial.print("TRA SPDL ");
  Serial.println(TRspd_L);

}

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
  TRspd_L = left;
  TRspd_R = right;

//  Serial.println(TRspd_L);
//  Serial.println(TRspd_R);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(L_1,OUTPUT);
  pinMode(L_2,OUTPUT);
  pinMode(R_1,OUTPUT);
  pinMode(R_2,OUTPUT);
  pinMode(L_pwm,OUTPUT);
  pinMode(R_pwm,OUTPUT);
  motor_speed(0,0);
  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder2,RISING);

  // T.every(100,speedReader);
  // T.every(1000,display);
}


void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
      message = Serial.readString();
      Serial.println(message);
  }
  
  T.update();
  motor_turn();

  msg_process(message);
  motor_speed(TRspd_R,TRspd_L);
  
}