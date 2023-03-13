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

void motor_controll(int VO_L, int VO_R){
  if (VO_L < 0){ 
      digitalWrite(L_1, HIGH);
      digitalWrite(L_2, LOW); 
      analogWrite(L_pwm, abs(VO_L));
  }else {
      digitalWrite(L_1, LOW);
      digitalWrite(L_2, HIGH); 
      analogWrite(L_pwm, abs(VO_L));
  }

  if (VO_R < 0){
      digitalWrite(R_1, HIGH);
      digitalWrite(R_2, LOW);
      analogWrite(R_pwm, abs(VO_R));
  }else {
      digitalWrite(R_1, LOW);
      digitalWrite(R_2, HIGH);
      analogWrite(R_pwm, abs(VO_R));
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}

void loop() {
  if (Serial.available() > 0){
    message = Serial.readString();
    msg_process(message);
    digitalWrite(LED, HIGH);
    motor_controll(TR_L, TR_R);
    Serial.println(message);
    delay(50);
  } else{
    digitalWrite(LED, LOW);
    motor_controll(0,0);
  }
}
