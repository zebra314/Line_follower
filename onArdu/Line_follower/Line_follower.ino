#define L_1  26
#define L_2  28
#define R_1  30
#define R_2  32
#define L_pwm  6
#define R_pwm  7
#define LED  10

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
  Serial.begin(57600);
  pinMode(LED,OUTPUT);  
  pinMode(L_1,OUTPUT);
  pinMode(L_2,OUTPUT);
  pinMode(R_1,OUTPUT);
  pinMode(R_2,OUTPUT);
  pinMode(L_pwm,OUTPUT);
  pinMode(R_pwm,OUTPUT);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
}

String msg1, msg2;
void loop() {
  if (Serial.available() > 0 ){
    msg1 = Serial.readStringUntil(':'); 
    msg2 = Serial.readStringUntil('!');
    if (msg1 == "1" and msg2 == "23"){
      digitalWrite(LED, HIGH);
      delay(10);
    }
    digitalWrite(LED, LOW);

    Serial.println(msg1);
    Serial.println(msg2);
    Serial.flush();
  } 
  digitalWrite(LED, LOW);
}
