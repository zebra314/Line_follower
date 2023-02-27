void setup() {
    Serial.begin(57600);
}

/**************************************
The message received from the Pi will be in string type. 
Normally, the data will be numbers with four digits, representing the percentage of the offset from the mid point.
For example, "10.95" means the current direction has 10.95 percent offset from the mid point at the moment.
When the process is finished, Arduino will receive "stop" message.
***************************************/ 

String message;
void loop() {

    // if there are msgs available
    if (Serial.available() > 0){
      message = Serial.readString();
      Serial.println(message);
    }
}