const int IMD_fault = 2;
const int BMS_fault = 3;
const int red = 4;  //turn on red lights
const int green = 5; //turn on green lights

void setup() {
  pinMode(IMD_fault, INPUT);  
  pinMode(BMS_fault, INPUT); 
  pinMode(red, OUTPUT); 
  pinMode(green, OUTPUT); 
  digitalWrite(red, LOW); //ensure both lights are initially off
  digitalWrite(green, LOW);
}

void loop() {
  if(IMD_fault == LOW && BMS_fault == LOW) {  //There is no fault from the IMD nor the BMS
    digitalWrite(green, HIGH);
  } 
  else { //There is a fault from the IMD and/or the BMS
  //Checking to make sure flash time is correct:
  //In the case of a fault, the red lights must flash between 2-5Hz at 50% duty cycle, we will do 2Hz, so one period is: on for
  //250ms, off for 250ms, this is done 2 times in one second. Since time high = time low for that 1 second, duty cycle is 50%
    digitalWrite(red, HIGH);
    delay(250);
    digitalWrite(red, LOW);
    delay(250);
  }
}
