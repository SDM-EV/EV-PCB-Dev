void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT); // red_1
  pinMode(3, OUTPUT); // red_2
  pinMode(4, OUTPUT); // grn_1
  pinMode(5, OUTPUT); // grn_2 
  pinMode(6, INPUT);  // imd_online
  pinMode(7, INPUT);  // bms_online
  pinMode(8, INPUT);  // imd_fault
  pinMode(9, INPUT);  // bms_fault
}

void loop() {
  // put your main code here, to run repeatedly:
  // assuming signal high is true, low is false
  if (digitalRead(6) & digitalRead(7) & !(digitalRead(8)) & !(digitalRead(9))) { // subject to change: verify signals
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
  } 
  else {
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    delay(350);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    delay(350);
  }
}
