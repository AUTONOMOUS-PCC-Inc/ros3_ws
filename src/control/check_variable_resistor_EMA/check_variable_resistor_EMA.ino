const int POT_PIN = A2;
float filteredValue = 0;
float alpha = 0.1;

void setup() {

  Serial.begin(9600);
  filteredValue = analogRead(POT_PIN);

}

 

void loop() {

  int rawValue = analogRead(POT_PIN);

  // EMA Filter Formula
  filteredValue = (alpha * rawValue) + ((1.0-alpha) * filteredValue);

  Serial.print("Raw: ");
  Serial.print(rawValue);
  Serial.print("\t Filtered: ");
  Serial.println((int)filteredValue);
  
  delay(100);

}
