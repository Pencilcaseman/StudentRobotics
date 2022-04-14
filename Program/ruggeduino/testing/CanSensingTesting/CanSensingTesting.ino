#define READ_PIN 4

void setup() {
  // put your setup code here, to run once:
  pinMode(READ_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Reading: ");
  Serial.println(digitalRead(READ_PIN));
}
