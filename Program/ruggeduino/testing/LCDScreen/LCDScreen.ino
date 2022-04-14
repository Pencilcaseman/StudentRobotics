#include <LiquidCrystal.h>

// Parameters: (rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  Serial.begin(9600);
  while(!Serial);
  
  lcd.begin(16, 2);
  lcd.clear();
}

void loop() {
  lcd.clear(); // Needed to correct itself if offset!! Should constantly clear and redraw
  lcd.setCursor(0, 0);
  lcd.print("Jeremy is hungry");
  lcd.setCursor(5, 1);
  lcd.print("for blood");
  delay(100);
}
