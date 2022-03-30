#include <Arduino.h>
#include <Servo.h>

// We communicate with the power board at 115200 baud.
#define SERIAL_BAUD 115200

#define FW_VER 1

//================//
// SERVO CONTROLS //
//================//
#define NUM_SERVOS 1
Servo servo[NUM_SERVOS];

void setup() {
  Serial.begin(SERIAL_BAUD);
  for(int i = 0; i < NUM_SERVOS; i++) {
    servo[i].attach(servo_pins[i+4]);
  }
}

int read_pin() {
  while (!Serial.available());
  int pin = Serial.read();
  return (int)(pin - 'a');
}

void command_read() {
  int pin = read_pin();
  // Read from the expected pin.
  int level = digitalRead(pin);
  // Send back the result indicator.
  if (level == HIGH) {
    Serial.write('h');
  } else {
    Serial.write('l');
  }
}

void command_analogue_read() {
  int pin = read_pin();
  int value = analogRead(pin);
  Serial.print(value);
}

void command_write(int level) {
  int pin = read_pin();
  digitalWrite(pin, level);
}

void command_mode(int mode) {
  int pin = read_pin();
  pinMode(pin, mode);
}

void loop() {
  // Turn the servo
  // servo.write(((sin((float) millis() / 2000) + 1) * 0.5) * 180);
  // float angle = ((sin((float) millis() / 2000) + 1) * 0.5) * 180;
  // servo.writeMicroseconds(map(angle, 0, 180, 500, 1900));
  
  // Fetch all commands that are in the buffer
  while (Serial.available()) {
    int selected_command = Serial.read();

    // The input should be an integer in the range
    // 32 to 122 -- this gives 2deg increments
    if (selected_command >= 0 && selected_command <= 90) {
      for(int i = 0; i < NUM_SERVOS; i++) {
        servo[i].write(selected_command * 2);
      }
    } else {
      // Do something different based on what we got:
      switch (selected_command) {
        case 'a':
          command_analogue_read();
          break;
        case 'r':
          command_read();
          break;
        case 'l':
          command_write(LOW);
          break;
        case 'h':
          command_write(HIGH);
          break;
        case 'i':
          command_mode(INPUT);
          break;
        case 'o':
          command_mode(OUTPUT);
          break;
        case 'p':
          command_mode(INPUT_PULLUP);
          break;
        case 'v':
          Serial.print("SRcustom:");
          Serial.print(FW_VER);
          break;
        default:
          // A problem here: we do not know how to handle the command!
          // Just ignore this for now.
          break;
      }
    }
  
    Serial.print("\n");
  }
}
