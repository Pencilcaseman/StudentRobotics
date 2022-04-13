// Documentation

/*
  #ADDSERVO id, pin, angle, minDutyCycle, maxDutyCycle, minAngle, maxAngle#
  Set up a new Servo Motor
  Parameters:
  0: Servo ID
  1: Pin
  2: Angle = 0
  3: Minimum duty cycle = 600
  4: Maximum duty cycle = 2400
  5: Minimum angle = 0
  6: Maximum angle = 180

  #SETANGLE id, angle#
  Set the angle of a Servo Motor
  Parameters:
  0: Servo ID
  1: Angle

  #GETANGLE id#
  Get the angle of a Servo Motor
  Parameters:
  0: Servo ID

  #ENABLE id#
  Enable a Servo Motor (i.e. attach())
  Parameters:
  0: Servo ID

  #DISABLE id#
  Disable a Servo Motor (i.e. no signal)
  Parameters:
  0: Servo ID

  #SETDISPLAY char#
  Set the display to be a value
  Parameters:
  0: Character to display
*/

#include <Arduino.h>
#include <Servo.h>
#include <avr/pgmspace.h>
#include "MaxMatrix.h"
#include <ArduinoSTL.h> // Requires "ArduinoSTL" library => https://github.com/mike-matera/ArduinoSTL
#include <map>
#include <vector>

// We communicate with the power board at 115200 baud.
#define SERIAL_BAUD 115200
#define FW_VER 1

#define DISP_DIN_PIN 11
#define DISP_CS_PIN 12
#define DISP_CLR_PIN 13

PROGMEM const unsigned char CH[] = {
  3, 8, B00000000, B00000000, B00000000, B00000000, B00000000, //  
  1, 8, B01011111, B00000000, B00000000, B00000000, B00000000, // !
  3, 8, B00000011, B00000000, B00000011, B00000000, B00000000, // "
  5, 8, B00010100, B00111110, B00010100, B00111110, B00010100, // #
  4, 8, B00100100, B01101010, B00101011, B00010010, B00000000, // $
  5, 8, B01100011, B00010011, B00001000, B01100100, B01100011, // %
  5, 8, B00110110, B01001001, B01010110, B00100000, B01010000, // &
  1, 8, B00000011, B00000000, B00000000, B00000000, B00000000, // '
  3, 8, B00011100, B00100010, B01000001, B00000000, B00000000, // (
  3, 8, B01000001, B00100010, B00011100, B00000000, B00000000, // )
  5, 8, B00101000, B00011000, B00001110, B00011000, B00101000, // *
  5, 8, B00001000, B00001000, B00111110, B00001000, B00001000, // +
  2, 8, B10110000, B01110000, B00000000, B00000000, B00000000, // ,
  4, 8, B00001000, B00001000, B00001000, B00001000, B00000000, // -
  2, 8, B01100000, B01100000, B00000000, B00000000, B00000000, // .
  4, 8, B01100000, B00011000, B00000110, B00000001, B00000000, // /
  4, 8, B00111110, B01000001, B01000001, B00111110, B00000000, // 0
  3, 8, B01000010, B01111111, B01000000, B00000000, B00000000, // 1
  4, 8, B01100010, B01010001, B01001001, B01000110, B00000000, // 2
  4, 8, B00100010, B01000001, B01001001, B00110110, B00000000, // 3
  4, 8, B00011000, B00010100, B00010010, B01111111, B00000000, // 4
  4, 8, B00100111, B01000101, B01000101, B00111001, B00000000, // 5
  4, 8, B00111110, B01001001, B01001001, B00110000, B00000000, // 6
  4, 8, B01100001, B00010001, B00001001, B00000111, B00000000, // 7
  4, 8, B00110110, B01001001, B01001001, B00110110, B00000000, // 8
  4, 8, B00000110, B01001001, B01001001, B00111110, B00000000, // 9
  2, 8, B01010000, B00000000, B00000000, B00000000, B00000000, // :
  2, 8, B10000000, B01010000, B00000000, B00000000, B00000000, // ;
  3, 8, B00010000, B00101000, B01000100, B00000000, B00000000, // <
  3, 8, B00010100, B00010100, B00010100, B00000000, B00000000, // =
  3, 8, B01000100, B00101000, B00010000, B00000000, B00000000, // >
  4, 8, B00000010, B01011001, B00001001, B00000110, B00000000, // ?
  5, 8, B00111110, B01001001, B01010101, B01011101, B00001110, // @
  4, 8, B01111110, B00010001, B00010001, B01111110, B00000000, // A
  4, 8, B01111111, B01001001, B01001001, B00110110, B00000000, // B
  4, 8, B00111110, B01000001, B01000001, B00100010, B00000000, // C
  4, 8, B01111111, B01000001, B01000001, B00111110, B00000000, // D
  4, 8, B01111111, B01001001, B01001001, B01000001, B00000000, // E
  4, 8, B01111111, B00001001, B00001001, B00000001, B00000000, // F
  4, 8, B00111110, B01000001, B01001001, B01111010, B00000000, // G
  4, 8, B01111111, B00001000, B00001000, B01111111, B00000000, // H
  3, 8, B01000001, B01111111, B01000001, B00000000, B00000000, // I
  4, 8, B00110000, B01000000, B01000001, B00111111, B00000000, // J
  4, 8, B01111111, B00001000, B00010100, B01100011, B00000000, // K
  4, 8, B01111111, B01000000, B01000000, B01000000, B00000000, // L
  5, 8, B01111111, B00000010, B00001100, B00000010, B01111111, // M
  5, 8, B01111111, B00000100, B00001000, B00010000, B01111111, // N
  4, 8, B00111110, B01000001, B01000001, B00111110, B00000000, // O
  4, 8, B01111111, B00001001, B00001001, B00000110, B00000000, // P
  4, 8, B00111110, B01000001, B01000001, B10111110, B00000000, // Q
  4, 8, B01111111, B00001001, B00001001, B01110110, B00000000, // R
  4, 8, B01000110, B01001001, B01001001, B00110010, B00000000, // S
  5, 8, B00000001, B00000001, B01111111, B00000001, B00000001, // T
  4, 8, B00111111, B01000000, B01000000, B00111111, B00000000, // U
  5, 8, B00001111, B00110000, B01000000, B00110000, B00001111, // V
  5, 8, B00111111, B01000000, B00111000, B01000000, B00111111, // W
  5, 8, B01100011, B00010100, B00001000, B00010100, B01100011, // X
  5, 8, B00000111, B00001000, B01110000, B00001000, B00000111, // Y
  4, 8, B01100001, B01010001, B01001001, B01000111, B00000000, // Z
  2, 8, B01111111, B01000001, B00000000, B00000000, B00000000, // [
  4, 8, B00000001, B00000110, B00011000, B01100000, B00000000, // \ 
  2, 8, B01000001, B01111111, B00000000, B00000000, B00000000, // ]
  3, 8, B00000010, B00000001, B00000010, B00000000, B00000000, // ^
  4, 8, B01000000, B01000000, B01000000, B01000000, B00000000, // _
  2, 8, B00000001, B00000010, B00000000, B00000000, B00000000, // `
  4, 8, B00100000, B01010100, B01010100, B01111000, B00000000, // a
  4, 8, B01111111, B01000100, B01000100, B00111000, B00000000, // b
  4, 8, B00111000, B01000100, B01000100, B00101000, B00000000, // c
  4, 8, B00111000, B01000100, B01000100, B01111111, B00000000, // d
  4, 8, B00111000, B01010100, B01010100, B00011000, B00000000, // e
  3, 8, B00000100, B01111110, B00000101, B00000000, B00000000, // f
  4, 8, B10011000, B10100100, B10100100, B01111000, B00000000, // g
  4, 8, B01111111, B00000100, B00000100, B01111000, B00000000, // h
  3, 8, B01000100, B01111101, B01000000, B00000000, B00000000, // i
  4, 8, B01000000, B10000000, B10000100, B01111101, B00000000, // j
  4, 8, B01111111, B00010000, B00101000, B01000100, B00000000, // k
  3, 8, B01000001, B01111111, B01000000, B00000000, B00000000, // l
  5, 8, B01111100, B00000100, B01111100, B00000100, B01111000, // m
  4, 8, B01111100, B00000100, B00000100, B01111000, B00000000, // n
  4, 8, B00111000, B01000100, B01000100, B00111000, B00000000, // o
  4, 8, B11111100, B00100100, B00100100, B00011000, B00000000, // p
  4, 8, B00011000, B00100100, B00100100, B11111100, B00000000, // q
  4, 8, B01111100, B00001000, B00000100, B00000100, B00000000, // r
  4, 8, B01001000, B01010100, B01010100, B00100100, B00000000, // s
  3, 8, B00000100, B00111111, B01000100, B00000000, B00000000, // t
  4, 8, B00111100, B01000000, B01000000, B01111100, B00000000, // u
  5, 8, B00011100, B00100000, B01000000, B00100000, B00011100, // v
  5, 8, B00111100, B01000000, B00111100, B01000000, B00111100, // w
  5, 8, B01000100, B00101000, B00010000, B00101000, B01000100, // x
  4, 8, B10011100, B10100000, B10100000, B01111100, B00000000, // y
  3, 8, B01100100, B01010100, B01001100, B00000000, B00000000, // z
  3, 8, B00001000, B00110110, B01000001, B00000000, B00000000, // {
  1, 8, B01111111, B00000000, B00000000, B00000000, B00000000, // |
  3, 8, B01000001, B00110110, B00001000, B00000000, B00000000, // }
  4, 8, B00001000, B00000100, B00001000, B00000100, B00000000, // ~
};

// Contains everything required to control a servo
class ServoConf {
  public:
    ServoConf() = default;

    ServoConf(int pin_,
              float angle_,
              float dutyCycleMin_,
              float dutyCycleMax_,
              float minAngle_,
              float maxAngle_) :
      pin(pin_),
      angle(angle_),
      dutyCycleMin(dutyCycleMin_),
      dutyCycleMax(dutyCycleMax_),
      minAngle(minAngle_),
      maxAngle(maxAngle_) {
      servo.attach(pin);
    }

    void update() {
      if (!active) return;

      int pwm = (int) map(angle, minAngle, maxAngle, dutyCycleMin, dutyCycleMax);
      servo.writeMicroseconds(pwm);
    }

    int pin = -1;
    float angle = 0;
    float dutyCycleMin = 0;
    float dutyCycleMax = 0;
    float minAngle = 0;
    float maxAngle = 0;
    bool active = true;
    Servo servo;
};


MaxMatrix disp(DISP_DIM_PIN, DISP_CS_PIN, DISP_CLR_PIN, 1);
byte buffer[10];
char output = ' ';

// Utility function to remove an element from a vector.
// This could be made MUCH nicer but I really cba...
template<typename T>
void removeElement(std::vector<T> &vec, int index) {
  std::vector<T> res;
  for (int i = 0; i < vec.size(); i++) {
    if (i != index) res.push_back(vec[i]);
  }
  vec.assign(res.begin(), res.end());
}

// Information about the current command
int cmdLen = 0;
bool awaitingSentinel = false;
String command;

// An array of ServoConf objects
std::vector<ServoConf> servos;
std::map<int, int> servoMap;

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
    Serial.println('h');
  } else {
    Serial.println('l');
  }
}

void command_analogue_read() {
  int pin = read_pin();
  int value = analogRead(pin);
  Serial.println(value);
}

void command_write(int level) {
  int pin = read_pin();
  digitalWrite(pin, level);
}

void command_mode(int mode) {
  int pin = read_pin();
  pinMode(pin, mode);
}

// Add a servo and add the servo reference in the map
void addServo(ServoConf conf, int id) {
  servoMap[id] = servos.size();
  servos.push_back(conf);
}

void updateServos() {
  for (int i = 0; i < servos.size(); i++) servos[i].update();
}

void printChar(char c) {
  if (c < 32) return;
  c -= 32;
  memcpy_P(buffer, CH + 7 * c, 7);
  m.writeSprite(0, 0, buffer);
  m.setColumn(buffer[0], 0);
}

// Split a string into components separated by commas. Ignore spaces
// and cast the values into floats. Returns a std::vector of floats.
std::vector<float> splitParams(String params) {
  std::vector<float> result;
  String current = "";

  for (int i = 0; i < params.length(); i++) {
    if (params[i] == ',') {
      result.push_back(current.toFloat());
      current = "";
    } else if (params[i] == ' ') {
      continue;
    } else {
      current += params[i];
    }
  }

  result.push_back(current.toFloat());
  return result;
}

void processCommand(String cmd) {
  String createServo = "ADDSERVO ";
  String setAngle = "SETANGLE ";
  String getAngle = "GETANGLE ";
  String enable = "ENABLE ";
  String disable = "DISABLE ";
  String setDisplay = "SETDISPLAY ";

  if (cmd.startsWith(createServo)) {
    // Set up a new Servo Motor
    // Parameters:
    //  0: Servo ID
    //  1: Pin
    //  2: Angle = 0
    //  3: Minimum duty cycle = 600
    //  4: Maximum duty cycle = 2400
    //  5: Minimum angle = 0
    //  6: Maximum angle = 180

    int ID = 0;
    int pin = 0;
    float angle = 0;
    float dutyCycleMin = 400;
    float dutyCycleMax = 2400;
    float minAngle = 0;
    float maxAngle = 180;

    String paramStr = cmd.substring(createServo.length());
    std::vector<float> params = splitParams(paramStr);

    if (params.size() < 2) {
      Serial.print("INVALID COMMAND");
      return;
    }

    // Extract the information
    ID = (int) params[0];
    pin = (int) params[1];

    if (params.size() >= 3) angle = params[2];
    if (params.size() >= 4) dutyCycleMin = params[3];
    if (params.size() >= 5) dutyCycleMax = params[4];
    if (params.size() >= 6) minAngle = params[5];
    if (params.size() >= 7) maxAngle = params[6];

    ServoConf conf(pin, angle, dutyCycleMin, dutyCycleMax, minAngle, maxAngle);
    addServo(conf, ID);
  }

  if (cmd.startsWith(setAngle)) {
    // Set the angle of a Servo Motor
    // Parameters:
    //  0: Servo ID
    //  1: Angle

    String paramStr = cmd.substring(setAngle.length());
    std::vector<float> params = splitParams(paramStr);

    if (params.size() != 2) {
      Serial.print("INVALID COMMAND");
      return;
    }

    int index = servoMap[(int) params[0]];
    servos[index].angle = params[1];
    servos[index].active = true;
  }

  if (cmd.startsWith(getAngle)) {
    // Get the angle of a Servo Motor
    // Parameters:
    //  0: Servo ID

    String paramStr = cmd.substring(getAngle.length());
    std::vector<float> params = splitParams(paramStr);

    if (params.size() != 1) {
      Serial.print("INVALID COMMAND");
      return;
    }

    int index = servoMap[(int) params[0]];
    servos[index].active = false;
  }

  if (cmd.startsWith(enable)) {
    // Enable a Servo Motor (i.e. attach())
    // Parameters:
    //  0: Servo ID

    String paramStr = cmd.substring(enable.length());
    std::vector<float> params = splitParams(paramStr);

    if (params.size() != 1) {
      Serial.print("INVALID COMMAND");
      return;
    }

    int index = servoMap[(int) params[0]];
    servos[index].active = true;
    servos[index].servo.attach(servos[index].pin);
  }

  if (cmd.startsWith(disable)) {
    // Disable a Servo Motor (i.e. no signal)
    // Parameters:
    //  0: Servo ID

    String paramStr = cmd.substring(disable.length());
    std::vector<float> params = splitParams(paramStr);

    if (params.size() != 1) {
      Serial.print("INVALID COMMAND");
      return;
    }

    int index = servoMap[(int) params[0]];
    servos[index].active = false;
    servos[index].servo.detach();
  }

  if (cmd.startsWith(setDisplay)) {
    String paramStr = cmd.substring(setDisplay.length());

    if (paramStr.length() != 1) {
      Serial.print("INVALID COMMAND");
      return;
    }

    char paramChars[1];
    paramStr.toCharArray(paramChars, 1);
    output = paramChars[0];
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  disp.init();
  disp.setIntensity(10);
}

void loop() {
  // Fetch all commands in the buffer, separated by a "#"
  while (Serial.available()) {
    char nextChar = Serial.read();

    if (nextChar != '#') {
      if (!awaitingSentinel) {
        // This is most likely a control command, so deal with it
        // differently

        switch (nextChar) {
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
            Serial.println(FW_VER);
            break;
          default:
            // A problem here: we do not know how to handle the command!
            // Just ignore this for now.
            break;
        }
      } else {
        command += nextChar;
        cmdLen++;
      }
    } else {
      if (awaitingSentinel) {
        // Deal with the command
        processCommand(command);
        Serial.print("\n");
      }

      awaitingSentinel = !awaitingSentinel;
      cmdLen = 0;
      command = "";
    }
  }
  printChar(output);
  updateServos();
}
