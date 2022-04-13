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

  #SETNUM num#
  Set number to be displayed on screen
  Parameters:
  0: Number

  #SETDOT dot#
  Set position of dot on screen
  Parameters:
  0: Dot Position
*/

#include <Arduino.h>
#include <Servo.h>

#include <ArduinoSTL.h> // Requires "ArduinoSTL" library => https://github.com/mike-matera/ArduinoSTL
#include <map>
#include <vector>

// We communicate with the power board at 115200 baud.
#define SERIAL_BAUD 115200
#define FW_VER 1

// Screen Settings
const int segmentPins[8] = {13, 8, 7, 6, 5, 4, 3, 2}; // dp, G, F, E, D, C, B, A
const int digitPins[4] = {9, 10, 11, 12};  // digits 1, 2, 3, 4
const int numeral[10] = {
  B11111100, //0
  B01100000, //1
  B11011010, //2
  B11110010, //3
  B01100110, //4
  B10110110, //5
  B10111110, //6
  B11100000, //7
  B11111110, //8
  B11100110, //9
};

// Screen Variables
int number = 0000;
int dot = 4;

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

// Raise to a power but return integer instead
int pown(int x, unsigned p) {
  int result = 1;
  while (p) {
    if (p & 0x1) result *= x;
    x *= x;
    p >>= 1;
  }
  return result;
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

void showDigit(int number, int digit) {
  for (int segment = 0; segment < 8; segment++) digitalWrite(segmentPins[segment], LOW);
  for (int i = 0; i < 4; i++) digitalWrite(digitPins[i], HIGH);

  digitalWrite(digitPins[digit], LOW);
  digitalWrite(segmentPins[0], digit == dot);
  for (int segment = 1; segment < 8; segment++) digitalWrite(segmentPins[segment], bitRead(numeral[number], segment));
}

void updateDisplay() {
  for (int digit = 0; digit < 4; digit++) {
    showDigit((number / pown(10, 3 - digit)) % 10, digit);
    delayMicroseconds(1);
  }
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
  String setNum = "SETNUM ";
  String setDot = "SETDOT ";

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

  if (cmd.startsWith(setNum)) {
    // Set number to be drawn
    // Parameters:
    //  0: Number

    String paramStr = cmd.substring(disable.length());
    std::vector<float> params = splitParams(paramStr);

    if (params.size() != 1) {
      Serial.print("INVALID COMMAND");
      return;
    }

    number = (int)params[0]
  }

  if (cmd.startsWith(setDot)) {
    // Set dot to be drawn
    // Parameters:
    //  0: Dot

    String paramStr = cmd.substring(disable.length());
    std::vector<float> params = splitParams(paramStr);

    if (params.size() != 1) {
      Serial.print("INVALID COMMAND");
      return;
    }

    dot = (int)params[0]
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  for (int i = 0; i < 8; i++) pinMode(segmentPins[i], OUTPUT);
  for (int i = 0; i < 4; i++) pinMode(digitPins[i], OUTPUT);
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

  updateServos();
  updateDisplay();
}
