#include <Arduino.h>
#include <Servo.h>
#include <ArduinoSTL.h> // Requires "ArduinoSTL" library

// We communicate with the power board at 115200 baud.
#define SERIAL_BAUD 115200
#define FW_VER 1
#define MAX_NUM_SERVOS 10

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
  
  int pin = -1;
  float angle = 0;
  float dutyCycleMin = 0;
  float dutyCycleMax = 0;
  float minAngle = 0;
  float maxAngle = 0;
  Servo servo;
};

// Information about the current command
int cmdLen = 0;
bool awaitingSentinel = false;
String command;

// An array of ServoConf objects
ServoConf servos[MAX_NUM_SERVOS];

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

void updateServos() {
  for (int i = 0; i < MAX_NUM_SERVOS; i++) {
    if (servos[i].pin == -1) continue;
    
    int pwm = (int) map(servos[i].angle, servos[i].minAngle, servos[i].maxAngle, servos[i].dutyCycleMin, servos[i].dutyCycleMax);
    servos[i].servo.writeMicroseconds(pwm);
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
  String setAngle = "SETANG ";
  String getAngle = "GETANG ";
  
  if (cmd.startsWith(setAngle)) {
    // Set the angle of a Servo Motor
    // Parameters:
    //  0: Index
    //  1: Angle
    
    String paramStr = cmd.substring(setAngle.length());
    std::vector<float> params = splitParams(paramStr);
    
    if (params.size() != 2) {
      Serial.print("INVALID COMMAND");
      return;
    }
    
    servos[(int) params[0]].angle = params[1];
  }

  if (cmd.startsWith(getAngle)) {
    // Set the angle of a Servo Motor
    // Parameters:
    //  0: Index
    //  1: Angle
    
    String paramStr = cmd.substring(getAngle.length());
    std::vector<float> params = splitParams(paramStr);
    
    if (params.size() != 1) {
      Serial.print("INVALID COMMAND");
      return;
    }
    
    Serial.print(servos[(int) params[0]].angle);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Config for a NORMAL servo
  servos[0] = ServoConf(
    9,
    180,
    600,
    2400,
    0,
    180
  );

  // Connfig for BIG BOI servo
  servos[1] = ServoConf(
    10,
    180,
    500,
    2500,
    0,
    250
  );
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
}
