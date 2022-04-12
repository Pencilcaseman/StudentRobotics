const int segmentPins[8] = {0, 8, 7, 6, 5, 4, 3, 2}; //dp, G, F, E, D, C, B, A
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

int number = 3462;

int pown(int x, unsigned p) {
  int result = 1;
  while (p) {
    if (p & 0x1) result *= x;
    x *= x;
    p >>= 1;
  }
  return result;
}

void showDigit(int number, int digit) {
  for (int segment = 1; segment < 8; segment++) digitalWrite(segmentPins[segment], LOW);
  for (int i = 0; i < 4; i++) digitalWrite(digitPins[i], HIGH);

  digitalWrite(digitPins[digit], LOW);
  for (int segment = 1; segment < 8; segment++) digitalWrite(segmentPins[segment], bitRead(numeral[number], segment));
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  for (int i = 0; i < 8; i++) pinMode(segmentPins[i], OUTPUT);
  for (int i = 0; i < 4; i++) pinMode(digitPins[i], OUTPUT);
}

void loop() {
  for (int digit = 0; digit < 4; digit++) {
    showDigit((number / pown(10, 3 - digit)) % 10, digit);
    delayMicroseconds(10);
  }
}
