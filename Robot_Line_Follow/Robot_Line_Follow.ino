#include <QTRSensors.h>
//#include <DRV8833.h> // nếu muốn.
//DRV8833 driver = DRV8833();
////////////////////////////////////////////////////////
/* Khai báo chân động cơ */
const int INA1 = 4, INA2 = 5, INB1 = 6, INB2 = 7;
/* Khai báo cảm biến */
const int S1 = A2, S2 = A3, S3 = A4, S4 = A5, S5 = A1;
const uint8_t SensorCount = 5;  // số lượng mắt của cảm biến
uint16_t sensorValues[SensorCount];
/* Khai báo nút nhấn */
int ButtonCalibrate = 12;
int ButtonStart = 12;
int currentState = HIGH, lastState = HIGH;
/* Khai báo PID */
float PID_out = 0;
float P, I, D;
// float kp = 0.135, ki = 0, kd = 0.0556; //low: 80 
// float kp = 0.22, ki = 0, kd = 0.12; //Medium: 150 
// float kp = 0.32, ki = 0, kd = 0.13; //flash: 210 
// float kp = 0.365, ki = 0, kd = 0.157; //max: 255 
// float kp = 0.55, ki = 0, kd = 0.16; //max: 255 12v
float kp = 0.135, ki = 0, kd = 0.0;
float value = 2100.0;
float lastError = 0;
/* Khai báo động cơ */
float Base_speed = 80;
float left_out = 0;
float right_out = 0;
/* Khai báo thêm */
bool check = true;
QTRSensors qtr;
typedef enum {
  Manual,
  Auto
} Mode;
int Pressing_Debounce_button(int pin) {
  currentState = digitalRead(pin);
  if (lastState == LOW && currentState == HIGH) {
    //Serial.println("The button is pressed");
    lastState = currentState;
    return 1;
  } else {
    //Serial.println("The button is released");
    lastState = currentState;
    return 0;
  }
}
void set_Output() {
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}
void set_INPUT() {
  //pinMode(ButtonCalibrate, INPUT_PULLUP);
  //pinMode(ButtonStart, INPUT_PULLUP);
  pinMode(ButtonCalibrate, INPUT);
  pinMode(ButtonStart, INPUT);
}
void init_QTR() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ S1, S2, S3, S4, S5 }, SensorCount);
  qtr.setEmitterPin(2);  // Chân báo tín hiệu
}
void manualCalibate() {
  /* Đặt các giá trị tính được vào đây */
  uint16_t minVal[5] = { 120, 148, 148, 120, 124 };
  uint16_t maxVal[5] = { 1468, 1520, 1736, 1688, 1668 };
  qtr.calibrationOn.initialized = true;
  qtr.calibrationOn.minimum = (uint16_t *)realloc(qtr.calibrationOn.minimum, sizeof(uint16_t) * 5);
  qtr.calibrationOn.maximum = (uint16_t *)realloc(qtr.calibrationOn.maximum, sizeof(uint16_t) * 5);
  for (uint8_t i = 0; i < SensorCount; i++) {
    qtr.calibrationOn.minimum[i] = minVal[i];
    qtr.calibrationOn.maximum[i] = maxVal[i];
  }
}
void autoCalibrate() {
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
}
void print_min_max_value() {
  // print the calibration minimum values measured when emitters were on
  Serial.print("Minimum: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print('\t');
  }
  Serial.println();
  //print the calibration maximum values measured when emitters were on
  Serial.print("Maximun: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print('\t');
  }
  Serial.println();
}
void mode_Calibrate(Mode mode) {
  digitalWrite(LED_BUILTIN, HIGH);
  if (mode == Auto) {
    Serial.print("Mode Auto");
    Serial.println();
    Serial.print("Press Button to Calibrate!");
    Serial.println();
    while (check) {
      if (Pressing_Debounce_button(ButtonCalibrate)) {
        Serial.print("Start Calibrate!");
        Serial.println();
        autoCalibrate();
        break;
      }
    }
  }
  if (mode == Manual) {
    Serial.print("Mode Manual");
    Serial.println();
    manualCalibate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  print_min_max_value();
}
void check_sensor(int position) {
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print(position);
  Serial.print('\t');
  Serial.print(PID_out);
  Serial.print('\t');
  Serial.print(left_out);
  Serial.print('\t');
  Serial.println(right_out);
  // delay(250);
}
void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  //Serial.println(position);
  //position = min(max(position,1000),3000);
  float error = value - (float)position;
  P = error;
  I = I + error;
  D = (error - lastError)/0.1;
  PID_out = P * kp + I * ki + D * kd;
  //PID_out = min(max(PID_out,-100.0),100.0);
  lastError = error;
}
void start_car() {
  Serial.println("press button to start!");
  while (check) {
    if (Pressing_Debounce_button(ButtonCalibrate)) {
      Serial.println("Start car!");
      break;
    }
  }
}
void Motor_Right(float speed, bool direction)  // 1 forward, 0 reverse
{
  if (direction) {
    analogWrite(INA1, 0);
    analogWrite(INA2, speed);
  } else {
    analogWrite(INA1, speed);
    analogWrite(INA2, 0);
  }
}
void Motor_Left(float speed, bool direction)  // 1 forward, 0 reverse
{
  if (direction) {
    analogWrite(INB1, speed);
    analogWrite(INB2, 0);
  } else {
    analogWrite(INB1, 0);
    analogWrite(INB2, speed);
  }
}
void Motor_control() {
  left_out = constrain(Base_speed + PID_out, -255, 255);  // or max(min(Base_speed + PID_out, 255), -255);
  right_out = constrain(Base_speed - PID_out, -255, 255);
  if (left_out >= 0) {
    Motor_Left(left_out, 1);
  } else {
    Motor_Left(abs(left_out), 0);
  }
  if (right_out >= 0) {
    Motor_Right(right_out, 1);
  } else {
    Motor_Right(abs(right_out), 0);
  }
  // delay(3);
}
void setup() {
  Serial.begin(9600);
  set_Output();
  set_INPUT();
  init_QTR();
  delay(500);
  mode_Calibrate(Auto);
  start_car();
}

void loop() {
  // check_sensor(qtr.readLineBlack(sensorValues));
  PID_control();
  // left_out = constrain(Base_speed + PID_out, -255, 255);  // or max(min(Base_speed + PID_out, 255), -255);
  // right_out = constrain(Base_speed - PID_out, -255, 255);
  Motor_control();
  //delay(50);
}