#include <Arduino.h>
#include "BluetoothSerial.h"
#include <LiquidCrystal_I2C.h>
// #include <EEPROM.h>

//debounce*//
#define debounce 50

//******* LCD setup **********//
LiquidCrystal_I2C lcd(0x3F, 16, 2);

//****** BLUETOOTH CODE **********
String device_name = "ESP32.......!";
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif

BluetoothSerial SerialBT;

char btdata = '0'; // VARIABLE FOR BLUETOOTH DATA INPUT VALUE

//GPIO Layout for push button//
const uint8_t swSpeedInc = 4;
const uint8_t swSpeedDec = 15;
const uint8_t swR_swing = 14;
const uint8_t swL_swing = 26;
const uint8_t swFwd = 27;
const uint8_t swMotor_3 = 2;
const uint8_t RST = 12;
const uint8_t Mode = 13;
const uint8_t swSwingInc = 33;
const uint8_t swSwingDec = 25;

//GPIO Layout for Speed Sensor//
const uint8_t RpmSensor1 = 34;
const uint8_t RpmSensor2 = 35;

const uint8_t speed = 36;

const uint8_t buzzer = 16; // esp32 connected with buzzer
uint8_t RPM = 2;
// uint8_t Kv = 20;
bool BallF_flag = false;
//..............ESP32 pin connected with motor and L298 motor driver...........//
const uint8_t motor_1 = 32; // Motor 1 is assumed to be the right motor
const uint8_t motor_2 = 19; // Motor 2 is assumed to be the left  motor
const uint8_t motor_3 = 18;
const uint8_t in_1 = 5;
const uint8_t in_2 = 17;

//.............variables saved in EEPROM....//
bool In_1_state = LOW;
bool In_2_state = HIGH;
String mode = "fwd";
uint8_t fwdSpeed = 0;
uint8_t L_swingSpeed_M1 = 0;
uint8_t L_swingSpeed_M2 = 0;
uint8_t R_swingSpeed_M1 = 0;
uint8_t R_swingSpeed_M2 = 0;

//........Counting the number of times the Swing inc or dec was pressed.........//
uint8_t countSwing = 1;
bool fwdM = false;
bool L_SM = false;
bool R_SM = false;

//...........current state of motor.........//
uint8_t dutycycle_M1 = 0;
uint8_t dutycycle_M2 = 0;
uint8_t dutycycle_M3 = 0;

//...........current state of speed and swing push button.........//
bool speedIncState = LOW;
bool speedDecState = LOW;
bool swingIncState = LOW;
bool swingDecState = LOW;

// add debounce to preSet button
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

//.......MODE VARs.......//
bool modeflag = false;
bool dir_clkW = true;
String display_direction = "ClockWise";

int count1 = 0;

float avg = 0; // average of the both duty cycle
//.........EEPROM Address.........//
uint8_t eepromSpeed_M1 = 0;
uint8_t eepromSpeed_M2 = 4;
uint8_t eepromSpeed_M3 = 8;
uint8_t eepromDir_1 = 12;
uint8_t eepromDir_2 = 16;

// Task variables
uint32_t countM1 = 0;
uint32_t countM2 = 0;
float RpmM1 = 0;
float RpmM2 = 0;
unsigned long delaymills = 0; // To provide a non blocking nature of code
bool checkRpm = false;

// Speed sensor
unsigned long speedtime = 0;
unsigned long balltime = 0;
bool speedFlag = true;
float ballSpeed = 0.0;
// Task Handler of the IR sensors
// TaskHandle_t rpmHandler;
// TaskHandle_t speedHandler;

//.............................function for buzzer..................//
void buzz(int time)
{
  digitalWrite(buzzer, HIGH);
  delay(time);
  digitalWrite(buzzer, LOW);
}

//.............................Set PWM..............................//
void setSpeed()
{
  ledcWrite(0, dutycycle_M1);
  ledcWrite(1, dutycycle_M2);
  ledcWrite(2, dutycycle_M3);

  // digitalWrite(in_1, LOW);
  // digitalWrite(in_2, LOW); // Put lcd code to indicate to wait
  // delay(10000);            // wait for the motor to gain the speed
  // buzz(100);
  // digitalWrite(in_1, In_1_state);
  // digitalWrite(in_2, In_2_state);
}

//.........write the dutycycle of motor M1 and M2..........//
void setDutycycle(String setMode)
{

  if (setMode == "fwd" && fwdM == true)
  {
    avg = (dutycycle_M1 + dutycycle_M2) / 2;
    dutycycle_M1 = avg;
    dutycycle_M2 = avg;
    countSwing = 1;
    fwdM = false;
  }

  else if (setMode == "L_swing" && L_SM == true)
  {
    avg = (dutycycle_M1 + dutycycle_M2) / 2;
    //   // Serial.println("H13");
    dutycycle_M1 = avg;
    dutycycle_M2 = avg;

    dutycycle_M1 -= (15 * countSwing);
    dutycycle_M2 += (15 * countSwing);
    L_SM = false;
  }

  else if (setMode == "R_swing" && R_SM == true)
  {
    avg = (dutycycle_M1 + dutycycle_M2) / 2;
    dutycycle_M1 = avg;
    dutycycle_M2 = avg;

    dutycycle_M1 += (15 * countSwing);
    dutycycle_M2 -= (15 * countSwing);
    R_SM = false;
  }

  // Serial.println("H15");
  digitalWrite(motor_1, dutycycle_M1);
  digitalWrite(motor_2, dutycycle_M2);
}

void decSwing()
{
  if (countSwing > 0)
  {
    if (mode == "L_swing")
    {
      dutycycle_M1 += 15;
      dutycycle_M2 -= 15;
    }
    else if (mode == "R_swing")
    {

      dutycycle_M1 -= 15;
      dutycycle_M2 += 15;
    }
    setDutycycle("NULL");
  }
}
void incSwing()
{
  if (mode == "L_swing")
  {

    dutycycle_M1 -= 15;
    dutycycle_M2 += 15;
  }
  else if (mode == "R_swing")
  {
    dutycycle_M1 += 15;
    dutycycle_M2 -= 15;
  }
  setDutycycle("NULL");
}

//..........upeate Serial monitor.................//
void updateSerial(bool serialUpdate)
{
  Serial.print("m1 :");
  Serial.println(dutycycle_M1);
  Serial.print("m2 :");
  Serial.println(dutycycle_M2);
  Serial.print("mode :");
  Serial.println(mode);
  Serial.print("ball feeder :");
  Serial.println(RPM);
  Serial.print("Swing val = ");
  Serial.println(countSwing);

  // updating LCD
  avg = (dutycycle_M1 + dutycycle_M2) / 2;

  // Printing on Bluetooth teminal
  SerialBT.print("Speed1 = "); // Display Dutycycle of Motor 1 on Bluetooth Terminal
  SerialBT.println(dutycycle_M1);
  SerialBT.print("Speed2 = "); // Display Dutycycle of Motor 1 on Bluetooth Terminal
  SerialBT.println(dutycycle_M2);
  SerialBT.print("RPM = ");
  SerialBT.println(RPM);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print((int)avg);
  lcd.setCursor(5, 0);
  lcd.print("K/h");
  lcd.setCursor(10, 0);
  lcd.print("BF");
  lcd.setCursor(13, 0);
  lcd.print(RPM);
  lcd.setCursor(0, 1);
  lcd.print("MODE ");
  lcd.setCursor(6, 1);
  lcd.print(mode);
  if (mode != "fwd")
  {
    lcd.setCursor(14, 1);
    lcd.print(countSwing);
  }
  serialUpdate = false;
}

// MODE//
// Setting -lcd

void settingLcd(uint8_t count)
{
  switch (count)
  {
  case 2:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ball Feeder Dir");
    lcd.setCursor(0, 1);
    lcd.print(display_direction);
    break;

  case 1:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Ball Per Minute");
    lcd.setCursor(1, 1);
    lcd.print(RPM);
    lcd.setCursor(6, 1);
    lcd.print("Ball/min");
    break;
  }
}

// //..........MODE.............//
void MODE()
{
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("SETTINGS");
  delay(750);

  Serial.println("INside the mode");
  Serial.print("modeflag = ");
  Serial.println(modeflag);
  uint8_t count = 0;
  while (modeflag)
  {
    delay(100);
    // Serial.println("Inside while");
    if (digitalRead(swSwingDec) == HIGH && count < 2)
    {
      delay(50);
      buzz(200);
      count += 1;
      settingLcd(count);
      Serial.print("count =");
      Serial.println(count);
    }
    if (digitalRead(swSwingInc) == HIGH && count > 1)
    {
      delay(50);
      buzz(200);
      count -= 1;
      settingLcd(count);
      Serial.print("count =");
      Serial.println(count);
    }

    switch (count)
    {
    case 2:
      if (digitalRead(swSpeedInc) == HIGH || digitalRead(swSpeedDec) == HIGH)
      {
        delay(50);
        buzz(200);
        dir_clkW = !(dir_clkW);

        if (dir_clkW == 0)
        {
          display_direction = "Anti-ClockWise";
          In_1_state = HIGH;
          In_2_state = LOW;
          digitalWrite(in_1, In_1_state);
          digitalWrite(in_2, In_2_state);
        }
        else if (dir_clkW == 1)
        {
          display_direction = "ClockWise";
          In_1_state = LOW;
          In_2_state = HIGH;
          digitalWrite(in_1, In_1_state);
          digitalWrite(in_2, In_2_state);
        }

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Ball Feeder Dir");
        lcd.setCursor(0, 1);
        lcd.print(display_direction);

        Serial.println(display_direction);
      }
      break;

    case 1:

      if (digitalRead(swSpeedInc) == HIGH && (RPM < 10))
      {
        delay(50);
        buzz(200);
        RPM += 1;
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("Ball Per Minute");
        lcd.setCursor(1, 1);
        lcd.print(RPM);
        lcd.setCursor(6, 1);
        lcd.print("Ball/min");
      }
      if (digitalRead(swSpeedDec) == HIGH && (RPM > 1))
      {
        delay(50);
        buzz(200);
        RPM -= 1;
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("Ball Per Minute");
        lcd.setCursor(1, 1);
        lcd.print(RPM);
        lcd.setCursor(6, 1);
        lcd.print("Ball/min");
      }

      break;
    }

    if (digitalRead(Mode) == HIGH)
    {
      delay(50);
      buzz(200);
      // dutycycle_M3 = RPM;
      switch (RPM)
      {
      case 1:
        dutycycle_M3 = 41;
        break;

      case 2:
        dutycycle_M3 = 50;
        break;
      case 3:
        dutycycle_M3 = 55;
        break;
      case 4:
        dutycycle_M3 = 60;
        break;

      case 5:
        dutycycle_M3 = 65;
        break;

      case 6:
        dutycycle_M3 = 70;
        break;
      case 7:
        dutycycle_M3 = 75;
        break;
      case 8:
        dutycycle_M3 = 80;
        break;
      case 9:
        dutycycle_M3 = 85;
        break;
      case 10:
        dutycycle_M3 = 90;
        break;
      // case 11:
      //   dutycycle_M3 = 100;
      //   break;
      // case 12:
      //   dutycycle_M3 = 105;
      //   break;
      // case 13:
      //   dutycycle_M3 = 110;
      //   break;
      // case 14:
      //   dutycycle_M3 = 115;
      //   break;
      // case 15:
      //   dutycycle_M3 = 120;
      //   break;
      // case 16:
      //   dutycycle_M3 = 125;
      //   break;
      // case 17:
      //   dutycycle_M3 = 130;
      //   break;
      // case 18:
      //   dutycycle_M3 = 135;
      //   break;
      // case 19:
      //   dutycycle_M3 = 140;
      //   break;
      // case 20:
      //   dutycycle_M3 = 145;
      //   break;
      }

      ledcWrite(2, dutycycle_M3);
      modeflag = false;
      Serial.println("Exiting mode");
    }
  }
}

void TaskRpm() // void *pvParameters)
{
  while (checkRpm)
  {
    if (digitalRead(RpmSensor1) == HIGH) // && (millis() - delaymills >= 3000))
    {
      countM1 += 1;
      Serial.println(countM1);
    }

    if (digitalRead(RpmSensor2) == HIGH) // && (millis() - delaymills >= 3000))
    {
      countM2 += 1;
      Serial.println(countM2);
    }

    if (millis() - delaymills > 3001)
    {
      RpmM1 = ((float)countM1 / 3.0) * 60;
      RpmM2 = ((float)countM2 / 3.0) * 60;
      countM1 = 0;
      countM2 = 0;
      checkRpm = false;
      break;
    }
  }

  Serial.print("Rpm of M1 = ");
  Serial.println(RpmM1);
  Serial.print("Rpm of M2 = ");
  Serial.println(RpmM2);

SerialBT.print("Rpm of M1 = ");
  SerialBT.println(RpmM1);
  SerialBT.print("Rpm of M2 = ");
  SerialBT.println(RpmM2);
  
  // checkRpm = false;
}

// void TaskSpeed(void * pvParameters)
// {
//  if(digitalRead(speed) == HIGH && speedFlag){
//   speedtime = millis();
//   speedFlag = false;
//  }
//  if(digitalRead(speed) == LOW){
//   balltime = millis() - speedtime;
//   speedFlag = true;
//  }
// ballSpeed = 19.444444 / balltime;
// Serial.print("Speed of ball = ");
// Serial.println(ballSpeed);
// }

void setup()
{
  Serial.begin(115200);
  Serial.println("I am READY!");
  SerialBT.begin(device_name);

  pinMode(motor_1, OUTPUT);
  pinMode(motor_2, OUTPUT);
  pinMode(motor_3, OUTPUT);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(buzzer, OUTPUT);

  pinMode(swSwingDec, INPUT);
  pinMode(swSwingInc, INPUT);
  pinMode(swL_swing, INPUT);
  pinMode(swR_swing, INPUT);
  pinMode(swSpeedDec, INPUT);
  pinMode(swSpeedInc, INPUT);
  pinMode(swMotor_3, INPUT);
  pinMode(swFwd, INPUT);
  pinMode(RST, INPUT);
  pinMode(Mode, INPUT);
  pinMode(RpmSensor1, INPUT);
  pinMode(RpmSensor2, INPUT);

  // Setting up lcd //
  lcd.init();      // Initialize LCD
  lcd.backlight(); // Turn on LCD backlight
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("WELCOME");
  lcd.setCursor(2, 1);
  lcd.print("Sports AMY");
  // delay(1000);
  Serial.println("START");

  // Tasks for speed measurement
  // xTaskCreate(
  //     TaskRpm,     /* Task function. */
  //     "Task1",     /* name of task. */
  //     100,       /* Stack size of task */
  //     NULL,        /* parameter of the task */
  //     1,           /* priority of the task */
  //     NULL); /* Task handle to keep track of created task */

  // xTaskCreate(
  //     TaskSpeed,     /* Task function. */
  //     "Task2",       /* name of task. */
  //     100,         /* Stack size of task */
  //     NULL,          /* parameter of the task */
  //     1,             /* priority of the task */
  //     NULL); /* Task handle to keep track of created task */

  // set dutycycle
  ledcSetup(0, 500, 8);
  ledcAttachPin(motor_1, 0);
  ledcSetup(1, 500, 8);
  ledcAttachPin(motor_2, 1);
  ledcSetup(2, 500, 8);
  ledcAttachPin(motor_3, 2);

  delay(1000);

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Please wait");

  // updateSerial(true);
  for (uint8_t i = 0; i <= 90; i++)
  {
    dutycycle_M1 = i;
    dutycycle_M2 = i;
    setSpeed();

    Serial.print("m1 :");
    Serial.println(dutycycle_M1);
    Serial.print("m2 :");
    Serial.println(dutycycle_M2);
    Serial.print("mode :");
    Serial.println(mode);
    delay(100);
  }
  updateSerial(true);
}

void loop()
{
  // Serial.println("LOOP");
  if (SerialBT.available())
  {
    btdata = SerialBT.read();
    Serial.print(" Bluetooth input: ");
    Serial.println(btdata);
  }
  //

  //.............button for increase and decrease speed and swing.....//
  speedDecState = digitalRead(swSpeedDec);
  speedIncState = digitalRead(swSpeedInc);
  swingDecState = digitalRead(swSwingDec);
  swingIncState = digitalRead(swSwingInc);

  //...........Ball feeder .............//
  if (digitalRead(Mode) == HIGH)
  {
    delay(50);
    buzz(200);
    Serial.println("MODE");
    modeflag = true;
    MODE();
    updateSerial(true);
  }

  if (digitalRead(swMotor_3) == HIGH)
  {
    delay(50);
    buzz(200);
    if (BallF_flag == false)
    {
      digitalWrite(in_1, In_1_state);
      digitalWrite(in_2, In_2_state);
      dutycycle_M3 = 50; // getDutycycle(RPM);
      ledcWrite(2, dutycycle_M3);
      BallF_flag = true;
      Serial.println("HIGH");
    }

    else
    {
      BallF_flag = false;
      dutycycle_M3 = 0;
      setSpeed();
      Serial.println("LOW");
    }
  }

  if (speedIncState == HIGH || btdata == 'i') // Increases the duty cycle of both the motors by 5%
  {
    if ((dutycycle_M1 <= 240) && (dutycycle_M2 <= 240))
    {
      delay(debounce);
      buzz(200);
      Serial.println("H11");
      dutycycle_M1 += 15; // Increases the speed of Motor 1
      dutycycle_M2 += 15; // Increases the speed of Motor 2
      setDutycycle("NULL");
      updateSerial(true);
      delay(200);
    }
    setSpeed();
    delaymills = millis();
    // checkRpm = true;
    // TaskRpm();
  }

  if (speedDecState == HIGH || btdata == 'k') // Decreases the duty cycle of both the motors by 5%
  {
    if ((dutycycle_M1 >= 15) && (dutycycle_M2 >= 15))
    {
      delay(debounce);
      buzz(200);
      Serial.println("H10");
      dutycycle_M1 -= 15; // Increases the speed of Motor 1
      dutycycle_M2 -= 15; // Increases the speed of Motor 2
      setDutycycle("NULL");
      updateSerial(true);
      delay(200);
    }
    setSpeed();
    delaymills = millis();
    // checkRpm = true;
    // TaskRpm();
  }

  //..........when pressing button forward (fwd) ........//

  if (digitalRead(swFwd) == HIGH || btdata == 'a')
  {
    delay(debounce);
    buzz(200);
    // Serial.println("H9");
    mode = "fwd";
    fwdM = true;
    setDutycycle(mode);
    updateSerial(true);
    setSpeed();
  }

  //..........when pressing button L_swing ........//

  if (digitalRead(swL_swing) == HIGH || btdata == 'b')
  {
    delay(debounce);
    buzz(200);
    // Serial.println("H8");
    mode = "L_swing";
    L_SM = true;
    setDutycycle(mode);
    updateSerial(true);
    setSpeed();
  }

  //..........when pressing button R_swing ........//

  if (digitalRead(swR_swing) == HIGH || btdata == 'c')
  {
    delay(debounce);
    buzz(200);
    // Serial.println("H7");
    mode = "R_swing";
    R_SM = true;
    setDutycycle(mode);
    updateSerial(true);
    setSpeed();
  }

  //...........increasing & decreasing swing Level.......//
  if ((digitalRead(swSwingDec) == HIGH || btdata == 't') && mode != "fwd")
  {
    if (((dutycycle_M1 >= 15) && (dutycycle_M2 >= 15)) && (countSwing > 1))
    {
      delay(debounce);
      countSwing -= 1;
      buzz(200);
      Serial.println("H1");
      decSwing();
      setSpeed();
      updateSerial(true);
    }
  }

  if ((digitalRead(swSwingInc) == HIGH || btdata == 'p') && mode != "fwd")
  {
    if ((dutycycle_M1 <= 240) && (dutycycle_M2 <= 240) && (countSwing < 20))
    {
      delay(debounce);
      countSwing += 1;
      buzz(200);
      Serial.println("H2");
      incSwing();
      setSpeed();
      updateSerial(true);
    }
  }

  if (digitalRead(RST) == HIGH || btdata == 'r')
  {
    Serial.println("RST");
    delay(debounce);
    buzz(200);
    dutycycle_M1 = 0;
    dutycycle_M2 = 0;
    digitalWrite(in_1, LOW);
    digitalWrite(in_2, LOW);
    setSpeed();
    updateSerial(true);
    lastDebounceTime = millis();
    while (digitalRead(RST))
    {
      if (millis() - lastDebounceTime >= 3000)
      {
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("RESETTING");
        SerialBT.println("Resetting");
        buzz(1000);
        ESP.restart();
      }
    }
  }
}