#include <Arduino.h>
#include "BluetoothSerial.h"
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

//******* LCD setup **********//
LiquidCrystal_I2C lcd(0x3F, 16, 2);

//****** BLUETOOTH CODE **********
String device_name = "ESP32 Paakhandi";
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
const uint8_t swMode = 12;
const uint8_t swMotor_3 = 2;
const uint8_t swRst = 13;
const uint8_t swSwingInc = 33;
const uint8_t swSwingDec = 25;

const uint8_t buzzer = 16; // esp32 connected with buzzer

//..............ESP32 pin connected with motor and L298 motor driver...........//
const uint8_t motor_1 = 32;
const uint8_t motor_2 = 19;
const uint8_t motor_3 = 18;
const uint8_t in_1 = 5;
const uint8_t in_2 = 17;

//.............variables saved in EEPROM....//
uint8_t speed_M1 = 0;
uint8_t speed_M2 = 0;
uint8_t speed_M3 = 0;
bool dir_1 = false;
bool dir_2 = false;
String mode = "fwd";
uint8_t fwdSpeed = 0;
uint8_t L_swingSpeed_M1 = 0;
uint8_t L_swingSpeed_M2 = 0;
uint8_t R_swingSpeed_M1 = 0;
uint8_t R_swingSpeed_M2 = 0;

//...........current state of motor.........//
uint8_t dutycycle_M1 = 0;
uint8_t dutycycle_M2 = 0;
uint8_t dutycycle_M3 = 0;

//...........current state of speed and swing push button.........//
bool speedIncState = HIGH;
bool speedDecState = HIGH;
bool swingIncState = HIGH;
bool swingDecState = HIGH;

// add debounce to preSet button
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

//.........EEPROM Address.........//
uint8_t eepromSpeed_M1 = 0;
uint8_t eepromSpeed_M2 = 4;
uint8_t eepromSpeed_M3 = 8;
uint8_t eepromDir_1 = 12;
uint8_t eepromDir_2 = 16;
uint8_t eepromFwd = 20;
// uint8_t eepromFwd_M2 = 24;
uint8_t eepromL_swing_M1 = 36;
uint8_t eepromL_swing_M2 = 24;
uint8_t eepromR_swing_M1 = 28;
uint8_t eepromR_swing_M2 = 32;

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
  analogWrite(motor_1, dutycycle_M1);
  analogWrite(motor_2, dutycycle_M2);
}

//.............................Display data to LCD.................//
void displayData()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L_M");
  lcd.setCursor(13, 0);
  lcd.print("R_M");
  lcd.setCursor(0, 1);
  lcd.print(dutycycle_M1);
  lcd.setCursor(13, 1);
  lcd.print(dutycycle_M2);
  lcd.setCursor(5, 0);
  // lcd.print(mode_dispaly);
  if (SerialBT.connected())
  {
    Serial.println("device is connected successfully");
    lcd.setCursor(8, 1);
    lcd.print("CONN");
  }
  else
  {
    Serial.println("not connected");
    lcd.setCursor(8, 1);
    lcd.print("NOT_CONN");
  }
}

//.........write the dutycycle of motor M1 and M2..........//
void setDutycycle(String setMode)
{
  if (setMode == "fwd")
  {
    Serial.println("H12");
    dutycycle_M1 = fwdSpeed;
    dutycycle_M2 = fwdSpeed;
  }

  else if (setMode == "L_swing")
  {
    Serial.println("H13");
    dutycycle_M1 = L_swingSpeed_M1;
    dutycycle_M2 = L_swingSpeed_M2;
  }
  else if (setMode == "R_swing")
  {
    Serial.println("H14");
    dutycycle_M1 = R_swingSpeed_M1;
    dutycycle_M2 = R_swingSpeed_M2;
  }

  Serial.println("H15");
  digitalWrite(motor_1, dutycycle_M1);
  digitalWrite(motor_2, dutycycle_M2);
}

//*PRINTING ON BLUETOOH TERMINAL//
void BT_print()
{
  SerialBT.print("Speed1 = "); // Display Dutycycle of Motor 1 on Bluetooth Terminal
  SerialBT.println(speed_M1);
  SerialBT.print("Speed2 = "); // Display Dutycycle of Motor 1 on Bluetooth Terminal
  SerialBT.println(speed_M2);
}

//..........save Forward mode............//
void saveFwd()
{
  fwdSpeed = dutycycle_M1;
  EEPROM.put(eepromFwd, fwdSpeed);
  EEPROM.commit(); // Store the values in EEPROM
  Serial.println("saved");
  delay(10);
}

//..........save L_swing mode............//
void saveL_swing()
{
  L_swingSpeed_M1 = dutycycle_M1;
  L_swingSpeed_M2 = dutycycle_M2;
  EEPROM.put(eepromL_swing_M1, L_swingSpeed_M1);
  EEPROM.put(eepromL_swing_M2, L_swingSpeed_M2);
  Serial.println("saved");
  EEPROM.commit(); // Store the values in EEPROM
  delay(10);
}

//..........save Forward mode............//
void saveR_swing()
{
  R_swingSpeed_M1 = dutycycle_M1;
  R_swingSpeed_M2 = dutycycle_M2;
  EEPROM.put(eepromR_swing_M1, R_swingSpeed_M1);
  EEPROM.put(eepromR_swing_M2, R_swingSpeed_M2);
  Serial.println("saved");
  EEPROM.commit(); // Store the values in EEPROM
  delay(10);
}

void decSwing()
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
  Serial.println(speed_M3);
  serialUpdate = false;
}


//..........setup function for ball feeder.............//
void setupBall_F(bool setBall_F){
  while (setBall_F)
  {
    if(digitalRead(swSpeedInc)){
      speed_M3 += 15;
      updateSerial(true);
    }
    if(digitalRead(swSpeedDec)){
      speed_M3 -= 15;
      updateSerial(true);
    }
    if(swMotor_3)
      break;
  }
  
}
void setup()
{
  Serial.begin(115200);
  Serial.println("I am READY!");
  SerialBT.begin(device_name);

  //............Start and get value for eeprom..............//
  EEPROM.begin(512);
  EEPROM.get(eepromFwd, fwdSpeed);
  EEPROM.get(eepromL_swing_M1, L_swingSpeed_M1);
  EEPROM.get(eepromL_swing_M2, L_swingSpeed_M2);
  EEPROM.get(eepromR_swing_M1, R_swingSpeed_M1);
  EEPROM.get(eepromR_swing_M2, R_swingSpeed_M2);

  pinMode(motor_1, OUTPUT);
  pinMode(motor_2, OUTPUT);
  pinMode(motor_3, OUTPUT);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(buzzer, OUTPUT);

  pinMode(swSwingDec, INPUT);
  pinMode(swSpeedInc, INPUT);
  pinMode(swL_swing, INPUT);
  pinMode(swR_swing, INPUT);
  pinMode(swSpeedDec, INPUT);
  pinMode(swSpeedInc, INPUT);
  pinMode(swMotor_3, INPUT);
  pinMode(swFwd, INPUT);
  pinMode(swRst, INPUT);
  pinMode(swMode, INPUT);

  // Setting up lcd //
  lcd.init();      // Initialize LCD
  lcd.backlight(); // Turn on LCD backlight
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("WELCOME");
  lcd.setCursor(2, 1);
  lcd.print("Who are you ?");
  // delay(1000);
  Serial.println("START");

  // set dutycycle
  dutycycle_M1 = 0;
  dutycycle_M2 = 0;
  digitalWrite(motor_1, dutycycle_M1);
  digitalWrite(motor_2, dutycycle_M2);
}

void loop()
{
  if (SerialBT.available())
  {
    btdata = SerialBT.read();
    Serial.print(" Bluetooth input: ");
    Serial.println(btdata);
  }
  delay(100);

  //.............button for increase and decrease speed and swing.....//
  speedDecState = digitalRead(swSpeedDec);
  speedIncState = digitalRead(swSpeedInc);
  swingDecState = digitalRead(swSwingDec);
  swingIncState = digitalRead(swSwingInc);

  //...........Ball feeder .............//
  if (digitalRead(swMotor_3) == HIGH || btdata == 'g')
  {
    lastDebounceTime = millis();
    buzz(200);
    while (digitalRead(swMotor_3))
    {
      if (millis() - lastDebounceTime >= 3000)
      {
        buzz(1000);
        setupBall_F(true);
        break;
      }
    }

    if (dutycycle_M3 == 0)
    {
      dutycycle_M3 = speed_M3;
      digitalWrite(motor_3, dutycycle_M3);
    }
    else
    {
      dutycycle_M3 = 0;
      digitalWrite(motor_3, 0);
    }
  }

  if (speedIncState == HIGH || btdata == 'i') // Increases the duty cycle of both the motors by 5%
  {
    if ((dutycycle_M1 <= 240) && (dutycycle_M2 <= 240))
    {
      buzz(200);
      Serial.println("H11");
      dutycycle_M1 += 15; // Increases the speed of Motor 1
      dutycycle_M2 += 15; // Increases the speed of Motor 2
      setDutycycle("NULL");
      updateSerial(true);
      delay(200);
    }
  }

  if (speedDecState == HIGH || btdata == 'k') // Decreases the duty cycle of both the motors by 5%
  {
    if ((dutycycle_M1 >= 15) && (dutycycle_M2 >= 15))
    {
      buzz(200);
      Serial.println("H10");
      dutycycle_M1 -= 15; // Increases the speed of Motor 1
      dutycycle_M2 -= 15; // Increases the speed of Motor 2
      setDutycycle("NULL");
      updateSerial(true);
      delay(200);
    }
  }

  //..........when pressing button forward (fwd) ........//

  if (digitalRead(swFwd) == HIGH || btdata == 'a')
  {
    lastDebounceTime = millis();
    buzz(200);
    while (digitalRead(swFwd))
    {
      if (millis() - lastDebounceTime >= 3000)
      {
        buzz(1000);
        saveFwd();
        break;
      }
    }

    Serial.println("H9");
    mode = "fwd";
    setDutycycle(mode);
    updateSerial(true);
  }

  //..........when pressing button L_swing ........//

  if (digitalRead(swL_swing) == HIGH || btdata == 'b')
  {
    lastDebounceTime = millis();
    buzz(200);
    while (digitalRead(swL_swing))
    {
      if (millis() - lastDebounceTime >= 3000)
      {
        buzz(1000);
        saveL_swing();
        break;
      }
    }

    Serial.println("H8");
    mode = "L_swing";
    setDutycycle(mode);
    updateSerial(true);
  }

  //..........when pressing button R_swing ........//

  if (digitalRead(swR_swing) == HIGH || btdata == 'c')
  {
    lastDebounceTime = millis();
    buzz(200);
    while (digitalRead(swR_swing))
    {
      if (millis() - lastDebounceTime >= 3000)
      {
        buzz(1000);
        saveR_swing();
        break;
      }
    }

    Serial.println("H7");
    mode = "R_swing";
    setDutycycle(mode);
    updateSerial(true);
  }

  //...........increasing & decreasing swing Level.......//
  if ((swingDecState == HIGH || btdata == 't') && mode != "fwd")
  {
    Serial.println("H1");
    decSwing();
    updateSerial(true);
  }

  if ((swingIncState == HIGH || btdata == 'p') && mode != "fwd")
  {
    Serial.println("H2");
    incSwing();
    updateSerial(true);
  }
}