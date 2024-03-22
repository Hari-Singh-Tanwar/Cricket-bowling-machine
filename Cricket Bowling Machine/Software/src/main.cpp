#include <Arduino.h>
#include <Wire.h>

//******* LCD setup **********//
#include <LiquidCrystal_I2C.h> // Address 0x3F
LiquidCrystal_I2C lcd(0x3F, 16, 2);
uint8_t text = 0;
uint8_t text2 = 0; // for setup mode display
uint8_t text3 = 0; // for preset_reset mode display

#include "BluetoothSerial.h"
//****** BLUETOOTH CODE **********

String device_name = "ESP32 Sports AMI"; // ESP32_MOTOR_CONTROLLER

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif

BluetoothSerial SerialBT;

char btdata = '0'; // VARIABLE FOR BLUETOOTH DATA INPUT VALUE

//************************************************//

// defines //
#define speed_change 5 // change of duty cycle at each button press
#define debounce 50    // debounce time in ms
#define RPM 5          // RPM of ball feeder

//***************GPIO Layout*************//
const uint8_t speed_Inc = 33;
const uint8_t speed_Dec = 25;
const uint8_t R_swing = 14;
const uint8_t L_swing = 26;
const uint8_t Fwd = 27;
const uint8_t Mode = 12;
const uint8_t Ball_feeder = 2;
const uint8_t RST = 13;
const uint8_t Swing_Inc = 4;
const uint8_t Swing_Dec = 15;

//********INPUT Var*******//
bool speed_Inc_state = LOW;
bool speed_Dec_state = LOW;
bool Swing_Inc_state = LOW;
bool Swing_Dec_state = LOW;
bool R_swing_state = LOW;
bool L_swing_state = LOW;
bool Fwd_state = LOW;
bool Mode_state = LOW;
bool Ball_feeder_state = LOW;
bool RST_state = LOW;

const uint8_t Motor1 = 32;
const uint8_t Motor2 = 19;
const uint8_t Motor3 = 18;
const uint8_t In1 = 5;
const uint8_t In2 = 17;

const uint8_t Speed1 = 34;
const uint8_t Speed2 = 35;
const uint8_t Hall1 = 36;
const uint8_t Hall2 = 39;

const uint8_t Buzzer = 16;

// PWM properties //
const uint16_t freq = 1000; // Frequency of PWM signal
const uint8_t res = 8;      // Resolution of PWM signsl (8 -> 8bit = 0 - 255)
const uint8_t ch0 = 0;      // Channel 0 of PWM signal
const uint8_t ch1 = 1;      // Channel 1 of PWM signal
const uint8_t ch2 = 2;      // Channel 2 of PWM signal

float speed_M1 = 0;
float speed_M2 = 0;
float dutyCycleM1 = 50;
float dutyCycleM2 = 50;
uint8_t dutyprint;

uint8_t mode_swing = 1;
uint8_t swing = 0;
float M1 = 1;
float M2 = 1;
String mode_dispaly = "Stright";

bool direction = 1; // 1 = forward , 0 = backward for the Ball feeder
uint8_t ballFeeder_duration = 10;
// uint8_t ballfeeder_speed = 1;
unsigned long rotation_time = (60000 / RPM);

bool mode_flag = 0;

//*****Task Handler******//
// TaskHandle_t ballSpeedHandler = NULL;
// void ballSpeed(void *parameters); // For Ball Speed

//**********PRINTING ON BLUETOOH TERMINAL***************//
bool btflag = 0;
void BT_print(void)
{
    // **** BLUETOOTH SCREEN PRINT ******
    while (btflag)
    {
        SerialBT.print("Speed1 = "); // Display Dutycycle of Motor 1 on Bluetooth Terminal
        SerialBT.println(speed_M1);
        SerialBT.print("Speed2 = "); // Display Dutycycle of Motor 1 on Bluetooth Terminal
        SerialBT.println(speed_M2);

        btdata = '0'; // CHANGE BTDATA VALUE TO "00"
        btflag = 0;
    }
}

//*****Buzzer******//
void beep(void) // function of buzzer for beep sound
{
    digitalWrite(Buzzer, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(Buzzer, LOW);
}

void restart_beep(void)
{
    digitalWrite(Buzzer, HIGH);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    digitalWrite(Buzzer, LOW);
}

bool state = LOW;
unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0;
bool feeder_state = LOW;

void feeder()
{
    if (direction == 1)
    {
        digitalWrite(In1, HIGH);
        digitalWrite(In2, LOW);
    }
    if (direction == 0)
    {
        digitalWrite(In1, LOW);
        digitalWrite(In2, HIGH);
    }
    // digitalWrite(Motor3, HIGH);
    // vTaskDelay(rotation_time / portTICK_PERIOD_MS); // debouncing switch for 50 ms
    // digitalWrite(Motor3, LOW);
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= rotation_time)
    {
        previousMillis = currentMillis;
        state != state;
        digitalWrite(Motor3, state);
    }
}

String display_direction = "Stright";

void mode()
{
    uint8_t count = 0;
    while (mode_flag)
    {
        lcd.clear();
        lcd.setCursor(6, 0);
        lcd.print("MODE");
        delay(500);
        if (digitalRead(Swing_Dec) == HIGH && count < 2)
        {
            count += 1;
        }
        if (digitalRead(Swing_Inc) == HIGH && count > 0)
        {
            count -= 1;
        }

        switch (count)
        {
        case 0:

            if (digitalRead(speed_Inc) == HIGH || digitalRead(speed_Dec) == HIGH)
            {
                direction = !(direction);
            }

            if (direction == 0)
            {
                display_direction = "REV";
            }
            else if (direction == 1)
            {
                display_direction = "FWD";
            }

            lcd.clear();
            lcd.setCursor(2, 0);
            lcd.print("Ball Feeder");
            lcd.setCursor(0, 1);
            lcd.print("Direction");
            lcd.setCursor(12, 1);
            lcd.print(display_direction);
            break;

        case 1:

            if (digitalRead(speed_Inc) == HIGH)
            {
                ballFeeder_duration += 5;
            }
            if (digitalRead(speed_Dec) == HIGH && (ballFeeder_duration > 5))
            {
                ballFeeder_duration -= 5;
            }
            lcd.clear();
            lcd.setCursor(2, 0);
            lcd.print("Ball Feeder");
            lcd.setCursor(1, 1);
            lcd.print("Time");
            lcd.setCursor(9, 1);
            lcd.print(ballFeeder_duration);
            lcd.setCursor(12, 1);
            lcd.print("Sec");

            break;
        }
        if (digitalRead(Mode) == HIGH)
        {
            mode_flag = 0;
        }
    }
}
//------------------------------------------------------------------------------------//

void setup()
{
    Serial.begin(115200);
    SerialBT.begin(device_name);

    //*****Push Buttons*****//
    pinMode(speed_Inc, INPUT);
    pinMode(speed_Dec, INPUT);
    pinMode(R_swing, INPUT);
    pinMode(L_swing, INPUT);
    pinMode(Mode, INPUT);
    pinMode(Fwd, INPUT);
    pinMode(RST, INPUT);
    pinMode(Swing_Inc, INPUT);
    pinMode(Swing_Dec, INPUT);
    pinMode(Ball_feeder, INPUT);

    //*****Speed Sensor*****//
    pinMode(Speed1, INPUT);
    pinMode(Speed2, INPUT);
    pinMode(Hall1, INPUT);
    pinMode(Hall2, INPUT);

    //*****Output******//
    pinMode(In1, OUTPUT);
    pinMode(In2, OUTPUT);

    pinMode(Buzzer, OUTPUT);

    ledcSetup(ch0, freq, res);
    ledcAttachPin(Motor1, ch0);
    ledcSetup(ch1, freq, res);
    ledcAttachPin(Motor2, ch1);
    // ledcSetup(ch2, freq, res);
    // ledcAttachPin(Motor3, ch2);

    // Setting up lcd //
    lcd.init();      // Initialize LCD
    lcd.backlight(); // Turn on LCD backlight
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("WELCOME");
    lcd.setCursor(2, 1);
    lcd.print("Sports AMY");

    // for (int i = 0; i < 8; i = i + 5)
    // {
    //     float speed_;
    //     speed_ = i * (2.55) * M1; // Changes Dutycycle into speed of motor i.e between 0 - 255
    //     ledcWrite(ch0, (uint8_t)speed_);
    //     ledcWrite(ch1, (uint8_t)speed_);
    //     delay(500);
    // }
    Serial.println("START");
    // Task to establish connection with the wifi
    // xTaskCreatePinnedToCore(
    //     ballSpeed,              // function name which you want to run
    //     "Gets the  ball speed", // function description
    //     2000,                   // stack size
    //     NULL,                   // parameters of function
    //     1,                      // task priority
    //     &ballSpeedHandler,      // task handler
    //     0                       // it will run in core 0
    // );
}

void loop()
{
    //******* CHECK BLUETOOTH IS CONNECTED OR NOT *******

    if (SerialBT.connected())
    {
        Serial.println("Bluetooth device is connected");
    }
    else
    {
        Serial.println("Bluetooth device is disconnected");
    }

    //******* BLUETOOTH INPUT VALUE IN "btdata" VARIABLE ******

    if (SerialBT.available())
    {
        btdata = SerialBT.read();
        Serial.print(" Bluetooth input  ");
        Serial.println(btdata);
    }

    speed_Inc_state = digitalRead(speed_Inc);
    speed_Dec_state = digitalRead(speed_Dec);
    Swing_Inc_state = digitalRead(Swing_Inc);
    Swing_Dec_state = digitalRead(Swing_Dec);
    Fwd_state = digitalRead(Fwd);
    R_swing_state = digitalRead(R_swing);
    L_swing_state = digitalRead(L_swing);
    Mode_state = digitalRead(Mode);
    Ball_feeder_state = digitalRead(Ball_feeder);
    speed_Inc_state = digitalRead(speed_Inc);

    if (speed_Inc_state == HIGH || btdata == 'i') // Increases the duty cycle of both the motors by 5%
    {
        if ((dutyCycleM1 < 100) && (dutyCycleM2 < 100))
        {
            vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
            dutyCycleM1 += speed_change;               // Increases the speed of Motor 1
            dutyCycleM2 += speed_change;               // Increases the speed of Motor 2
            beep();                                    // Beep to indicate the exicution
            btflag = 1;
        }
    }

    if (speed_Dec_state == HIGH || btdata == 'k') // Decreases the duty cycle of both the motors by 5%
    {
        if ((dutyCycleM1 != 0) && (dutyCycleM2 != 0))
        {
            vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
            dutyCycleM1 -= speed_change;               // Decreases the speed of Motor 1
            dutyCycleM2 -= speed_change;               // Decreases the speed of Motor 2
            beep();                                    // Beep to indicate the exicution
            btflag = 1;
        }
    }

    if (Fwd_state == HIGH || btdata == 'a')
    {
        vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
        beep();
        btflag = 1;
        mode_swing = 1;
        swing = 0;
        mode_dispaly = "Stright";
    }

    if (L_swing_state == HIGH || btdata == 'b')
    {
        vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
        beep();
        btflag = 1;
        mode_swing = 2;
        swing = 1;
        mode_dispaly = "Leg Spin";
    }

    if (R_swing_state == HIGH || btdata == 'c')
    {
        vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
        beep();
        btflag = 1;
        mode_swing = 3;
        swing = 1;
        mode_dispaly = "Off spin";
    }

    if ((Swing_Inc_state == HIGH || btdata == 'd') && (swing < 5))
    {
        vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
        beep();
        btflag = 1;
        swing += 1;
    }

    if ((Swing_Dec_state == HIGH || btdata == 'e') && (swing > 1))
    {
        vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
        beep();
        btflag = 1;
        swing -= 1;
    }

    if (Ball_feeder_state == HIGH || btdata == 'f')
    {
        vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
        beep();
        btflag = 1;
        feeder_state = !feeder_state;
    }

    if (Mode_state == HIGH || btdata == 'g')
    {
        vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
        beep();
        btflag = 1;
        mode_flag = 1;
        mode();
    }

    if (RST_state == HIGH || btdata == 'r')
    {
        vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
        beep();
        btflag = 1;
        ESP.restart();
    }

    if (btdata == 'j') // Stops both the motors
    {
        vTaskDelay(debounce / portTICK_PERIOD_MS); // debouncing switch for 50 ms
        beep();
        btflag = 1;
        dutyCycleM1 = 0; // Motor 1 Stop
        dutyCycleM2 = 0; // Motor 2 Stop
    }

    switch (mode_swing)
    {
    case 1:
        M1 = 1.0;
        M2 = 1.0;
        break;
    case 2:
        switch (swing)
        {
        case 1:
            M1 = (1.0 / 1.5);
            M2 = 1.0;
            break;
        case 2:
            M1 = (1.0 / 3.0);
            M2 = 1.0;
            break;
        case 3:
            M1 = (1.0 / 5.0);
            M2 = 1.0;
            break;
        case 4:
            M1 = (1.0 / 7.0);
            M2 = 1.0;
            break;
        case 5:
            M1 = (1.0 / 9.0);
            M2 = 1.0;
            break;
        }
    case 3:
        switch (swing)
        {
        case 1:
            M1 = 1.0;
            M2 = (1.0 / 1.5);
            break;
        case 2:
            M1 = 1.0;
            M2 = (1.0 / 3.0);
            break;
        case 3:
            M1 = 1;
            M2 = (1.0 / 5.0);
            break;
        case 4:
            M1 = 1.0;
            M2 = (1.0 / 7.0);
            break;
        case 5:
            M1 = 1.0;
            M2 = (1.0 / 9.0);
            break;
        }
    default:
        break;
    }

    // Motor 1
    speed_M1 = dutyCycleM1 * (2.55) * M1; // Changes Dutycycle into speed of motor i.e between 0 - 255
    ledcWrite(ch0, (uint8_t)speed_M1);    // Generates PWM output for Motor 1
    Serial.print("DutyCycle of Motor1 = ");
    Serial.print(dutyCycleM1);
    Serial.print(" Speed = ");
    Serial.println((uint8_t)speed_M1);
    // Motor 2
    speed_M2 = dutyCycleM2 * (2.55) * M2; // Changes Dutycycle into speed of motor i.e between 0 - 255
    ledcWrite(ch1, (uint8_t)speed_M2);    // Generates PWM output for Motor 2
    Serial.print("DutyCycle of Motor 2 = ");
    Serial.print(dutyCycleM2);
    Serial.print(" Speed = ");
    Serial.println((uint8_t)speed_M2);

    dutyprint = (speed_M1 + speed_M2) / 2;
    // Ball Feeder
    if (feeder_state)
    {
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis1 >= (ballFeeder_duration * 1000))
        {
            // save the last time you blinked the LED
            previousMillis1 = currentMillis;

            feeder();
        }
    }

    //**************LCD****************//
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("SPEED");
    lcd.setCursor(9, 0);
    lcd.print(dutyprint);
    lcd.setCursor(0, 1);
    lcd.print(mode_dispaly);
    lcd.setCursor(8, 1);
    lcd.print(swing);

    //***********Printing on BT device***********//
    BT_print();
    delay(150);
}