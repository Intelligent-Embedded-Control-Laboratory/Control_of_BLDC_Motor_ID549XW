#include <Arduino.h>
#include <LS7366R.h>

/* Pin Definition */
const uint8_t PIN_MOTOR_EN = 23;      // 馬達的啟動腳位 (HIGH=Enable, LOW=Disable)，給 Low 會直接停轉 (TODO:)
const uint8_t PIN_MOTOR_DIR = 21;     // 馬達的正反轉腳位 (HIGH=其中一轉，LOW=另一轉)
const uint8_t PIN_MOTOR_VOLTAGE = 22; // 馬達的電壓腳位，要注意的是，命令映射是相反。也就是說，PWM 值給滿，其實是最低的電壓輸出給馬達，PWM 值給最低，是最高的電壓輸出給馬達。
const uint8_t PIN_QEI = 10; // 解馬達 Encoder 的晶片腳位

const int PWM_FREQ = 20000;                    // 對於這顆馬達 (ID549XW) 是一個固定參數
const int PWM_RES = 12;                        // 對於這顆馬達 (ID549XW) 是一個固定參數
const int PWM_MAX = (int)(pow(2.0, PWM_RES) - 1); // PWM 命令的最大值
const int CONFIG_CONTROL_LOOP_PERIOD_MICROS = 1000.0;            // 內部中斷的執行時間間隔

/* Interrupt Service Routine (ISR) */
unsigned t_enter = 0;
double T = CONFIG_CONTROL_LOOP_PERIOD_MICROS / 1000000.0;
int idx = 0;
double t = 0.0;

/* Encoder Sensor of Motor */
LS7366R encoder;
long int pos = 0; // Position of motor
long int pos_p = 0; // Previous step position of motor
double vel = 0.0; // Velocity of motor

/* Control */
double u = 0.0;


/* Function Declaration */
void control_loop();
void motor_ID549XW_update(double torque);
void print_data();
double sign(double u_in);

/* Main */
void setup()
{
    Serial.begin(115200);

    /* Pin Mode Settings */
    pinMode(PIN_MOTOR_EN, OUTPUT);
    pinMode(PIN_MOTOR_VOLTAGE, OUTPUT);
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    analogWriteFrequency(PIN_MOTOR_VOLTAGE, PWM_FREQ);
    analogWriteResolution(PWM_RES);

    /* Initialize default values of Pins */
    digitalWrite(PIN_MOTOR_EN, HIGH);
    digitalWrite(PIN_MOTOR_DIR, LOW);
    analogWrite(PIN_MOTOR_VOLTAGE, (int)PWM_MAX);

    /* Initialize LS7366R Pulse Decoder */
    encoder.begin(&PIN_QEI, 1);

    /* Initialize Private Timer */
    t_enter = micros();
}

void loop()
{
    if (micros() - t_enter >= CONFIG_CONTROL_LOOP_PERIOD_MICROS)
    {
        t_enter = micros();

        control_loop();
    }
}

void control_loop()
{
    idx++;
    t = (double)(idx)*T;

    /* Update Motor Position */
    encoder.read();
    pos = encoder.get_pulse()[0];
    vel = (double)(pos - pos_p) / T;
    pos_p = pos;

    /* Update Motor Torque Command */
    double f = 1.0 / 10.0;
    u = PWM_MAX * sign(sin(2.0 * PI * f * t));

    /* Update Control */
    if (abs(u) >= PWM_MAX)
    {
        u = PWM_MAX * sign(u);
    }

    /* Update Motor Input */
    motor_ID549XW_update(u);

    /* Print Data */
    print_data();
}


void motor_ID549XW_update(double torque)
{
    if (torque > 0.0)
    {
        digitalWrite(PIN_MOTOR_DIR, LOW);
        analogWrite(PIN_MOTOR_VOLTAGE, (int)(PWM_MAX - torque));
    }
    else if (torque < 0.0)
    {
        digitalWrite(PIN_MOTOR_DIR, HIGH);
        analogWrite(PIN_MOTOR_VOLTAGE, (int)(torque + PWM_MAX));
    }
    else
    {
        digitalWrite(PIN_MOTOR_DIR, LOW);
        analogWrite(PIN_MOTOR_VOLTAGE, (int)PWM_MAX);
    }
}

double sign(double u_in)
{
    if (u_in > 0.0)
        return 1.0;
    else if (u_in < 0.0)
        return -1.0;
    return 0.0;
}

void print_data()
{
    Serial.print(pos);
    Serial.print(" ");
    Serial.print(vel);
    Serial.print(" ");
    Serial.print(u);
    Serial.println();
}