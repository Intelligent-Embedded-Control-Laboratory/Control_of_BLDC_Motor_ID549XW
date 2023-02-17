#include <Arduino.h>
#include <LS7366R.h>
#include <moving_average_filter.h>
#include <first_order_iir_filters.h>

/* Pin Definition */
const uint8_t PIN_MOTOR_EN = 23;      // 馬達的啟動腳位 (HIGH=Enable, LOW=Disable)，給 Low 會直接停轉 (TODO:)
const uint8_t PIN_MOTOR_DIR = 21;     // 馬達的正反轉腳位 (HIGH=其中一轉，LOW=另一轉)
const uint8_t PIN_MOTOR_VOLTAGE = 22; // 馬達的電壓腳位，要注意的是，命令映射是相反。也就是說，PWM 值給滿，其實是最低的電壓輸出給馬達，PWM 值給最低，是最高的電壓輸出給馬達。
const uint8_t PIN_QEI = 10; // 解馬達 Encoder 的晶片腳位
const uint8_t PIN_SERVO_BREAKER = 9; // 煞車硬體的腳位

const int PWM_FREQ = 20000;                    // 對於這顆馬達 (ID549XW) 是一個固定參數
const int PWM_RES = 12;                        // 對於這顆馬達 (ID549XW) 是一個固定參數
const int PWM_MAX = (int)(pow(2.0, PWM_RES) - 1); // PWM 命令的最大值
const int CONFIG_ISR_PERIOD_MICROS = 1000.0;            // 內部中斷的執行時間間隔

/* Interrupt Service Routine (ISR) */
IntervalTimer isr_control_timer;
double T = CONFIG_ISR_PERIOD_MICROS / 1000000.0;
int idx = 0;
double t = 0.0;

/* Encoder Sensor of Motor */
LS7366R encoder;
long int pos = 0;   // Position of motor
long int pos_p = 0; // Previous step position of motor
// long pos_d = 0;
double pos_df = 0.0;
double pos_d = 0.0;
double vel = 0.0;     // Velocity of motor
double vel_f = 0.0;

/* Velocity Filter  */
MovingAverageFilter filter;
FirstOrderIIR command_filter;

/* Control */
double u = 0.0;     // Control
double e = 0.0;     // Tracking error of fused and desired attitude
double e_dot = 0.0; //
double kp = 0.0;
double kd = 0.0;
double ki = 0.0;
double x = 0.0;

/* Function Declaration */
void isr_control();
void motor_ID549XW_update(double torque);
double sign(double u_in);

void print_data()
{
    Serial.print(pos);
    Serial.print(" ");
    Serial.print(pos_df);
    Serial.print(" ");
    Serial.print(vel);
    Serial.print(" ");
    Serial.print(vel_f);
    Serial.print(" ");
    Serial.print(u);
    Serial.println();
}

/* Main */
void setup()
{
    Serial.begin(115200);

    pinMode(PIN_MOTOR_EN, OUTPUT);
    pinMode(PIN_MOTOR_VOLTAGE, OUTPUT);
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    analogWriteFrequency(PIN_MOTOR_VOLTAGE, PWM_FREQ);
    analogWriteResolution(PWM_RES);

    digitalWrite(PIN_MOTOR_EN, HIGH);
    digitalWrite(PIN_MOTOR_DIR, LOW);
    analogWrite(PIN_MOTOR_VOLTAGE, (int)PWM_MAX);

    encoder.begin(&PIN_QEI, 1);
    filter.init(20);
    command_filter.init(FirstOrderIIR::TYPE_LPF, 1, T);

    isr_control_timer.begin(isr_control, CONFIG_ISR_PERIOD_MICROS);
    isr_control_timer.priority(255);
}

void loop()
{
}

void isr_control()
{
    idx++;
    t = (double)(idx)*T;

    /* Update Motor Position Command */
    double f = 1.0 / 5.0;
    pos_d = 400 * sign(sin(2.0 * PI * f * t));
    command_filter.update(&pos_d, &pos_df);

    /* Update Motor Position */
    encoder.read();
    pos = encoder.get_pulse()[0];
    vel = (double)(pos - pos_p) / T;
    pos_p = pos;
    vel_f = filter.update(vel); // Filtered velocity

    /* Update Control */
    double e_pos = pos - pos_df;
    double kp = 20.0;
    double kd = 0.5;
    u = -kp * e_pos - kd * vel_f;
    if (abs(u) >= PWM_MAX)
    {
        u = PWM_MAX * sign(u);
    }

    /* Update Motor Input */
    motor_ID549XW_update(u);


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
