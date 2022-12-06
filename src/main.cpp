#include <Arduino.h>
#include <stdio.h>
#include "driver/ledc.h"
#include "driver/pcnt.h"

#define SPEED_MODE LEDC_HIGH_SPEED_MODE

#define RIGHT_TIMER LEDC_TIMER_1
#define RIGHT_MOTOR_SPEED_CHANNEL LEDC_CHANNEL_1
#define RIGHT_MOTOR_SPEED_PIN 17
#define RIGHT_MOTOR_UP_PIN 23
#define RIGHT_MOTOR_DOWN_PIN 31
#define RIGHT_UPPER_LIMIT_PIN 27
#define RIGHT_LOWER_LIMIT_PIN 14
#define RIGHT_MOTION_ENCODER_PIN 22

#define LEFT_TIMER LEDC_TIMER_0
#define LEFT_MOTOR_SPEED_CHANNEL LEDC_CHANNEL_0
#define LEFT_MOTOR_SPEED_PIN 16
#define LEFT_MOTOR_UP_PIN 32
#define LEFT_MOTOR_DOWN_PIN 4
#define LEFT_UPPER_LIMIT_PIN 12
#define LEFT_LOWER_LIMIT_PIN 13
#define LEFT_MOTION_COUNTER_UNIT PCNT_UNIT_0
#define LEFT_MOTION_ENCODER_PIN 33

#define UP_BUTTON_PIN 25
#define DOWN_BUTTON_PIN 26

int BASE_FREQUENCY = 1000; // 1khz of base speed of 1000khz max
int LEFT_SPEED = 0;        // 0 <-> MIN_SLOWDOWN_SPEED00 representing a percentage of motors peed
int RIGHT_SPEED = 0;       // 0 <-> MIN_SLOWDOWN_SPEED00 representing a percentage of motors peed
int16_t LEFT_COUNT = 0;        // Left post revolution counts
int16_t RIGHT_COUNT = 0;       // Right post revolution counts
int ACCEPTABLE_SKEW = 3;
int MIN_SLOWDOWN_SPEED = 10;
int COUNTER_LIMIT = 10000;
int LEFT_OVERFLOW_COUNTER = 0; // pulse counter overflow counter
uint16_t COUNTER_FILTER_VAL=  1000;                             // filter (damping, inertia) value for avoiding glitches in the count, max. 1023

pcnt_isr_handle_t user_isr_handle = NULL; // interrupt handler - not used

void initialise_motors()
{
  log_v("Setting up timers and channels");
  ledc_timer_config_t right_timer, left_timer;
  right_timer.duty_resolution = left_timer.duty_resolution = LEDC_TIMER_1_BIT; // resolution of PWM duty
  right_timer.freq_hz = left_timer.freq_hz = 100000;                           // frequency of PWM signal
  right_timer.speed_mode = left_timer.speed_mode = SPEED_MODE;                 // timer mode
  right_timer.clk_cfg = left_timer.clk_cfg = LEDC_USE_REF_TICK;                // Auto select the source clock

  right_timer.timer_num = RIGHT_TIMER;
  ledc_timer_config(&right_timer);
  left_timer.timer_num = LEFT_TIMER;
  ledc_timer_config(&left_timer);

  ledc_channel_config_t right_channel, left_channel;
  right_channel.speed_mode = left_channel.speed_mode = SPEED_MODE;
  right_channel.intr_type = left_channel.intr_type = LEDC_INTR_DISABLE;
  right_channel.duty = left_channel.duty = 1;
  right_channel.hpoint = left_channel.hpoint = 0;

  right_channel.channel = RIGHT_MOTOR_SPEED_CHANNEL;
  right_channel.gpio_num = RIGHT_MOTOR_SPEED_PIN;
  right_channel.timer_sel = RIGHT_TIMER;
  ledc_channel_config(&right_channel);

  left_channel.channel = LEFT_MOTOR_SPEED_CHANNEL;
  left_channel.gpio_num = LEFT_MOTOR_SPEED_PIN;
  left_channel.timer_sel = LEFT_TIMER;
  ledc_channel_config(&left_channel);
}
void IRAM_ATTR CounterOverflow(void *arg)
{                                                   // Interrupt for overflow of pulse counter
  LEFT_OVERFLOW_COUNTER = LEFT_OVERFLOW_COUNTER + 1;            // increase overflow counter
  //PCNT.int_clr.val = BIT(LEFT_MOTION_COUNTER_UNIT); // clean overflow flag
  pcnt_counter_clear(LEFT_MOTION_COUNTER_UNIT);     // zero and reset of pulse counter unit
}

void initialise_pulse_counters()
{                                                              // initialise pulse counter
  pcnt_config_t left_pulse_counter = {};                       // Instance of pulse counter
  left_pulse_counter.pulse_gpio_num = LEFT_MOTION_ENCODER_PIN; // pin assignment for pulse counter = GPIO 15
  left_pulse_counter.pos_mode = PCNT_COUNT_INC;                // count rising edges (=change from low to high logical level) as pulses
  left_pulse_counter.counter_h_lim = COUNTER_LIMIT;            // set upper limit of counting
  left_pulse_counter.unit = LEFT_MOTION_COUNTER_UNIT;          // select ESP32 pulse counter unit 0
  left_pulse_counter.channel = PCNT_CHANNEL_0;                 // select channel 0 of pulse counter unit 0
  pcnt_unit_config(&left_pulse_counter);                       // configur rigisters of the pulse counter

  pcnt_counter_pause(LEFT_MOTION_COUNTER_UNIT); // pause puls counter unit
  pcnt_counter_clear(LEFT_MOTION_COUNTER_UNIT); // zero and reset of pulse counter unit
  pcnt_event_enable(LEFT_MOTION_COUNTER_UNIT, PCNT_EVT_H_LIM);   // enable event for interrupt on reaching upper limit of counting
  pcnt_isr_register(CounterOverflow, NULL, 0, &user_isr_handle); // configure register overflow interrupt handler
  pcnt_intr_enable(LEFT_MOTION_COUNTER_UNIT);                    // enable overflow interrupt
  pcnt_set_filter_value(LEFT_MOTION_COUNTER_UNIT, COUNTER_FILTER_VAL); // set damping, inertia
  pcnt_filter_enable(LEFT_MOTION_COUNTER_UNIT);                     // enable counter glitch filter (damping)
  pcnt_counter_resume(LEFT_MOTION_COUNTER_UNIT); // resume counting on pulse counter unit
}


void read_counters() {                                           // function for reading pulse counter (for timer)
  pcnt_get_counter_value(LEFT_MOTION_COUNTER_UNIT, &LEFT_COUNT);     // get pulse counter value - maximum value is 16 bits
}

void setup()
{

  pinMode(LEFT_MOTION_ENCODER_PIN, INPUT_PULLDOWN);
  pinMode(RIGHT_MOTION_ENCODER_PIN, INPUT_PULLDOWN);
  initialise_motors();
  initialise_pulse_counters();

  Serial.begin(115200);
  Serial.setDebugOutput(true);

  pinMode(UP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_LOWER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_UPPER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LEFT_LOWER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LEFT_UPPER_LIMIT_PIN, INPUT_PULLUP);

  digitalWrite(LEFT_MOTOR_UP_PIN, LOW);
  digitalWrite(LEFT_MOTOR_DOWN_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_UP_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_DOWN_PIN, LOW);
}
void set_left_motor_speed(int speed)
{
  log_i("Setting left speed to %i", speed);
  if (speed > 100)
  {
    speed = 100;
  }
  if (speed > 0)
  {
    ledc_timer_resume(SPEED_MODE, LEFT_TIMER);
    ledc_set_freq(SPEED_MODE, LEFT_TIMER, (speed * BASE_FREQUENCY));
    // ledcWriteTone(LEFT_MOTOR_SPEED_CHANNEL, (speed * BASE_FREQUENCY));
  }
  else
  {
    ledc_timer_pause(SPEED_MODE, LEFT_TIMER);
    // ledc_stop(SPEED_MODE, LEFT_MOTOR_SPEED_CHANNEL, 0);
  }
}

bool get_switch_state(int button)
{
  if (!digitalRead(button))
    return true;
  return false;
}

void go_up()
{
  log_i("Going up - not implimented");
}

void go_down()
{
  log_i("Going down");
  if (get_switch_state(LEFT_LOWER_LIMIT_PIN))
  {
    log_d("Left lower limit, setting left speed to %i", LEFT_SPEED);
    LEFT_SPEED = 0;
    digitalWrite(LEFT_MOTOR_DOWN_PIN, LOW);
  }
  else
  {
    digitalWrite(LEFT_MOTOR_DOWN_PIN, HIGH);
    if (
        ((LEFT_COUNT - ACCEPTABLE_SKEW) > RIGHT_COUNT) &&
        (LEFT_SPEED >= 0))
    {
      log_i("Slowing down left motor");
      LEFT_SPEED--;
    }
    else if (LEFT_SPEED < 100)
    {
      log_i("Speeding up left motor");
      LEFT_SPEED++;
    }
  }
  set_left_motor_speed(LEFT_SPEED);
}

void loop()
{
  log_d("Speed L:%d R:%d Count L: %d R:%d", LEFT_SPEED, RIGHT_SPEED, LEFT_COUNT, RIGHT_COUNT);
  if (get_switch_state(UP_BUTTON_PIN) && get_switch_state(DOWN_BUTTON_PIN))
  {
    // Maybe some special behavior 🤔
  }
  else if (get_switch_state(UP_BUTTON_PIN))
  {
    // initialise_motors();
    go_up();
  }
  else if (get_switch_state(DOWN_BUTTON_PIN))
  {
    // initialise_motors();
    go_down();
  }
  else
  {
    set_left_motor_speed(0);
  }
  read_counters();
  // Reset SKEW with both on limit
  if ((get_switch_state(LEFT_UPPER_LIMIT_PIN) && get_switch_state(RIGHT_UPPER_LIMIT_PIN)) ||
      (get_switch_state(LEFT_UPPER_LIMIT_PIN) && get_switch_state(RIGHT_UPPER_LIMIT_PIN)))
  {
    LEFT_COUNT = RIGHT_COUNT = 0;
  }
}
void app_main(void)
{
  log_i("Starting hoist controller");
  setup();
  while (true)
  {
    loop();
  }
}