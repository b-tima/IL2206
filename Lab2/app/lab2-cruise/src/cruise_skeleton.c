/* Cruise control skeleton for the IL 2206 embedded lab
 *
 * Maintainers:  Rodolfo Jordao (jordao@kth.se), George Ungereanu (ugeorge@kth.se)
 *
 * Description:
 *
 *   In this file you will find the "model" for the vehicle that is being simulated on top
 *   of the RTOS and also the stub for the control task that should ideally control its
 *   velocity whenever a cruise mode is activated.
 *
 *   The missing functions and implementations in this file are left as such for
 *   the students of the IL2206 course. The goal is that they get familiriazed with
 *   the real time concepts necessary for all implemented herein and also with Sw/Hw
 *   interactions that includes HAL calls and IO interactions.
 *
 *   If the prints prove themselves too heavy for the final code, they can
 *   be exchanged for alt_printf where hexadecimals are supported and also
 *   quite readable. This modification is easily motivated and accepted by the course
 *   staff.
 */
#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
#include <stdint.h>

#define DEBUG 1

#define HW_TIMER_PERIOD 1 /* 100ms */

#define THROTTLE_K 2
#define THROTTLE_TI 100

#define DEFAULT_THROTTLE 5

#define OVERLOAD_OK 445

/* Button Patterns */

#define GAS_PEDAL_FLAG 0x08
#define BRAKE_PEDAL_FLAG 0x04
#define CRUISE_CONTROL_FLAG 0x02

#define SW_9 1 << 9
#define SW_8 1 << 8
#define SW_7 1 << 7
#define SW_6 1 << 6
#define SW_5 1 << 5
#define SW_4 1 << 4

/* Switch Patterns */

#define TOP_GEAR_FLAG 0x00000002
#define ENGINE_FLAG 0x00000001

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear

#define LED_RED_17 1 << 17
#define LED_RED_16 1 << 16
#define LED_RED_15 1 << 15
#define LED_RED_14 1 << 14
#define LED_RED_13 1 << 13
#define LED_RED_12 1 << 12

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0004 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE];
OS_STK ControlTask_Stack[TASK_STACKSIZE];
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIOTask_Stack[TASK_STACKSIZE];
OS_STK SwitchIOTask_Stack[TASK_STACKSIZE];
OS_STK WatchdogTask_Stack[TASK_STACKSIZE];
OS_STK OverloadDetectionTask_Stack[TASK_STACKSIZE];
OS_STK OverloadMakerTask_Stack[TASK_STACKSIZE];

// Task Priorities

#define WATCHDOG_PRIO 1
#define OVERLOAD_MAKER_PRIO 2
#define STARTTASK_PRIO 5
#define VEHICLETASK_PRIO 10
#define CONTROLTASK_PRIO 12
#define SWITCH_IO_TASK_PRIO 13
#define BUTTON_IO_TASK_PRIO 14
#define OVERLOAD_DETECTION_PRIO 15

// Task Periods

#define CONTROL_PERIOD 300
#define VEHICLE_PERIOD 300

#define SWITCH_IO_POLL_PERIOD 10
#define BUTTON_IO_POLL_PERIOD 10
#define OVERLOAD_DETECTION_PERIOD 10
#define WATCHDOG_PERIOD 300
#define OVERLOAD_MAKER_PERIOD 300

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;
OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_Engine;

OS_EVENT *Mbox_Cruise;
OS_EVENT *Mbox_TopGear;
OS_EVENT *Mbox_Gas;

// Semaphores
OS_EVENT *sem_vehicle;
OS_EVENT *sem_control;
OS_EVENT *sem_button;
OS_EVENT *sem_switch;
OS_EVENT *sem_watchdog;
OS_EVENT *sem_overload;
OS_EVENT *sem_overload_maker;

OS_EVENT *sem_overload_ok;

// SW-Timer
OS_TMR *timer_vehicle;
OS_TMR *timer_control;
OS_TMR *timer_button;
OS_TMR *timer_switch;
OS_TMR *timer_overload;
OS_TMR *timer_watchdog;
OS_TMR *timer_overloadmaker;

/*
 * Types
 */
enum active
{
  on = 2,
  off = 1
};

/*
 * Global variables
 */
int delay;            // Delay of HW-timer
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs
int red_leds = 0;
int green_leds = 0;

int overload_sleep = 0;

void vehicle_callback(void *ptmr, void *arg)
{
  OSSemPost(sem_vehicle);
}

void control_callback(void *ptmr, void *arg)
{
  OSSemPost(sem_control);
}

void button_callback(void *ptmr, void *arg)
{
  OSSemPost(sem_button);
}

void switch_callback(void *ptmr, void *arg)
{
  OSSemPost(sem_switch);
}

void watchdog_callback(void *ptmr, void *arg)
{
  OSSemPost(sem_watchdog);
}

void overload_callback(void *ptmr, void *arg)
{
  OSSemPost(sem_overload);
}

void overloadmaker_callback(void *ptmr, void *arg){
  OSSemPost(sem_overload_maker);
}

/*
 * Helper functions
 */

int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void *context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */

  return delay;
}

static int b2sLUT[] = {
    0x40, //0
    0x79, //1
    0x24, //2
    0x30, //3
    0x19, //4
    0x12, //5
    0x02, //6
    0x78, //7
    0x00, //8
    0x18, //9
    0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval)
{
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity)
{
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if (velocity < 0)
  {
    out_sign = int2seven(10);
    tmp *= -1;
  }
  else
  {
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp / 10) * 10);

  out = int2seven(0) << 21 |
        out_sign << 14 |
        out_high << 7 |
        out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE, out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
  int tmp = target_vel;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if (target_vel < 0)
  {
    out_sign = int2seven(10);
    tmp *= -1;
  }
  else
  {
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp / 10) * 10);

  out = int2seven(0) << 21 |
        out_sign << 14 |
        out_high << 7 |
        out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE, out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
  if (position >= 0)
  {
    red_leds |= LED_RED_17;
  }
  if (position >= 400)
  {
    red_leds |= LED_RED_16;
  }
  if (position >= 800)
  {
    red_leds |= LED_RED_15;
  }
  if (position >= 1200)
  {
    red_leds |= LED_RED_14;
  }
  if (position >= 1600)
  {
    red_leds |= LED_RED_13;
  }
  if (position >= 2000)
  {
    red_leds |= LED_RED_12;
  }
}

/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 * 
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 */
void VehicleTask(void *pdata)
{
  uint8_t perr;
  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  INT8U err;
  void *msg;
  INT8U *throttle;
  INT16S acceleration;
  INT16U position = 0;
  INT16S velocity = 0;
  enum active brake_pedal = off;
  enum active engine = off;

  printf("Vehicle task created!\n");

  while (1)
  {
    err = OSMboxPost(Mbox_Velocity, (void *)&velocity);

    OSSemPend(sem_vehicle, 0, &perr);

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */
    msg = OSMboxPend(Mbox_Throttle, 1, &err);
    if (err == OS_NO_ERR)
      throttle = (INT8U *)msg;
    /* Same for the brake signal that bypass the control law */
    msg = OSMboxPend(Mbox_Brake, 1, &err);
    if (err == OS_NO_ERR)
      brake_pedal = *((enum active *)msg);
    /* Same for the engine signal that bypass the control law */
    msg = OSMboxPend(Mbox_Engine, 1, &err);
    if (err == OS_NO_ERR)
      engine = *((enum active *)msg);

    // vehichle cannot effort more than 80 units of throttle
    if (*throttle > 80)
      *throttle = 80;

    // brakes + wind
    if (brake_pedal == off)
    {
      // wind resistance
      acceleration = -wind_factor * velocity;
      // actuate with engines
      if (engine == on)
        acceleration += (*throttle);

      // gravity effects
      if (400 <= position && position < 800)
        acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)
        acceleration -= 2 * gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)
        acceleration += 2 * gravity_factor; //traveling downhill
      else if (2000 <= position)
        acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else
      acceleration = -brake_factor * velocity;

    printf("Position: %d m\n", position);
    printf("Velocity: %d m/s\n", velocity);
    printf("Accell: %d m/s2\n", acceleration);
    printf("Throttle: %d V\n", *throttle);

    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity + acceleration * VEHICLE_PERIOD / 1000.0;
    // reset the position to the beginning of the track
    if (position > 2400)
      position = 0;

    show_velocity_on_sevenseg((INT8S)velocity);
    show_position(position);
  }
}

/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void *pdata)
{
  INT8U err;
  INT8U throttle; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void *msg;
  INT16S *current_velocity;
  INT16S target_velocity = -1;
  INT16S last_error = 0;
  uint8_t perr;

  enum active gas_pedal = off;
  enum active top_gear = off;
  enum active cruise_control = off;

  printf("Control Task created!\n");

  while (1)
  {
    msg = OSMboxPend(Mbox_Velocity, 0, &err);
    current_velocity = (INT16S *)msg;

    msg = OSMboxPend(Mbox_Gas, 0, &err);
    gas_pedal = *((enum active *)msg);

    msg = OSMboxPend(Mbox_Cruise, 0, &err);
    cruise_control = *((enum active *)msg);

    msg = OSMboxPend(Mbox_TopGear, 0, &err);
    top_gear = *((enum active *)msg);

    if (cruise_control == on && target_velocity == -1 && *current_velocity > 20)
    {
      target_velocity = *current_velocity;
    }

    green_leds &= ~LED_GREEN_0;

    if (top_gear == on && *current_velocity > 20)
    {
      if (cruise_control == on && target_velocity == -1)
      {
        target_velocity = *current_velocity;
      }
      else if (cruise_control == off)
      {
        target_velocity = -1;
      }

      if (cruise_control == on)
      {

        throttle = throttle + THROTTLE_K * (1 + CONTROL_PERIOD / (2 * THROTTLE_TI)) * (target_velocity - *current_velocity) + (CONTROL_PERIOD / (2 * THROTTLE_TI) - 1) * last_error;

        last_error = target_velocity - *current_velocity;

        show_target_velocity(target_velocity);
        green_leds |= LED_GREEN_0;
      }
      else
      {
        last_error = 0;
        show_target_velocity(0);
        if (gas_pedal == on)
        {
          throttle += DEFAULT_THROTTLE;
        }
        else
        {
          throttle = throttle > 0 ? throttle - DEFAULT_THROTTLE : 0;
        }
      }
    }
    else
    {
      last_error = 0;
      show_target_velocity(0);
      if (gas_pedal == on)
      {
        throttle += DEFAULT_THROTTLE;
      }
      else
      {
        throttle = throttle > 0 ? throttle - DEFAULT_THROTTLE : 0;
      }
    }

    //IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, green_leds);
    err = OSMboxPost(Mbox_Throttle, (void *)&throttle);

    OSSemPend(sem_control, 0, &perr);
  }
}

void SwitchIOTask(void *pdata)
{
  uint8_t perr;
  int switch_io;
  enum active top_gear;
  enum active engine;
  while (1)
  {
    switch_io = switches_pressed();

    if (switch_io & ENGINE_FLAG)
    {
      red_leds |= LED_RED_0;
      engine = on;
    }
    else
    {
      red_leds &= ~LED_RED_0;
      engine = off;
    }

    if (switch_io & TOP_GEAR_FLAG)
    {
      red_leds |= LED_RED_1;
      top_gear = on;
    }
    else
    {
      red_leds &= ~LED_RED_1;
      top_gear = off;
    }

    // overload maker switches
    int by64 = 0;
    if (switch_io & SW_4)
    {
      by64 += 1;
    }
    if (switch_io & SW_5)
    {
      by64 += 2;
    }
    if (switch_io & SW_6)
    {
      by64 += 4;
    }
    if (switch_io & SW_7)
    {
      by64 += 8;
    }
    if (switch_io & SW_8)
    {
      by64 += 16;
    }
    if (switch_io & SW_9)
    {
      by64 += 32;
    }

    float percentage = 300 * (((float)by64) / ((float)64));
    overload_sleep = (int)percentage;

    OSMboxPost(Mbox_Engine, &engine);
    OSMboxPost(Mbox_TopGear, &top_gear);
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, red_leds);

    OSSemPend(sem_switch, 0, &perr);
  }
}

void ButtonIOTask(void *pdata)
{
  uint8_t perr;
  int buttons;

  enum active cruise;
  enum active brake;
  enum active gas;

  while (1)
  {

    buttons = buttons_pressed();

    if (buttons & CRUISE_CONTROL_FLAG)
    {
      green_leds |= LED_GREEN_2;
      cruise = on;
    }
    else
    {
      green_leds &= ~LED_GREEN_2;
      cruise = off;
    }

    if ((buttons & BRAKE_PEDAL_FLAG) && cruise == off)
    {
      green_leds |= LED_GREEN_4;
      brake = on;
    }
    else
    {
      green_leds &= ~LED_GREEN_4;
      brake = off;
    }

    if ((buttons & GAS_PEDAL_FLAG) && cruise == off)
    {
      green_leds |= LED_GREEN_6;
      gas = on;
    }
    else
    {
      green_leds &= ~LED_GREEN_6;
      gas = off;
    }

    OSMboxPost(Mbox_Gas, &gas);
    OSMboxPost(Mbox_Brake, &brake);
    OSMboxPost(Mbox_Cruise, &cruise);
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, green_leds);

    OSSemPend(sem_button, 0, &perr);
  }
}

void WatchdogTask(void *pdata)
{
  uint8_t perr;
  int *ok_signal;
  while (1)
  {
    OSSemPend(sem_watchdog, 0, &perr);

    OSSemPend(sem_overload_ok, 1, &perr);

    if (perr == OS_ERR_TIMEOUT)
    {
      printf("Overload detected!\n");
    }
  }
}

void OverloadDetectionTask(void *pdata)
{
  int ok_signal = OVERLOAD_OK;
  OS_SEM_DATA sem_data;
  uint8_t perr;

  while (1)
  {
    OSSemQuery(sem_overload_ok, &sem_data);

    if (sem_data.OSCnt == 0)
    {
      OSSemPost(sem_overload_ok);
    }

    OSSemPend(sem_overload, 0, &perr);
  }
}

void OverloadMaker(void *pdata)
{
  uint8_t perr;
  while(1){

    OSTimeSet(0);

    while(OSTimeGet() <= overload_sleep){}

    OSSemPend(sem_overload_maker, 0, &perr);
  }
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */

void StartTask(void *pdata)
{
  INT8U err;
  void *context;

  static alt_alarm alarm; /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000;
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start(&alarm,
                      delay,
                      alarm_handler,
                      context) < 0)
  {
    printf("No system clock available!n");
  }

  /* 
   * Create and start Software Timer 
   */

  /*
   * Creation of Kernel Objects
   */

  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void *)0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void *)0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void *)1);    /* Empty Mailbox - Velocity */
  Mbox_Engine = OSMboxCreate((void *)1);   /* Empty Mailbox - Engine */

  Mbox_Gas = OSMboxCreate((void *)1);
  Mbox_Cruise = OSMboxCreate((void *)1);
  Mbox_TopGear = OSMboxCreate((void *)0);

  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */

  err = OSTaskCreateExt(
      ControlTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ControlTask_Stack[TASK_STACKSIZE - 1], // Pointer to top
      // of task stack
      CONTROLTASK_PRIO,
      CONTROLTASK_PRIO,
      (void *)&ControlTask_Stack[0],
      TASK_STACKSIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      VehicleTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &VehicleTask_Stack[TASK_STACKSIZE - 1], // Pointer to top
      // of task stack
      VEHICLETASK_PRIO,
      VEHICLETASK_PRIO,
      (void *)&VehicleTask_Stack[0],
      TASK_STACKSIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      ButtonIOTask,
      NULL,
      &ButtonIOTask_Stack[TASK_STACKSIZE - 1],
      BUTTON_IO_TASK_PRIO,
      BUTTON_IO_TASK_PRIO,
      (void *)&ButtonIOTask_Stack[0],
      TASK_STACKSIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      SwitchIOTask,
      NULL,
      &SwitchIOTask_Stack[TASK_STACKSIZE - 1],
      SWITCH_IO_TASK_PRIO,
      SWITCH_IO_TASK_PRIO,
      (void *)&SwitchIOTask_Stack[0],
      TASK_STACKSIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      WatchdogTask,
      NULL,
      &WatchdogTask_Stack[TASK_STACKSIZE - 1],
      WATCHDOG_PRIO,
      WATCHDOG_PRIO,
      (void *)&WatchdogTask_Stack[0],
      TASK_STACKSIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      OverloadDetectionTask,
      NULL,
      &OverloadDetectionTask_Stack[TASK_STACKSIZE - 1],
      OVERLOAD_DETECTION_PRIO,
      OVERLOAD_DETECTION_PRIO,
      (void *)&OverloadDetectionTask_Stack[0],
      TASK_STACKSIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      OverloadMaker,
      NULL,
      &OverloadMakerTask_Stack[TASK_STACKSIZE - 1],
      OVERLOAD_MAKER_PRIO,
      OVERLOAD_MAKER_PRIO,
      (void *)&OverloadMakerTask_Stack[0],
      TASK_STACKSIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK);

  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void)
{

  printf("Lab: Cruise Control\n");

  OSTaskCreateExt(
      StartTask, // Pointer to task code
      NULL,      // Pointer to argument that is
      // passed to task
      (void *)&StartTask_Stack[TASK_STACKSIZE - 1], // Pointer to top
      // of task stack
      STARTTASK_PRIO,
      STARTTASK_PRIO,
      (void *)&StartTask_Stack[0],
      TASK_STACKSIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  uint8_t perr;
  char name1[] = "Vehicle_timer";
  char name2[] = "Control_timer";

  sem_vehicle = OSSemCreate(0);
  sem_control = OSSemCreate(0);
  sem_switch = OSSemCreate(0);
  sem_button = OSSemCreate(0);
  sem_watchdog = OSSemCreate(0);
  sem_overload = OSSemCreate(0);
  sem_overload_ok = OSSemCreate(0);
  sem_overload_maker = OSSemCreate(0);

  timer_vehicle = OSTmrCreate(0, VEHICLE_PERIOD, OS_TMR_OPT_PERIODIC, vehicle_callback, NULL, name1, &perr);
  timer_control = OSTmrCreate(0, CONTROL_PERIOD, OS_TMR_OPT_PERIODIC, control_callback, NULL, name2, &perr);
  timer_button = OSTmrCreate(0, BUTTON_IO_POLL_PERIOD, OS_TMR_OPT_PERIODIC, button_callback, NULL, name2, &perr);
  timer_switch = OSTmrCreate(0, SWITCH_IO_POLL_PERIOD, OS_TMR_OPT_PERIODIC, switch_callback, NULL, name2, &perr);
  timer_watchdog = OSTmrCreate(0, WATCHDOG_PERIOD, OS_TMR_OPT_PERIODIC, watchdog_callback, NULL, name2, &perr);
  timer_overload = OSTmrCreate(0, OVERLOAD_DETECTION_PERIOD, OS_TMR_OPT_PERIODIC, overload_callback, NULL, name2, &perr);
  timer_overloadmaker = OSTmrCreate(0, OVERLOAD_MAKER_PERIOD, OS_TMR_OPT_PERIODIC, overloadmaker_callback, NULL, name2, &perr);

  OSTmrStart(timer_vehicle, &perr);
  OSTmrStart(timer_control, &perr);
  OSTmrStart(timer_button, &perr);
  OSTmrStart(timer_switch, &perr);
  OSTmrStart(timer_watchdog, &perr);
  OSTmrStart(timer_overload, &perr);
  OSTmrStart(timer_overloadmaker, &perr);

  OSStart();

  return 0;
}
