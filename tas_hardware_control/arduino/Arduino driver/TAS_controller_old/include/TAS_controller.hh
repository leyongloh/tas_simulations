#ifndef _TASCONTROLLER_H_
#define _TASCONTROLLER_H_

#include <ros.h>
#include <geometry_msgs/Vector3.h>

/* ---------- Limits -------------------------------------------------------- */
#define MAX_STEER PI /6.0 // 8.0 //6.0 FOR BIGGER RANGE, ONLY CHANGE HERE, DOMINIK
#define MIN_RPM   -55.0*3.6 //[rpm]   MEKONG NEEDS SCALING
#define MAX_RPM   55.0*3.6  //[rpm]   MEKONG NEEDS SCALING
#define MAX_ACCEL 25.0*3.5 //[r/min²] MEKONG NEEDS SCALING

/* ---------- Car configuration --------------------------------------------- */
#define NUM_MOTORS 2
// TODO: add wheelbase and track for differntial simulation ////////////////////

/* ---------- Pin configuration --------------------------------------------- */ 
// Motors
#define RPM 120

#define M1_MOTOR_STEPS    200 //configure the pins connected
#define M1_EN_PIN    23
#define M1_DIR_PIN   25 //direction 25
#define M1_STEP_PIN  5 //step 5

#define M1_M0        26
#define M1_M1        28
#define M1_M2        13
#define M1_CFG1_PIN  26
#define M1_CFG2_PIN  28

#define M2_MOTOR_STEPS    200
#define M2_EN_PIN    22
#define M2_DIR_PIN   24
#define M2_STEP_PIN  11  // OC3A  5

// #define M2_CS_PIN    27
// #define M2_DIAG1_PIN 29
#define M2_M0        27
#define M2_M1        29
#define M2_CFG1_PIN  27
#define M2_CFG2_PIN  29


// Steering servo
#define STEERING_TX 17 //18
#define STEERING_RX 16 //19

// Remote control
#define RC_CH1 19 // steering
#define RC_CH2 20 // throttle
#define RC_CH3 21 // control mode switch

// #define RC_CH1 48 // steering
// #define RC_CH2 50 // throttle
// #define RC_CH3 52 // control mode switch

/* ---------- Stepper driver specific --------------------------------------- */
// Enable levels (Pullup'ed, therefore reverse of the datasheet)
#define ENABLE  HIGH
#define DISABLE LOW

#define FLOATING -1

// Microsteps (per step)
#define RESOLUTION 32 //32 µsteps per step, as solderd on the small board

#define STALL_VALUE 0

/* ---------- Stepper specific ---------------------------------------------- */
#define STEPSPR 200 //*3.5  // steps per revolution //before 200
const uint16_t frq_scale_ = STEPSPR * RESOLUTION; // adjusted steps per rev
const float car_diameter = 0.140;
const float dis_betw_wheels = 0.38;
const float car_chasis = 0.360;

/* ---------- Steering servo specific --------------------------------------- */
#define SV_ID  253 // servo ID (default is 253, brute force at 254)
#define SV_MID 508 // 532 // MID OF SERVO, CAN BE CHANGED HERE AND ONLY HERE, DOMINIK
#define SV_RES 0.325 // [deg/PWM]
const float SV_RES_RAD = SV_RES * DEG_TO_RAD; // [rad/PWM]

/* ---------- Ramp timer specific (timer 2) --------------------------------- */
#define T2_COMPARE_MATCH 25   //before 255
const float T2_DT = 15625.0 / float(T2_COMPARE_MATCH); // 16000000/1024 = 15625

/* ---------- Remote control specific --------------------------------------- */
#define RC_IDLE 50000

#define CH1INT (PIND & (1<<PD2))
#define CH2INT (PIND & (1<<PD1))
#define CH3INT (PIND & (1<<PD0))

// Steering
#define RC_CH1_MID      1500    // VALUES SENT FROM REMOTE CONTROLLER, NEVER CHANGE, DOMINIK
#define RC_CH1_DEAD     20      // VALUES SENT FROM REMOTE CONTROLLER, NEVER CHANGE, DOMINIK
#define RC_CH1_RANGE_UP 505	//500     // VALUES SENT FROM REMOTE CONTROLLER, NEVER CHANGE, DOMINIK
#define RC_CH1_RANGE_DN 505	//500     // VALUES SENT FROM REMOTE CONTROLLER, NEVER CHANGE, DOMINIK

// Throttle
#define RC_CH2_MID      1465
#define RC_CH2_DEAD     30	
#define RC_CH2_MAX      2100	//2000
#define RC_CH2_MIN      700     //800
#define RC_CH2_RANGE_UP 500
#define RC_CH2_RANGE_DN 575
#define RC_CH2_THR      1800

// Control mode
#define RC_CH3_THR      1400
#define RC_CH3_MAX      2050
#define RC_CH3_RANGE    500
#define RC_CH3_MAXALERT 1850
#define RC_CH3_MINALERT 900

// These are inteded to be used with ch3 as a reference for ch1 and ch2 to
// cirumvent PWM baseline fluctuations with varying rc battery levels.
#define RC_CH3C12_LOWMID  485
#define RC_CH3C12_HIGHMID 500

/* ---------- Additional definitions (readability) -------------------------- */
// Control modes
#define INACTIVE   0
#define MANUAL     1
#define AUTONOMOUS 2

// Movement direction
#define FORWARD   1
#define BACKWARD -1

// Run modes
#define STOP  0
#define RUN   1
#define ACCEL 2
#define DECEL 3

// Stepper controller modes
#define SPREADCYCLE 0
#define STEALTHCHOP 1

/* ---------- Type definitions ---------------------------------------------- */
struct stepperInfo {
  // status variables
  volatile uint_fast8_t mode = STOP;
  volatile bool movement_requested = false;

  // per movement variables (only changed once per movement)
  volatile int_fast8_t dir_tgt = FORWARD;
  float v_tgt;
  volatile int_fast16_t f_tgt;

  // per iteration variables (potentially changed every interrupt)
  volatile int_fast8_t dir = FORWARD;
  volatile float v;
  volatile int_fast16_t f;
};

/* ---------- Derived limits ------------------------------------------------ */
const int SV_LIM = round(MAX_STEER * RAD_TO_DEG / SV_RES);

const uint_fast16_t MIN_STP_FRQ = round((abs(MIN_RPM)/60.0) * float(frq_scale_));
const uint_fast16_t MAX_STP_FRQ = round((MAX_RPM/60.0) * float(frq_scale_));
/* 60.0*/
const uint_fast16_t deltaF_ = round(float(frq_scale_) * (MAX_ACCEL/60.0/T2_DT)); // df / 1 s
//const uint_fast16_t deltaF_ = 100;
const uint_fast16_t RC_CH1_MAX = RC_CH1_MID + RC_CH1_RANGE_UP + RC_CH1_DEAD;
const uint_fast16_t RC_CH1_MIN = RC_CH1_MID - RC_CH1_RANGE_DN - RC_CH1_DEAD;


/* ---------- Global variables ---------------------------------------------- */
// Pin iterators
const char dir_pin[NUM_MOTORS] = {M1_DIR_PIN, M2_DIR_PIN};
const char en_pin[NUM_MOTORS] = {M1_EN_PIN, M2_EN_PIN};
const char cfg1_pin[NUM_MOTORS] = {M1_CFG1_PIN, M2_CFG1_PIN};
const char cfg2_pin[NUM_MOTORS] = {M1_CFG2_PIN, M2_CFG2_PIN};

// Status flags
bool active_ = false; // motor driver
uint_fast8_t mode_ = INACTIVE; // control mode

// Helpers
volatile stepperInfo steppers[NUM_MOTORS];
const int direction_mod[NUM_MOTORS] = {-1, 1}; // direction modifiers, so the motors run in the same direction.

// ROS related
ros::NodeHandle nh;
volatile int ros_sa_; // steering angle in degree
volatile int_fast8_t ros_dir_tgt = FORWARD;
volatile uint_fast16_t ros_frq_[NUM_MOTORS];
bool ros_newCmd_ = false;

// Remote control variables
volatile unsigned long ch1_time_;
volatile unsigned long ch2_time_;
volatile unsigned long ch3_time_;

volatile unsigned long ch1_read_;
volatile unsigned long ch2_read_;
volatile unsigned long ch3_read_;

volatile bool readingCh1_ = false;
volatile bool readingCh2_ = false;
volatile bool readingCh3_ = false;

// These make sure that all input channels are read evenly and stutters due to
// corrupted PWM readings are kept at a minimum.
volatile bool skip_ = false;
volatile byte updated_ = 0;
const byte ch1_bm_ = 1;
const byte ch2_bm_ = 2;
const byte ch3_bm_ = 4;
const byte allCh = ch1_bm_|ch2_bm_|ch3_bm_;

uint_fast16_t rc_sa_; // steering angle
int_fast8_t rc_dir_tgt = FORWARD; // motor direction target
uint_fast16_t frq_virtual_; // virtual velocity (center of axles)
uint_fast16_t rc_frq_[NUM_MOTORS]; // motor frequency target

// Ramp timer variables
volatile byte stp_change_ = 0;
byte m_bm[NUM_MOTORS] = {1,2};
volatile bool ramp_timer_active_ = false;

#endif // end _TASCONTROLLER_H_
