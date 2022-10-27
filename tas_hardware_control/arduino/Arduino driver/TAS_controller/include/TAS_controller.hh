#ifndef _TASCONTROLLER_H_
#define _TASCONTROLLER_H_

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include "FastAccelStepper.h"

/* ---------- Limits -------------------------------------------------------- */
#define MAX_STEER PI /6.0 // 8.0 //6.0 FOR BIGGER RANGE, ONLY CHANGE HERE

// max rpm is set here
// factor from step frequency to rpm is 0.3
// 455 steps per second equate to ~ 1 m/s 
// max tested value is 800 -> make sure to have long enough way for braking
// 200 steps per second is tested value for current autonomous implementation
const uint_fast16_t MAX_STP_FRQ = 200;

/* ---------- Car configuration --------------------------------------------- */
#define NUM_MOTORS 2
/* ---------- Pin configuration --------------------------------------------- */ 

// Steering servo
#define STEERING_TX 17 //18
#define STEERING_RX 16 //19

// Remote control
#define RC_CH1 19 // steering
#define RC_CH2 20 // throttle
#define RC_CH3 21 // control mode switch


/* ---------- Car dimensions to calculate for Ackermann steering ------------ */
const float dis_betw_wheels = 0.38;
const float car_chasis = 0.360;

/* ---------- Steering servo specific --------------------------------------- */
#define SV_ID  253 // servo ID (default is 253, brute force at 254)
#define SV_MID 508 // 532 // MID OF SERVO, CAN BE CHANGED HERE AND ONLY HERE, DOMINIK
#define SV_RES 0.325 // [deg/PWM]
const float SV_RES_RAD = SV_RES * DEG_TO_RAD; // [rad/PWM]5

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

/* ---------- Derived limits ------------------------------------------------ */
const int SV_LIM = round(MAX_STEER * RAD_TO_DEG / SV_RES);


// values for steering
const uint_fast16_t RC_CH1_MAX = RC_CH1_MID + RC_CH1_RANGE_UP + RC_CH1_DEAD;
const uint_fast16_t RC_CH1_MIN = RC_CH1_MID - RC_CH1_RANGE_DN - RC_CH1_DEAD;


/* ---------- Global variables ---------------------------------------------- */

// Status flags
uint_fast8_t mode_ = INACTIVE; // control mode

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

#endif // end _TASCONTROLLER_H_
