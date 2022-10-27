#include "include/TAS_controller.hh"    //definition of all pins and variables
#include <util/atomic.h>            
#include <ArduinoHardware.h>

#include <tas_msgs/Control.h>           //tas messages
#include <Herkulex.h>                   //driver for the servo motor
#include <std_msgs/Float32.h>

#include <Arduino.h>
#include "DRV8825.h"                    //driver for the stepper motors

DRV8825 m1_driver(M1_MOTOR_STEPS, M1_DIR_PIN, M1_STEP_PIN, M1_EN_PIN, M1_M0, M1_M1, M1_M2);
DRV8825 m2_driver(M2_MOTOR_STEPS, M2_DIR_PIN, M2_STEP_PIN, M2_EN_PIN, M2_M0, M2_M1, M1_M2);

int READING = 0;

int flag = -1;

int ros_servo_cmd;
int ros_motor_cmd;

int servo_output;
int motor_output;

int encoder1PinA =4;
int encoder1PinB =2;
volatile long encoder1Pos = 0;

int encoder2PinA =6;
int encoder2PinB =3;
volatile long encoder2Pos = 0;

// time calculation
long previousMillis = 0;
long interval = 1000;
long count = 0;

// setup ros message for encoder data publishing
std_msgs::Float32 enc1pub;
std_msgs::Float32 enc2pub;
std_msgs::Float32 enc3pub;

//float VelX;
//float deltaTheta;
//float ThetaVel;
ros::Publisher rospub_enc1("/encoder/VelX", &enc1pub);  //publisher for linear Vel
ros::Publisher rospub_enc2("/encoder/VelL", &enc2pub);  //publisher for deltaTheta
ros::Publisher rospub_enc3("/encoder/VelR", &enc3pub);  //publisher for angular Vel

//callback function, will be called when receiving messages from ROS
void servo_cb(const geometry_msgs::Vector3& cmd_msg)
{
  //the value is limited in 1000 - 2000    
  if ((int(cmd_msg.x) > 999) && (int(cmd_msg.x) < 2001))
  {ros_motor_cmd = int(cmd_msg.x);}
  else
  {ros_motor_cmd = 1500;}  //zero velocity

  if ((int(cmd_msg.y) > 999) && (int(cmd_msg.y) < 2001))
  {
    ros_servo_cmd = int(cmd_msg.y);
  }
  else
  {
    ros_servo_cmd = 1500; //zero steering
  }

   
  servo_output = map(ros_servo_cmd,1000,2000,-30,30);    //mapping the signal to steering angles
  ros_sa_ = map(ros_servo_cmd,1000,2000,SV_MID-SV_LIM,SV_MID+SV_LIM);     //mapping the signal to the step pulse
  
  double Curveradius;   
  double servo_in_rad;
  
  if (servo_output!= 0)
  {
    servo_in_rad = servo_output*PI/180;
    Curveradius = car_chasis/tan(servo_in_rad);
  }
  
    // Ackerman steering 
    // backward
    if(ros_motor_cmd < 1500) {
      ros_dir_tgt = BACKWARD;
      motor_output = map(ros_motor_cmd,1000,1500,MIN_STP_FRQ,0);
      if(servo_output == 0) {
           ros_frq_[0] = motor_output;
           ros_frq_[1] = motor_output;
         }
        else {        
          ros_frq_[0] = uint_fast16_t(motor_output*(1 - dis_betw_wheels/(2*Curveradius)));
          ros_frq_[1] = uint_fast16_t(motor_output*(1 + dis_betw_wheels/(2*Curveradius))); 
         }   
      }
    // forward
    else if(ros_motor_cmd > 1500) {
      ros_dir_tgt = FORWARD;
      motor_output = map(ros_motor_cmd,1500,2000,0,MAX_STP_FRQ);
      if(servo_output == 0) {
            ros_frq_[0] = motor_output;
            ros_frq_[1] = motor_output;
          }
        else {
          ros_frq_[0] = uint_fast16_t(motor_output*(1 - dis_betw_wheels/(2*Curveradius)));
          ros_frq_[1] = uint_fast16_t(motor_output*(1 + dis_betw_wheels/(2*Curveradius))); 
          }
      }
    // standstill
    else {
      motor_output = 0;
      ros_frq_[0] = motor_output;
      ros_frq_[1] = motor_output;
    }

    ros_newCmd_ = true;

}
ros::Subscriber<geometry_msgs::Vector3> sub("/servo", servo_cb);


//initial setup, run only once at the beginning
int timer0= 0;
void setup() {
 
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rospub_enc1);
  nh.advertise(rospub_enc2);
  nh.advertise(rospub_enc3);
  Serial.begin(115200); // for debugging purposes
  // Motors
  initM1();
  initM2();
  // PWM timers
  setFastPWM();   // used for PFM =)
  initRampTimer(); // timer for accel and decel
  // Status LED

  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Steering servo
  
  Herkulex.beginSerial2(115200); //open serial port 1 
  Herkulex.reboot(SV_ID); //reboot first motor
  delay(500);
  Herkulex.initialize(); //initialize motors  
  delay(200);
//  Serial.println(Herkulex.stat(SV_ID));
  // 
  initRC();

  delay(3000);
    
}

float VelX;
float deltaTheta;
float ThetaVel;
int counterForOdom;
int PrviouscounterForOdom;
float VelLeft;
float VelRight;

void loop() {
//  Serial.print(counterForOdom);
//  Serial.print(" ");
//  Serial.println(PrviouscounterForOdom);
if (counterForOdom == PrviouscounterForOdom + 1){
  
    //publish the encoder data
    enc1pub.data = VelX;
    enc2pub.data = VelLeft; 
    enc3pub.data = VelRight;
        
//                Serial.print(enc1pub.data,4);
//                Serial.print(" ");
//                Serial.print(enc2pub.data,4);
//                Serial.print(" ");
//                Serial.println();

     rospub_enc1.publish(&enc1pub);
     rospub_enc2.publish(&enc2pub);
     rospub_enc3.publish(&enc3pub);
  }
  PrviouscounterForOdom=counterForOdom;
  unsigned long ch3_read = readCh3(); // control mode 
  unsigned long ch2_read = readCh2(); // throttle
  

  bool newDir = false; // new direction flag
  bool newFrq = false; // new frequency / velocity flag

  // char str[32];
  // sprintf(str, "MODE %u | CH3 %lu | CH2 %lu", mode_, ch3_read, ch2_read);
  // nh.loginfo(str);

  /* ========== MANUAL CONTROL MODE ========================================= */
  if(ch3_read < RC_CH3_THR && ch3_read > RC_CH3_MINALERT) {
//  Serial.println("The car is in manual mode");
    /* ---------- 1. Mode switch handling ----------------------------------- */
    if(mode_ != MANUAL){
      // nh.loginfo("MANUAL mode");
      resetMotors();
      resetRCCmd();
      // TODO: indicate manual mode ////////////////////////////////////////////
      digitalWrite(LED_BUILTIN, LOW);
      // TODO: activate manual mode interrupt //////////////////////////////////
      mode_ = MANUAL;

      
    }

    /* ---------- 2. Steering ----------------------------------------------- */
    unsigned long ch1_read = readCh1(); // steering
    // left steer
    if(ch1_read < RC_CH1_MID - RC_CH1_DEAD) {
      if(ch1_read > RC_CH1_MIN) {
        rc_sa_ = map(ch1_read, RC_CH1_MIN, RC_CH1_MID - RC_CH1_DEAD, SV_MID-SV_LIM, SV_MID);
        //Serial.print("LEFT VALID CH1: ");
        //Serial.println(ch1_read);
        //Serial.println(rc_sa_);


      } 
      else if(ch1_read <= RC_CH1_MIN) {
        // INVALID CH1 VALUE ERROR HANDLING: TOO LOW
        // rc_error();
        //Serial.print("Steering value below minimum: ");
        //Serial.println(ch1_read);
        //Serial.println(rc_sa_);
        rc_sa_ = SV_MID - SV_LIM;
      }
      else {
        Serial.print("INVALID LEFT STEER");
      }
    }
    // right steer
    else if(ch1_read > RC_CH1_MID + RC_CH1_DEAD) {
      if(ch1_read < RC_CH1_MAX) {
        rc_sa_ = map(ch1_read, RC_CH1_MID + RC_CH1_DEAD, RC_CH1_MAX, SV_MID, SV_MID+SV_LIM);
        //Serial.print("RIGHT VALID CH1: ");
        //Serial.println(ch1_read);
        //Serial.println(rc_sa_);

      }
      else if(ch1_read >= RC_CH1_MAX){
        // INVALID CH1 VALUE ERROR HANDLING: TOO HIGH
        // rc_error();
        //Serial.print("Steering value above maximum: ");
        //Serial.println(ch1_read);
        //Serial.println(rc_sa_);
        rc_sa_ = SV_MID + SV_LIM;

      }
      else {
        Serial.print("INVALID RIGHT STEER");
      }
    }
    // center steer
    else {
      rc_sa_ = SV_MID;
      //Serial.print("MID STEERING: ");
      //Serial.println(ch1_read);
      //Serial.println(rc_sa_);
    }
    
    double steering_angle;
    int p = rc_sa_ - SV_MID; // Formerly rc_sa_ - 512
    steering_angle = double(p)* 0.325*PI/180;
    double wheelbase_ = car_chasis; // 0.4
    double wheel_separation_ = dis_betw_wheels; // 0.35
    double radiusRef;
    int steerDirection = int(steering_angle>0.0) - int(steering_angle<0.0);

    /* ---------- 3. Throttle ----------------------------------------------- */
    // forward
    if(ch2_read < RC_CH2_MID - RC_CH2_DEAD) {
        rc_dir_tgt = FORWARD;
        if(ch2_read > RC_CH2_MIN) {
              //rc_dir_tgt = FORWARD;
              frq_virtual_ = map(ch2_read, RC_CH2_MIN, RC_CH2_MID - RC_CH2_DEAD, MIN_STP_FRQ, 0);
              
              if(steerDirection == 0) {
                    rc_frq_[0] = frq_virtual_;
                    rc_frq_[1] = frq_virtual_;
              }
              else {
                    radiusRef = car_chasis/tan(steering_angle);         
                    rc_frq_[0] = uint_fast16_t(frq_virtual_*(1 - dis_betw_wheels/(2*radiusRef)));
                    rc_frq_[1] = uint_fast16_t(frq_virtual_*(1 + dis_betw_wheels/(2*radiusRef))); 
                    //Serial.println("Steer direction is ackerman, forward");
          
                    //Serial.print("Left wheel: ");
                    //Serial.println(rc_frq_[0]);
                    //Serial.print("Right wheel: ");
                    //Serial.println(rc_frq_[1]);
                    //Serial.println(radiusRef);
              }    
        }
        else if(ch2_read != 0) {
            //Serial.println("Throttle input invalid, forward");
            //Serial.println(ch2_read);
            //Serial.print(rc_dir_tgt);
            // INVALID CH2 VALUE ERROR HANDLING: TOO LOW
            // rc_error();
        }
    }
    
    // backward
    else if(ch2_read > RC_CH2_MID + RC_CH2_DEAD) {
        rc_dir_tgt = BACKWARD;
        if(ch2_read < RC_CH2_MAX) {
            //rc_dir_tgt = BACKWARD;
            frq_virtual_ = map(ch2_read, RC_CH2_MID + RC_CH2_DEAD, RC_CH2_MAX, 0, MAX_STP_FRQ);
            
            if(steerDirection == 0) {
                rc_frq_[0] = frq_virtual_;
                rc_frq_[1] = frq_virtual_;
                //Serial.println("Steer direction is 0, backward");
    
            }
            else {
               radiusRef = car_chasis/tan((steering_angle));
               rc_frq_[0] = uint_fast16_t(frq_virtual_*(1 - dis_betw_wheels/(2*radiusRef)));
               rc_frq_[1] = uint_fast16_t(frq_virtual_*(1 + dis_betw_wheels/(2*radiusRef))); 
               //Serial.println("Steer direction is ackerman, backward");
    
            }
        }
        else {
          //Serial.println("Throttle input invalid, backward");
          //Serial.println(ch2_read);
          //Serial.print(rc_dir_tgt);
          // INVALID CH2 VALUE ERROR HANDLING: TOO HIGH
          // rc_error();
        }
    }
    // standstill
    else {
        frq_virtual_ = 0;
        rc_frq_[0] = frq_virtual_;
        rc_frq_[1] = frq_virtual_;
    }

    // TODO: virtual differential //////////////////////////////////////////////
    
    //rc_frq_[0] = frq_virtual_;
    //rc_frq_[1] = frq_virtual_;


    //Serial.println(steering_angle);
    /*
    Serial.print(0);
    Serial.print(" ");
    Serial.print(20000);
    Serial.print(" ");
    Serial.print(rc_frq_[0]); // Right tyre
    Serial.print(" ");
    Serial.println(rc_frq_[1]); // Left tyre
    */

    /* ---------- 4. Control target update ---------------------------------- */
    // Steering
   Herkulex.clearError(SV_ID);
   Herkulex.moveOne(SV_ID, rc_sa_, 100, LED_GREEN); // THIRD ARGUMENT "100" IS TIME TO STEER, BEFORE "int(rc_sa_/5)" WAS USED, DOMINIK
//    Serial.println(Herkulex.stat(SV_ID));
//    Serial.print("Get servo Angle:");
//    Serial.println(Herkulex.getAngle(SV_ID));
    // Throttle, controlled acceleration
    for(uint_fast8_t iMot = 0; iMot < 2; ++iMot) {
      newDir |= setDirTarget(iMot, rc_dir_tgt);
      newFrq |= setFrqTarget(iMot, rc_frq_[iMot]);
    }    
    // Throttle, direct. USE ONLY FOR DEBUGGING PURPOSES!
    // for(uint8_t iM=0; iM<2; ++iM) {
    //   if(rc_dir_tgt != steppers[iM].dir) switchDirection(iM);
    //   setTimerFrequency(rc_frq_[iM], 0L, iM);
    // }
    // activateAll();
  
  }
  
  /* ========== AUTONOMOUS CONTROL MODE ===================================== */
  else if(ch3_read > RC_CH3_THR
          && ch3_read > RC_CH3_MAXALERT
          && ch3_read < RC_CH3_MAX) {

//    Serial.println("The car is in autonomous mode");
    /* ---------- 1. Mode switch handling ----------------------------------- */
    if(mode_ != AUTONOMOUS) {
      // nh.loginfo("AUTONOMOUS mode");
      resetMotors();
      resetRosCmd();
      // TODO: indicate autonomous mode ////////////////////////////////////////
      digitalWrite(LED_BUILTIN, HIGH);
      // TODO: activate manual mode interrupt //////////////////////////////////
      mode_ = AUTONOMOUS;
    }
    
    /* ---------- 2. Control target update ---------------------------------- */
    if(ch2_read > RC_CH2_THR) {
//    Serial.println(ch2_read);
      if(ros_newCmd_) {
//         char str2[50];
//         sprintf(str2, "SA: %i | DIR %i | M1 %u | M2 %u", ros_sa_, ros_dir_tgt, ros_frq_[0], ros_frq_[1]);
//         nh.loginfo(str2);
        
        // Steering
        Herkulex.clearError(SV_ID);
        Herkulex.moveOne(SV_ID, ros_sa_, 100, LED_BLUE); // THIRD ARGUMENT "100" IS TIME TO STEER, BEFORE "int(ros_sa_/5)" WAS USED, DOMINIK
        // Throttle, controlled acceleration
        for(uint_fast8_t iMot = 0; iMot<2; ++iMot) {
          newDir |= setDirTarget(iMot, ros_dir_tgt);
          newFrq |= setFrqTarget(iMot, ros_frq_[iMot]);
        }
        // Throttle, direct. USE ONLY FOR DEBUGGING PURPOSES!
        // for(uint8_t iM=0; iM<2; ++iM) {
        //   if(ros_dir_tgt != steppers[iM].dir) switchDirection(iM);
        //   setTimerFrequency(ros_frq_[iM], 0, iM);
        // }
        // activateAll();
        ros_newCmd_ = false;
      } else {
         nh.loginfo("killswitch active, no new cmd!");
      }    
    }
    else {
      nh.loginfo("killswitch inactive!");
      resetMotors();
      resetRosCmd();
    }
  }  
  else if(ch3_read != 0) {
    // INVALID CH3 VALUE ERROR HANDLING
    // nh.loginfo("RC CH3 error");
  }

  /* ========== INCATIVE CONTROL MODE ======================================= */
  if(!ch3_read && !ch2_read) {
    if(mode_ != INACTIVE) {
      rc_inactive();
    }
  }

  /* ========== SPEED CONTROLLER UPDATE ===================================== */
  if(newDir || newFrq) {
    activateAll();
    initRamp();
  }
  READING = digitalRead(M2_DIR_PIN);
  //Serial.println(READING);
  
  nh.spinOnce(); 

}

/** ----------------------------------------------------------------------------
 * REMOTE CONTROL
 * -------------------------------------------------------------------------- */
// initRC - RC setup function
// Enables the external interrupts that are used to read the RC PWM values.
void initRC(void) {
  pinMode(RC_CH1, INPUT);
  pinMode(RC_CH2, INPUT);
  pinMode(RC_CH3, INPUT);
  attachInterrupt(digitalPinToInterrupt(RC_CH1), ch1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH2), ch2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH3), ch3ISR, CHANGE);
}

// readChX - PWM readout functions
// These could in theory all be collapsed into one function each accessing
// arrays. However, this would impose a slight decrease in performance.
// return: PWM value for channel X, 0 if timeout (RC_IDLE)
unsigned long readCh1(void) {
  readingCh1_ = true; // makes sure that the following assignments aren't interrupted
  unsigned long timeBuffer = ch1_time_;
  unsigned long readBuffer = ch1_read_;
  readingCh1_ = false;
  unsigned long now = micros();
  // check timeout (RC_IDLE)
  if(now - timeBuffer > RC_IDLE) {
    return 0;
  } else {
    return readBuffer;
  }
}

unsigned long readCh2(void) {
  readingCh2_ = true;
  unsigned long timeBuffer = ch2_time_;
  unsigned long readBuffer = ch2_read_;
  readingCh2_ = false;
  unsigned long now = micros();
  if(now - timeBuffer > RC_IDLE) {
    return 0;
  } else {
    return readBuffer;
  }
}

unsigned long readCh3(void) {
  readingCh3_ = true;
  unsigned long timeBuffer = ch3_time_;
  unsigned long readBuffer = ch3_read_;
  readingCh3_ = false;
  unsigned long now = micros();
  if(now - timeBuffer > RC_IDLE) {
    return 0;
  } else {
    return readBuffer;
  }
}

// chXISR - Interrupt service routines
// These are executed on PWM input flanks
void ch1ISR(void) {
  if(!readingCh1_ && !(updated_&ch1_bm_)) {
    // Input high: start the timer
    if(CH1INT) {
      ch1_time_ = micros();
      skip_ = false; // timer delayed flag
    }
    // Input low and not delayed: stop the timer
    else if(!skip_) {
      ch1_read_ = micros() - ch1_time_;
      updated_ &= ch1_bm_;
      // EVALUATE: disable ch2ISR here /////////////////////////////////////////
      if(updated_&allCh) {
        updated_ = 0;
        // EVALUATE: enable all rc ISRs here ///////////////////////////////////
      }
    }
  } else {
    skip_ = true;
  }
}

void ch2ISR(void) {
  if(!readingCh2_ && !(updated_&ch2_bm_)) {
    skip_ = true;
    if(CH2INT) {
      ch2_time_ = micros();
    } else {
      ch2_read_ = micros() - ch2_time_;
      updated_ &= ch2_bm_;      
      if(updated_&allCh) {        
        updated_ = 0;        
      }
    }
  }
}

void ch3ISR(void) {
  if(!readingCh3_ && !(updated_&ch3_bm_)) {
    skip_ = true;
    if(CH3INT) {
      ch3_time_ = micros();
    } else {
      ch3_read_ = micros() - ch3_time_;
      updated_ &= ch3_bm_;
      if(updated_&allCh) {
        updated_ = 0;
      }
    }
  }
}

// Blocking version: slightly more accurate, but also blocks execution of other
// functions
// unsigned long readCh1Blocking(void) {
//   readingCh1_ = true;
//   unsigned long timeBuffer = ch1_time_;
//   unsigned long readBuffer = ch1_read_;
//   readingCh1_ = false;
//   return readBuffer;
// }

// void ch1ISRBlocking(void) {
//   if(!readingCh1_) {
//     ch1_time_ = micros();
//     while (true) {
//       if(!CH1INT) {
//         ch1_read_ = micros() - ch1_time_;
//         break;
//       }
//       if(micros() - ch1_time_ > 3000) {
//         ch1_read_ = 0;
//         break;
//       }
//     }
//   }
// }

/** ----------------------------------------------------------------------------
 * STEPPER MOTORS & DRIVERS
 * -------------------------------------------------------------------------- */
/* __________ Setup functions _______________________________________________ */
// initMX - pin setup functions
void initM1(void) {
  // TMC 2130
  // See https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2130_datasheet.pdf
  // Chapter 22, Page 81 for a detailed guide on how to set this up.
    m1_driver.begin();
    m1_driver.enable();
  
//  m1_driver.begin();
//  m1_driver.rms_current(1200);
//  m1_driver.microsteps(16);
//  m1_driver.hold_delay(1);
//  // m1_driver.power_down_delay(0);
//  
//  m1_driver.stealthChop(true);
//  m1_driver.stealth_autoscale(true);
//  m1_driver.stealth_gradient(4);
//  m1_driver.stealth_amplitude(200);
//  m1_driver.stealth_freq(0b01);
//  m1_driver.stealth_max_speed(110);
//
//  m1_driver.off_time(4);
//  m1_driver.blank_time(1);
//  m1_driver.hysteresis_start(6);
//  m1_driver.hysteresis_end(0);
//  // m1_driver.sync_phases(5);
//
//  m1_driver.THIGH(64);
//  m1_driver.sg_stall_value(10);
//  m1_driver.coolstep_min_speed(107);
//  m1_driver.sg_min(2);
//  m1_driver.sg_step_width(2);
//  // m1_driver.semax(2);
//
//  m1_driver.diag1_stall(1);
//  m1_driver.diag1_active_high(1);
//  // m1_driver.sedn(0b01);
//  
  digitalWrite(M1_DIR_PIN, LOW);
  digitalWrite(M1_STEP_PIN, LOW);
//  pinMode(M1_DIAG1_PIN, INPUT);

}

void initM2(void) {

  m2_driver.begin();
  m2_driver.enable();
//  m2_driver.begin();
//  m2_driver.rms_current(1200);
//  m2_driver.microsteps(16);
//  m2_driver.hold_delay(1);
//  // m2_driver.power_down_delay(0);
//  
//  m2_driver.stealthChop(true);
//  m2_driver.stealth_autoscale(true);
//  m2_driver.stealth_gradient(4);
//  m2_driver.stealth_amplitude(200);
//  m2_driver.stealth_freq(0b01);
//  m2_driver.stealth_max_speed(110);
//
//  m2_driver.off_time(4);
//  m2_driver.blank_time(1);
//  m2_driver.hysteresis_start(6);
//  m2_driver.hysteresis_end(0);
//  // m2_driver.sync_phases(5);
//
//  m2_driver.THIGH(64);
//  m2_driver.sg_stall_value(10);
//  m2_driver.coolstep_min_speed(107);
//  m2_driver.sg_min(2);
//  m2_driver.sg_step_width(2);
//  // m2_driver.semax(2);
//
//  m2_driver.diag1_stall(1);
//  m2_driver.diag1_active_high(1);
//  
  digitalWrite(M2_DIR_PIN, HIGH);
  digitalWrite(M1_STEP_PIN, LOW);
//  pinMode(M2_DIAG1_PIN, INPUT);
}

// setStepperMode (TMC2100 only)
// Sets the mode of operation for the stepper motor drivers and the micro
// stepping resolution.
// Params:
// - motorID
// - mode (SPREADCYCLE(SP) / STEALTHCHOP(ST))
// - microsteps (SP:1,2,4,16; SC:4,16)) // make sure to change the ms resolution in the header
// See the TMC2100 manual for a more detailed explanation
void setStepperMode(uint_fast8_t motorID, uint_fast8_t mode, uint_fast8_t microsteps) {
  int_fast8_t CFG1, CFG2;  
  switch(mode) {
    case SPREADCYCLE:
      switch(microsteps) {
        case 1:
          // Serial.println("SP1");  
          CFG2 = LOW; CFG1 = LOW; break;
        case 2:
          // Serial.println("SP2");
          CFG2 = LOW; CFG1 = FLOATING; break; // with interpolation
          // CFG2 = LOW; CFG1 = HIGH; break;
        case 4:
          // Serial.println("SP4");
          CFG2 = HIGH; CFG1 = FLOATING; break; // with interpolation
          // CFG2 = HIGH; CFG1 = LOW; break;
        case 16:
          // Serial.println("SP16");
          CFG2 = FLOATING; CFG1 = LOW; break; // with interpolation
          // CFG2 = HIGH; CFG1 = HIGH; break;
        default:
          // Serial.println("SP16d");
          CFG2 = FLOATING; CFG1 = LOW; break; // 16, with interpolation
      }
      break;
    case STEALTHCHOP:
      CFG2 = FLOATING;
      switch(microsteps) {
        case 4:
          // Serial.println("SC4");
          CFG1 = HIGH; break;
        case 16:
          // Serial.println("SC16");
          CFG1 = FLOATING; break;
        default:
          // Serial.println("SC16d");
          CFG1 = FLOATING; break; // 16
      }
      break;
  }
  if(CFG2 == FLOATING) {
    pinMode(cfg2_pin[motorID], INPUT);
  } else {
    pinMode(cfg2_pin[motorID], OUTPUT);
    digitalWrite(cfg2_pin[motorID], CFG2);
  }
  if(CFG1 == FLOATING) {
    pinMode(cfg1_pin[motorID], INPUT);
  } else {
    pinMode(cfg1_pin[motorID], OUTPUT);
    digitalWrite(cfg1_pin[motorID], CFG1);
  }
}

// setFastPWM - Enable dynamic PWM timers (1,3,4,5)
void setFastPWM(void) {
  // make sure all timers are active (Power Reduction Registers)
  bitClear(PRR0, PRTIM1);
  bitClear(PRR1, PRTIM3);

  // set all timers to Fast PWM Mode, with toggle OCnA/B to low on match
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << WGM11) | (0 << WGM10);
  TCCR1B = (1 << WGM13)  | (1 << WGM12);
  TCCR3A = (1 << COM3A1) | (0 << COM3A0) | (1 << WGM31) | (0 << WGM30);
  TCCR3B = (1 << WGM33)  | (1 << WGM32);
}

/* __________ Communication & control functions _____________________________ */
// Set the EN and DIR pins respectively
void activate(uint_fast8_t channel) {
  digitalWrite(en_pin[channel], ENABLE);
  // delayMicroseconds(1);
  active_ = true;
}

void activateAll() {
  if(!active_) {
    for(uint_fast8_t iC = 0; iC<NUM_MOTORS; ++iC) {
      digitalWrite(en_pin[iC], ENABLE);
    }
    // delayMicroseconds(1);
    active_ = true;
  }
}

void deactivate(uint_fast8_t channel) {
  digitalWrite(en_pin[channel], DISABLE);
  // delayMicroseconds(1);
  active_ = false;
}

void deactivateAll() {
  if(active_) {
    for(uint_fast8_t iC = 0; iC<NUM_MOTORS; ++iC) {
      digitalWrite(en_pin[iC], DISABLE);
    }
    // delayMicroseconds(1);
    active_ = false;
  }
}

void switchDirection(uint8_t channel) {
  // nh.loginfo("Switching direction");
  if(steppers[channel].dir == FORWARD){
    digitalWrite(dir_pin[channel], direction_mod[channel]>0?HIGH:LOW);
    steppers[channel].dir = BACKWARD;
  } else {
    digitalWrite(dir_pin[channel], direction_mod[channel]>0?LOW:HIGH);
    steppers[channel].dir = FORWARD;
  }
  delayMicroseconds(10);
}

/* __________ PFM timer functions (timers 1,3,4,5) __________________________ */
// setPrescaler
// Sets the prescaler registers for the PWM frequency generation.
// Parameters:
// - prescaler: selected prescaler value
// - channel: selected motor channel
void setPrescaler(int prescaler, int channel) {
  byte conf = 0; // do NOT change this, the compiler "optimizes" this away -.-
  switch (prescaler) {    
    case 0:
      // conf |= (0 << CS12) | (0 << CS11) |(0 << CS10);
      break;
    case 1:
      conf |= (0 << CS12) | (0 << CS11) |(1 << CS10);
      break;
    case 8:
      conf |= (0 << CS12) | (1 << CS11) |(0 << CS10);
      break;
    case 64:
      conf |= (0 << CS12) | (1 << CS11) |(1 << CS10);
      break;
    case 256:
      conf |= (1 << CS12) | (0 << CS11) |(0 << CS10);
      break;
    case 1024:
      conf |= (1 << CS12) | (0 << CS11) |(1 << CS10);
      break;
    default:
      conf |= (0 << CS12) | (0 << CS11) |(0 << CS10);
      break;
  }  
  switch (channel) {
    case 0:
      TCCR1B &= ~((1 << CS12) | (1 << CS11) |(1 << CS10)); // reset prescaler
      TCCR1B |= conf;
      break;
    case 1:
      TCCR3B &= ~((1 << CS12) | (1 << CS11) |(1 << CS10));
      TCCR3B |= conf;
      break;
  }
}

// setTimerFrequency
// Sets the PWM waveform generators as close as possible to the selected
// frequency.
// Parameters:
// - Herz, milliHerz: selected frequency
// - channel: selected motor channel
void setTimerFrequency(long Herz, long milliHertz, uint_fast8_t channel) {
  long top = 1;
  if (Herz >= 8000000L) { // Highest frequency: 16/2 MHz
    return;
  }
  else if (Herz >= 244) {
    setPrescaler(1, channel);
    top = 16000000000L/(1000L*Herz + milliHertz);
  }
  else if (Herz >= 31) {
    setPrescaler(8, channel);
    top = 2000000000L/(1000L*Herz + milliHertz);
  }
  else if (Herz >= 3 && milliHertz >= 815) {
    setPrescaler(64, channel);
    top = 250000000L/(1000L*Herz + milliHertz);
  }

  else if ((Herz > 0) || (Herz == 0 && milliHertz >= 954)) {
    setPrescaler(256, channel);
    top = 62500000L/(1000L*Herz + milliHertz);
  }
  else if (Herz == 0 && milliHertz >= 239) {
    setPrescaler(1024, channel);
    top = 15625000/milliHertz;
  }
  else {
    setPrescaler(0, channel);
    top = 65535;
  } 
  switch (channel) {
    case 0:
      if (TCNT1 > top-1)
        TCNT1 = 65535;
      ICR1 = top-1;
      OCR1A = (top-1L)/2L;
      break;
    case 1:
      if (TCNT3 > top-1)
        TCNT3 = 65535;
      ICR3 = top-1;
      OCR3A = (top-1L)/2L;
      break;    
  }
}

/** ----------------------------------------------------------------------------
 * RAMP TIMER / SPEED CONTROLLER FUNCTIONS (timer 2)
 * -------------------------------------------------------------------------- */
/* __________ Setup functions _______________________________________________ */
// initRampTimer - Ramp up/down timer initialization
void initRampTimer(void) {
  // Power on Timer 2
  bitClear(PRR0, PRTIM2);
  // Mode to CTC toggle
  TCCR2A = _BV(WGM21);
  // Prescaler to 1024
  TCCR2B = _BV(CS22)|_BV(CS21)|_BV(CS20);
  // set to 61 Hz (lowest possible) -> 16 MHz / Prescaler / Compare Match
  OCR2A = T2_COMPARE_MATCH;
}

// veltodir
// Returns the directions flag corresponding to the velocity parameter
// (essentially a sgn() function)
inline int_fast8_t veltodir(float vel) {
  return vel<0?-1:1;
}

// veltofrq
// Calculates and returns the frequency corresponding to the velocity parameter
inline uint_fast16_t veltofrq(float vel) {
  return lround(abs(vel)/60.0 * frq_scale_);
}

// setDirTarget
// Sets the direction target (dir) for the motor iS.
// Returns true if the requested target differs from the current one, else false
bool setDirTarget(uint8_t iS, int_fast8_t dir) {  
  if(steppers[iS].dir_tgt != dir) {
    disableRmpTimer();
    steppers[iS].dir_tgt = dir;
    stp_change_ |= m_bm[iS]; // update "change request" flag
    return true;
  }
  return false;
}

// setFrqTarget
// Sets the frequency target (f_req) for the motor iS.
// Returns true if the requested target differs from the current one, else false
bool setFrqTarget(uint8_t iS, int_fast16_t f_req) {
  if(steppers[iS].f_tgt != f_req) { // DANGEROUS MAYBE
    disableRmpTimer();
    steppers[iS].f_tgt = f_req; // stepping frequency target
    stp_change_ |= m_bm[iS]; // update "change request" flag
    return true;
  }
  return false;
}

// setVelTarget
// Calculates the frequency and direction corresponding to the requested
// velocity (v_req) and sets the targets for the motor iS.
// Returns true if the requested target differs from the current one, else false
bool setVelTarget(uint8_t iS, float v_req) {
  int_fast8_t dir_req = veltodir(v_req);
  uint_fast16_t f_req = veltofrq(v_req);
  steppers[iS].v_tgt = v_req;   // velocity target
  bool vd = setDirTarget(iS, dir_req);
  bool vf = setFrqTarget(iS, f_req);
  return (vd || vf);
}

// setVelTargetSolo
// Just convenience function to update the v_tgt of motor iS.
void setVelTargetSolo(uint8_t iS, float v_req) {
  steppers[iS].v_tgt = v_req;   // velocity target
}

// initRamp
// Triggers the ramp timer if new targets have been set. Initializes a movement
// in accordance to the set velocity and direction targets.
void initRamp(void) {
  if(!stp_change_) {
    return;
  }
  for(uint_fast8_t iM = 0; iM<NUM_MOTORS; ++iM) {
    // change indicated
    if(stp_change_ & m_bm[iM]) {
      int_fast16_t f_target = 0; // initial change
      bool switchDir = false;
      // I Change in direction
      if(steppers[iM].dir != steppers[iM].dir_tgt) {      
        // - from standstill
        if(steppers[iM].f == 0) { 
          // nh.loginfo("standstill");
          switchDirection(iM);
          // to movement
          if(steppers[iM].f_tgt) {
            if(deltaF_ >= steppers[iM].f_tgt) {
              f_target = steppers[iM].f_tgt;          
              stp_change_ &= ~m_bm[iM];
              steppers[iM].mode = RUN;
            }
            else {
              f_target = deltaF_;
              steppers[iM].mode = ACCEL;
            }
          }
          // to standstill
          else {
            stp_change_ &= ~m_bm[iM];
            steppers[iM].mode = STOP;
            continue;
          }
        }
        // - from movement
        else {
          f_target = steppers[iM].f - deltaF_;    
          if(f_target <= 0) {
            switchDir = true;
            f_target = 0;
            if(steppers[iM].f_tgt == 0) {
              stp_change_ &= ~m_bm[iM];
              //Serial.println("Change in direction - DECEL - STOP");
              steppers[iM].mode = STOP;
            } else {
              //Serial.println("Change in direction - DECEL - ACCEL");
              steppers[iM].mode = ACCEL;
            }
          } else {
            //Serial.println("Change in direction - DECEL");
            steppers[iM].mode = DECEL;
          }
        }
      }
      // II Change in speed
      // - acceleration
      else if(steppers[iM].f < steppers[iM].f_tgt) {
        //Serial.println("Change in speed - ACCEL");
        f_target = steppers[iM].f + deltaF_;
        if(f_target >= steppers[iM].f_tgt) {
          //Serial.println("Change in speed - ACCEL - RUN");
          stp_change_ &= ~m_bm[iM];
          f_target = steppers[iM].f_tgt;
          steppers[iM].mode = RUN;
        } else {
          //Serial.println("Change in speed - ACCEL - ACCEL");
          steppers[iM].mode = ACCEL;
        }
      }
      // - deceleration
      else if(steppers[iM].f > steppers[iM].f_tgt) {
        f_target = steppers[iM].f - deltaF_;
        if(f_target <= steppers[iM].f_tgt) {
          stp_change_ &= ~m_bm[iM];
          f_target = steppers[iM].f_tgt;
          if(f_target) {
            //Serial.println("Change in speed - DECEL - RUN");
            steppers[iM].mode = RUN;
          } else {
            //Serial.println("Change in speed - DECEL - STOP");
            steppers[iM].mode = STOP;
          }
        } else {
          //Serial.println("Change in speed - DECEL -  DECEL");
          steppers[iM].mode = DECEL;
        }
      // III Error: change indicated without valid targets
      } else {
        stp_change_ &= ~m_bm[iM];
        if(steppers[iM].f) {
          steppers[iM].mode = RUN;
        } else {
          steppers[iM].mode = STOP;
        }
        continue;
      }

      // Initial timer update
      setTimerFrequency(f_target, 0, iM);
      steppers[iM].f = f_target;
      if(switchDir == true && steppers[iM].f == 0) {
        //Serial.println("Direction was changed in function");
        switchDirection(iM);
      }
    }
  }
  if(stp_change_) {    
    enableRmpTimer();
  }
}

/* Interrupt service routine that adjusts the stepping frequency in fixed
 * intervals (see initRampTimer). */

ISR(TIMER2_COMPA_vect) {
  if(stp_change_) {
    for(uint_fast8_t iM = 0; iM<NUM_MOTORS; ++iM) {
      if(stp_change_ & m_bm[iM]) {
        bool switchDir = false;
        int_fast16_t target = 0;
        // A - Acceleration
        if(steppers[iM].mode == ACCEL) {
          target = steppers[iM].f + deltaF_;
          // target reached
          if(target >= steppers[iM].f_tgt) {
            target = steppers[iM].f_tgt;
            steppers[iM].v = steppers[iM].v_tgt;
            steppers[iM].mode == RUN;
            stp_change_ &= ~m_bm[iM];
          }
        }
        // B - Deceleration
        else if(steppers[iM].mode == DECEL) {
          target = steppers[iM].f - deltaF_;
          // reverse direction
          if(target >= 0 && steppers[iM].dir != steppers[iM].dir_tgt) {
            steppers[iM].mode = ACCEL;
            target = 0;
            switchDir = true;          
          }
          // Stop
          else if(target >= 0 && steppers[iM].f_tgt == 0) {
            target = 0;
            steppers[iM].v = 0.0;
            steppers[iM].mode == STOP;
            stp_change_ &= ~m_bm[iM];
          }
          // Target reached
          else if(target <= steppers[iM].f_tgt && steppers[iM].dir == steppers[iM].dir_tgt) {
            target = steppers[iM].f_tgt;
            steppers[iM].v = steppers[iM].v_tgt;
            steppers[iM].mode == RUN;
            stp_change_ &= ~m_bm[iM];
          }
        // C - Error case
        } else {
          // NOT IN A DYNAMIC MODE ERROR HANDLING
          stp_change_ &= ~m_bm[iM];
          continue;
        }
        // TODO: EVALUATE PUTTING THIS EXTRA FOR SYNC /////////////////////////
        setTimerFrequency(target, 0, iM);        
        steppers[iM].f = target;
        if(switchDir) {
          switchDirection(iM);
        }
      }
    }
  }  
  else {
    // disable interrupt   
    resetRmpTimer();
  }
}

void enableRmpTimer(void) {
  if(!ramp_timer_active_) {
    bitSet(TIMSK2, OCIE2A); // enable timer 2 interrupt
    ramp_timer_active_ = true;
  }  
}

void disableRmpTimer(void) {
  if(ramp_timer_active_) {
    bitClear(TIMSK2, OCIE2A); // disable timer 2 interrupt
    ramp_timer_active_ = false;
  }
}

void resetRmpTimer(void) {
  if(ramp_timer_active_) {
    bitClear(TIMSK2, OCIE2A); // disable timer 2 interrupt
    ramp_timer_active_ = false;
    if(stp_change_) {
      stp_change_ = 0;
    }
  }
}

void resetPWM(void) {
  for(uint_fast8_t iM = 0; iM < NUM_MOTORS; ++iM) {
    if(steppers[iM].mode != STOP) {
      setTimerFrequency(0, 0, iM);
      steppers[iM].f = 0;
      steppers[iM].f_tgt = 0;
      if(steppers[iM].dir != FORWARD) {
        switchDirection(iM);
      }
      steppers[iM].dir = FORWARD;
      steppers[iM].dir_tgt = 1;
      steppers[iM].mode == STOP;
    }
  }
}

/** ----------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -------------------------------------------------------------------------- */

void resetMotors(void) {
  deactivateAll();
  resetRmpTimer();
  resetPWM();
}

void rc_error(void) {
  // nh.loginfo("RC error!");
  resetMotors();
  resetRCCmd();
}

void rc_inactive(void) {
  if(mode_ != INACTIVE) {
    // nh.loginfo("RC inactive!");
    resetMotors();
    resetRCCmd();
    resetRosCmd();
    mode_ = INACTIVE;
  }
}

void resetRosCmd(void) {
  ros_sa_ = SV_MID;
  ros_frq_[0] = 0;
  ros_frq_[1] = 0;
}

void resetRCCmd(void) {
  rc_sa_ = SV_MID;
  rc_frq_[0] = 0;
  rc_frq_[1] = 0;
}
