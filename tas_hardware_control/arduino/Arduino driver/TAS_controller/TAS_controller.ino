#include "include/TAS_controller.hh"    //definition of all pins and variables
#include <util/atomic.h>            
#include <ArduinoHardware.h>

#include <tas_msgs/Control.h>           //tas messages
#include <Herkulex.h>                   //driver for the servo motor
#include <std_msgs/Float32.h>

#include <Arduino.h>


int ros_servo_cmd;
int ros_motor_cmd;

int servo_output;
int motor_output;

// Encoder needs to be implemented. --Luis Bähr, July 2021
/*
int encoder1PinA =4;
int encoder1PinB =2;
volatile long encoder1Pos = 0;

int encoder2PinA =6;
int encoder2PinB =3;
volatile long encoder2Pos = 0;


// setup ros message for encoder data publishing
std_msgs::Float32 enc1pub;
std_msgs::Float32 enc2pub;
std_msgs::Float32 enc3pub;

ros::Publisher rospub_enc1("/encoder/VelX", &enc1pub);  //publisher for linear Vel
ros::Publisher rospub_enc2("/encoder/VelL", &enc2pub);  //publisher for deltaTheta
ros::Publisher rospub_enc3("/encoder/VelR", &enc3pub);  //publisher for angular Vel
*/

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
      motor_output = map(ros_motor_cmd,1000,1500,MAX_STP_FRQ,0);
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

// Subscriber to receive control messages from ROS
ros::Subscriber<geometry_msgs::Vector3> sub("/servo", servo_cb);


//initial setup, run only once at the beginning
int timer0= 0;
void setup() {
 
  // setup nodes to communicate with ROS
  nh.initNode();
  nh.subscribe(sub);
  // Encoder needs to be implemented. --Luis Bähr, July 2021
  /*
  nh.advertise(rospub_enc1);
  nh.advertise(rospub_enc2);
  nh.advertise(rospub_enc3);*/
  Serial.begin(115200); // for debugging purposes

  // Setup steppers M1 and M2 from stepper.ino
  stepper_setup();

  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Steering servo  
  Herkulex.beginSerial2(115200); //open serial port 1 
  Herkulex.reboot(SV_ID); //reboot first motor
  delay(500);
  Herkulex.initialize(); //initialize motors  
  delay(200);

  // initialize remote control from remote_ctrl.ino 
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
  // Encoder for odometry not yet implemented
  /*
  if (counterForOdom == PrviouscounterForOdom + 1){
  
    //publish the encoder data
    enc1pub.data = VelX;
    enc2pub.data = VelLeft; 
    enc3pub.data = VelRight;

     rospub_enc1.publish(&enc1pub);
     rospub_enc2.publish(&enc2pub);
     rospub_enc3.publish(&enc3pub);
  }
  PrviouscounterForOdom=counterForOdom; */


  unsigned long ch3_read = readCh3(); // control mode 
  unsigned long ch2_read = readCh2(); // throttle
  

  bool newDir = false; // new direction flag
  bool newFrq = false; // new frequency / velocity flag

  
  /* ========== MANUAL CONTROL MODE ========================================= */
  if(ch3_read < RC_CH3_THR && ch3_read > RC_CH3_MINALERT) {
//  Serial.println("The car is in manual mode");
    /* ---------- 1. Mode switch handling ----------------------------------- */
    if(mode_ != MANUAL){
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


      } 
      else if(ch1_read <= RC_CH1_MIN) {
        // INVALID CH1 VALUE ERROR HANDLING: TOO LOW
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

      }
      else if(ch1_read >= RC_CH1_MAX){
        // INVALID CH1 VALUE ERROR HANDLING: TOO HIGH
        rc_sa_ = SV_MID + SV_LIM;

      }
      else {
        Serial.print("INVALID RIGHT STEER");
      }
    }
    // center steer
    else {
      rc_sa_ = SV_MID;
    }
    
    // calculate steering values used for throttle
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
              frq_virtual_ = map(ch2_read, RC_CH2_MIN, RC_CH2_MID - RC_CH2_DEAD, MAX_STP_FRQ, 0);
              
              if(steerDirection == 0) {
                    rc_frq_[0] = frq_virtual_;
                    rc_frq_[1] = frq_virtual_;
              }
              else {
                    radiusRef = car_chasis/tan(steering_angle);         
                    rc_frq_[0] = uint_fast16_t(frq_virtual_*(1 - dis_betw_wheels/(2*radiusRef)));
                    rc_frq_[1] = uint_fast16_t(frq_virtual_*(1 + dis_betw_wheels/(2*radiusRef))); 
              }    
        }
        else if(ch2_read != 0) {
            // INVALID CH2 VALUE ERROR HANDLING: TOO LOW
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
          // INVALID CH2 VALUE ERROR HANDLING: TOO HIGH
        }
    }
    // standstill
    else {
        frq_virtual_ = 0;
        rc_frq_[0] = frq_virtual_;
        rc_frq_[1] = frq_virtual_;
    }

    /* ---------- 4. Control target update ---------------------------------- */
    // Steering
   Herkulex.clearError(SV_ID);
   Herkulex.moveOne(SV_ID, rc_sa_, 100, LED_GREEN); // THIRD ARGUMENT "100" IS TIME TO STEER, BEFORE "int(rc_sa_/5)" WAS USED, DOMINIK

    // Throttle, controlled acceleration
    setSpeedInHz(rc_frq_[1], rc_frq_[0], rc_dir_tgt);
  
  }
  
  /* ========== AUTONOMOUS CONTROL MODE ===================================== */
  else if(ch3_read > RC_CH3_THR
          && ch3_read > RC_CH3_MAXALERT
          && ch3_read < RC_CH3_MAX) {

//    Serial.println("The car is in autonomous mode");
    /* ---------- 1. Mode switch handling ----------------------------------- */
    if(mode_ != AUTONOMOUS) {
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
        
        // Steering
        Herkulex.clearError(SV_ID);
        Herkulex.moveOne(SV_ID, ros_sa_, 100, LED_BLUE); // THIRD ARGUMENT "100" IS TIME TO STEER, BEFORE "int(ros_sa_/5)" WAS USED, DOMINIK
        // Throttle, controlled acceleration
        for(uint_fast8_t iMot = 0; iMot<2; ++iMot) {
          //newDir |= setDirTarget(iMot, ros_dir_tgt);
          //newFrq |= setFrqTarget(iMot, ros_frq_[iMot]);
        }

      // set the new speed for both motors
      setSpeedInHz(ros_frq_[1], ros_frq_[0], ros_dir_tgt);

        ros_newCmd_ = false;
      } else {
         nh.loginfo("killswitch active, no new cmd!");
      }    
    }
    else {
      nh.loginfo("killswitch inactive!");
      stopMotors();
    }
  }  
  else if(ch3_read != 0) {
    // INVALID CH3 VALUE ERROR HANDLING
  }

  /* ========== INCATIVE CONTROL MODE ======================================= */
  if(!ch3_read && !ch2_read) {
    if(mode_ != INACTIVE) {
      rc_inactive();
    }
  }

  /* ========== SPEED CONTROLLER UPDATE ===================================== */
  if(newDir || newFrq) {
    //activateAll();
    //initRamp();
  }
  
  nh.spinOnce(); 

}

/** ----------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -------------------------------------------------------------------------- */

void rc_error(void) {
  resetRCCmd();
}

void rc_inactive(void) {
  if(mode_ != INACTIVE) {
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
