/** ----------------------------------------------------------------------------
 * STEPPER MOTORS & DRIVERS
 * The FastAccelStepper library of Jochen Kiemes is used.
 * It is available through the Arduino IDE or directly from Github
 *
 * This implementation uses timer1 of the Arduino Mega.
 * Make sure to set "#define FAS_TIMER_MODULE 1" in line 27 of AVRStepperPins.h
 * if you have downloaded a new version of the library. This ensures the use of
 * the correct timer module.  
 * -------------------------------------------------------------------------- */
// includes and stepper specific defines, some are still in the TAS_controller.hh
#define MICROSTEPS 32

#define ACCELERATION 80          // Acceleration in steps per second
#define ACCELERATION_BRAKE 250    // Acceleration for decellerating car

// M1 is left stepper
#define M1_EN_PIN    23
#define M1_DIR_PIN   25
#define M1_STEP_PIN  12   // This was pin 5 for the old configuration. Make sure the correct pin is connected.

// M2 is right stepper
#define M2_EN_PIN    22
#define M2_DIR_PIN   24
#define M2_STEP_PIN  11 

uint8_t dir = FORWARD;
uint8_t M1_isStopped = true;
uint8_t M2_isStopped = true;

// FastAccelStepper specific setup
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1_stepper = NULL;
FastAccelStepper *M2_stepper = NULL;

/* __________ Setup functions _______________________________________________ */
void stepper_setup(void)
{
  // setup steppers
  engine.init();
  M1_stepper = engine.stepperConnectToPin(M1_STEP_PIN);
  M2_stepper = engine.stepperConnectToPin(M2_STEP_PIN);

  // set Pins for stepper M1
  if(M1_stepper){
    M1_stepper->setDirectionPin(M1_DIR_PIN);
    M1_stepper->setEnablePin(M1_EN_PIN, false);   // false  indicates that enable is active high
    M1_stepper->setAutoEnable(true);
  }

  // set Pins for stepper M2
  if(M2_stepper){
    M2_stepper->setDirectionPin(M2_DIR_PIN, false);
    M2_stepper->setEnablePin(M2_EN_PIN, false);
    M2_stepper->setAutoEnable(true); 
  }
}

void setSpeedInHz(uint32_t f_tgt_M1, uint32_t f_tgt_M2, uint8_t dir_tgt)
{
  // set speed (independant of direction)
  M1_stepper->setSpeedInHz(f_tgt_M1*MICROSTEPS);
  M2_stepper->setSpeedInHz(f_tgt_M2*MICROSTEPS);

  // get current speeds of the motors
  int32_t M1_currentSpeed = M1_stepper->getCurrentSpeedInMilliHz()/1000;
  int32_t M2_currentSpeed = M2_stepper->getCurrentSpeedInMilliHz()/1000;

  // calculate acceleration for the motors
  if(dir_tgt == FORWARD)
  {
    // calculate difference to overcome
    int32_t M1_speedDiff = (f_tgt_M1 * MICROSTEPS) - M1_currentSpeed;
    int32_t M2_speedDiff = (f_tgt_M2 * MICROSTEPS) - M2_currentSpeed;

    // calculate relative difference and adjust acceleration
    if(M1_speedDiff > M2_speedDiff)
    {
      float rel_acc = (float) M2_speedDiff / (float) M1_speedDiff;
      M1_stepper->moveByAcceleration(ACCELERATION*MICROSTEPS);
      M2_stepper->moveByAcceleration((int32_t) (ACCELERATION*MICROSTEPS*rel_acc));
    }
    else if(M1_speedDiff < M2_speedDiff)
    {
      float rel_acc = (float) M1_speedDiff / (float) M2_speedDiff;
      M2_stepper->moveByAcceleration(ACCELERATION*MICROSTEPS);
      M1_stepper->moveByAcceleration((int32_t) (ACCELERATION*MICROSTEPS*rel_acc));
    }
    else
    {
      M1_stepper->moveByAcceleration(ACCELERATION*MICROSTEPS);
      M2_stepper->moveByAcceleration(ACCELERATION*MICROSTEPS);
    }
  }
  else
  {
    // calculate difference to overcome
    int32_t M1_speedDiff = (f_tgt_M1 * MICROSTEPS) + M1_currentSpeed;
    int32_t M2_speedDiff = (f_tgt_M2 * MICROSTEPS) + M2_currentSpeed;

    // calculate relative difference and adjust acceleration
    if(M1_speedDiff > M2_speedDiff)
    {
      float rel_acc = (float) M2_speedDiff / (float) M1_speedDiff;
      M1_stepper->moveByAcceleration(-ACCELERATION*MICROSTEPS);
      M2_stepper->moveByAcceleration((int32_t) (-ACCELERATION*MICROSTEPS*rel_acc));
    }
    else if(M1_speedDiff < M2_speedDiff)
    {
      float rel_acc = (float) M1_speedDiff / (float) M2_speedDiff;
      M2_stepper->moveByAcceleration(-ACCELERATION*MICROSTEPS);
      M1_stepper->moveByAcceleration((int32_t) (-ACCELERATION*MICROSTEPS*rel_acc));
    }
    else
    {
      M1_stepper->moveByAcceleration(-ACCELERATION*MICROSTEPS);
      M2_stepper->moveByAcceleration(-ACCELERATION*MICROSTEPS);
    }
  }
}

// function to stop any movement in autonomous mode when trigger is released
void stopMotors()
{
  // stop left wheel
  M1_stepper->setAcceleration(ACCELERATION_BRAKE*MICROSTEPS);
  M1_stepper->applySpeedAcceleration();
  M1_stepper->stopMove();
  // stop right right wheel
  M2_stepper->setAcceleration(ACCELERATION_BRAKE*MICROSTEPS);
  M2_stepper->applySpeedAcceleration();
  M2_stepper->stopMove();
}
