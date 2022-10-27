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