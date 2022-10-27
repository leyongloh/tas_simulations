#! /usr/bin/env python
# coding: utf-8

## @file
# Contains the emulated WiiController for the simulation. Uses code from the teleop_keyboard module.

## @author Martin
## @maintainer Jiangnan 

import rospy
import threading
import signal
import sys, select, termios, tty
from geometry_msgs.msg import Vector3


## WiiControllerSim class that mimics the functionality of the controller provided in the lab.
# Forwards commands to '/servo' to 'arduino/servo' if publish is set to True, otherwise doesn't publish at all.
# Used to interrupt the control flow from the autonomous nodes if the car should stop on users notice.
class WiiControllerSim:
    
    # Default constructor for the WiiControllerSim class.
    # Initializes the forward subscriber, publisher and the publishing switch. 
    def __init__(self):
        self._wii_cmd_pub = rospy.Publisher('arduino/servo', Vector3, queue_size=4)
        self._wii_cmd_sub = rospy.Subscriber('servo', Vector3, self._cmd_pub, queue_size=4)
        self.pub = False
    
    ## Function to keep the object alive while executing all callbacks from the forward subscriber.
    # Acts as a target for threading module to make keyboard input possible while running the forwarding.
    def run(self):
        rospy.spin()

    ## Callback function of the forward subscriber. 
    # Allows for the interruption of autonomous control.
    # @note Publishes the received data to the lower level ArduinoSim control node only if self.pub is set to true. 
    # @param data Callback data from the forward subscriber. Type Vector3.
    def _cmd_pub(self, data):
        cmd_vec = data
        if self.pub:
            self._wii_cmd_pub.publish(cmd_vec)
            
    ## Function switches the publishing state to the opposite of the current state.
    def switch_pub(self):
        self.pub = not self.pub

    ## Function to stop the car movements after quitting the autonomous control mode.
    def stop_car(self):
        ctrl_vec = Vector3()
        ctrl_vec.x = 1500
        ctrl_vec.y = 1500
        self._wii_cmd_pub.publish(ctrl_vec)
        
    ## @var _wii_cmd_pub
    # ROS publisher for 'arduino/servo' topic messages of type Vector3.
    
    ## @var _wii_cmd_sub
    # ROS subscriber to 'servo' topic messages of type Vector3.
    
    ## @var pub
    # Control variable for the subscriber callback. If True, allows forwarding of commands. If False, interrupts control message passing. Type bool.


## TeleopKeyboard class to provide manual control over the car with the keyboard.
# Code adapted from the teleop_twist_keyboard module. 
# @note For further information consult https://github.com/ros-teleop/teleop_twist_keyboard.
class TeleopKeyboard:
    
    ## Default constructor for the keyboard. Initializes the move binding dictionary and the control publisher.
    def __init__(self):
        self._msg = """
        Reading from the keyboard  and Publishing to Arduino Emulation!
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .
        ---------------------------
        
        anything else : stop
        
        Enter : exit teleop mode
        """
        self._move_bindings = {'u':(2000,1000), 'i':(2000,1500),'o':(2000,2000),
                             'j':(1500,1000), 'l':(1500,2000),
                             'm':(1000,1000), ',':(1000,1500),'.':(1000,2000),
            }
        self._ctrl_pub = rospy.Publisher('arduino/servo', Vector3, queue_size = 1)

    ## Starts the teleop functionality. Use instructions can be found in self._msg in the constructor above.
    def run(self):
        self._settings = termios.tcgetattr(sys.stdin)  # Get current input stream state.
        x = 1500  # 1500 is zero velocity in arduino encoding according to tas_autonomous_control C++ files.
        y = 1500
        try:
            print(self._msg)
            while not rospy.is_shutdown():
                key = self._getKey()
                if key in self._move_bindings.keys():  # Assings earlier defined speed and turn values to the control vector.
                    x = self._move_bindings[key][0]
                    y = self._move_bindings[key][1]
                else:  # Includes the 'k' key which would assign (1500,1500) anyways.
                    x = 1500
                    y = 1500
                    if key in['\x03', '\x0d']:  # Ctrl-C or 'Enter'.
                        return
                    
                ctrl_vec = Vector3()
                ctrl_vec.x = x; ctrl_vec.y = y;
                self._ctrl_pub.publish(ctrl_vec)
                
        except Exception as e:
            print(e)
            
        finally:  # Always stop the car after teleoperation.
            ctrl_vec = Vector3()
            ctrl_vec.x = 1500; ctrl_vec.y = 1500;
            self._ctrl_pub.publish(ctrl_vec)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)  # Reset input stream state after operation.
    
    ## Function to get the currently pressed key.
    def _getKey(self):
        tty.setraw(sys.stdin.fileno())  # Set the file descriptor of the system input stream to raw.
        select.select([sys.stdin], [], [], 0)  # Makes sure sys.stdin is ready for reading.
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)  # Reset input stream after key press.
        return key
    
    ## @var _msg
    # Short terminal instruction message on how to use the node.
    
    ## @var _move_bindings
    # Dictionary mapping the different keys towards specific speed and steer comannds. Type dict.
    
    ## @var _ctrl_pub
    # ROS publisher for 'arduino/servo' topic messages of type Vector3.

## Special Ctrl-C interrupt to cleanly finish with multithreading.
def signal_term_handler(signal, frame):
  print('User Keyboard interrupt, exiting..')
  sys.exit(0)

## Main function starts the ros node and initializes the WiiControllerSim and the TeleopKeyboard.
# Prints instructions for the user and then switches modes according to the input.
def main():
    rospy.init_node('Wii_Controller_Sim', anonymous=True)
    wii_ctrl = WiiControllerSim()
    teleop = TeleopKeyboard()
    rospy.loginfo('Wii Controller Sim initialized!')
    wii_msg = """
        #########################################
        ##          Wii Controller Sim         ##
        ##  Enter: Start/Stop autonomous mode  ##
        ##  q/Q: Quit                          ##
        #########################################
        """
    print(wii_msg)
          
    thread1 = threading.Thread(target=wii_ctrl.run, args=())  # Run the forwarding in seperate thread to make input possible.
    thread1.daemon = True
    thread1.start()
    
    while not rospy.is_shutdown():
        try:
            # cmd = raw_input() # raw_input doesn't exist in python 3 anymore --> updated to input()
            cmd = input()
        except KeyboardInterrupt:
            print('Shutdown initialized!')
            thread1.join()
            sys.exit()

        if cmd == '':
            if wii_ctrl.pub == False:
                wii_ctrl.pub = True  # Will change the publishing state in the seperate thread.
                print('Started autonomous mode!')
            else:
                wii_ctrl.pub = False  # Disable forwarding loop, start with teleoperation.
                wii_ctrl.stop_car()
                print('Stopped autonomous mode!')
                teleop.run()
                print('Started autonomous mode!')  # User has manually stopped teleoperation, therefore autonomous mode is switched back on.
                wii_ctrl.pub = True
                print(wii_msg)
        elif cmd in ['q', 'Q']:
            print('Shutdown initialized!')
            break
        else:
            print('Unknown command! Omitting...')
    
    rospy.loginfo('Shutting down Wi Controller Sim!')

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_term_handler)  # Register a different signal handler to make clean interrupts with threading and rospy possible.
    main()
