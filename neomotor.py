import time
from dynamixel_python import DynamixelManager

from neopixel_12 import np, NP12_COUNT, NP12_PIN, COLORS, COLOR_TUPLES

USB_PORT = '/dev/tty.usbmodem101'

BAUDRATE = 9600
DYNAMIXEL_MODEL = 'xl330-m288'
ID = 1

# def single_motor_example():
if True:
    """
    turn on a single dynamixel and sweep it between position 0 and position 1024 three times
    """
    motors = DynamixelManager(USB_PORT, baud_rate=BAUDRATE)
    testMotor = motors.add_dynamixel('TestMotor', ID, DYNAMIXEL_MODEL)
    motors.init()

    # if not testMotor.ping():
    #     raise BaseException('motor not configured correctly')

    testMotor.set_operating_mode(3)   
    testMotor.set_led(True)
    testMotor.set_torque_enable(True)
    testMotor.set_profile_velocity(262)

    for i in range(3):
        testMotor.set_goal_position(0)
        time.sleep(0.5)

        testMotor.set_goal_position(1024)
        time.sleep(0.5)

    testMotor.set_torque_enable(False)
    testMotor.set_led(False)

def reset_motor():
    testMotor.set_operating_mode(3)  
    testMotor.set_led(True)

    # while not testMotor.get_torque_enable():
    testMotor.set_torque_enable(True)
        # time.sleep(0.5) 
    testMotor.set_profile_velocity(262)
    testMotor.set_goal_position(2000)
    
    testMotor.set_torque_enable(False)

    testMotor.set_led(False)

def listen_write():
    try:
        pos = testMotor.get_present_position()
    except:
        return
    clock_pos = int(pos/4096*NP12_COUNT)
    clock_pos = min(NP12_COUNT, clock_pos)
    print(clock_pos)
    for i in range(clock_pos):
        np[i] = (i,0,0,0)
    for j in range(clock_pos, NP12_COUNT):
        np[j] = (0,0,0,0)
    np.write()

def loop():
    while True:
        listen_write()
        time.sleep(0.2)
 
if __name__=="__main__":
    reset_motor()
    time.sleep(1)
    testMotor.set_torque_enable(False)
   
# if __name__ == '__main__':
    # single_motor_example()
