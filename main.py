import time
from dynamixel_python import DynamixelManager

from neopixel_12 import np, NP_COUNT, NP_PIN, COLORS, COLOR_TUPLES

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
    read_attempts = 0
    pos = None
    while read_attempts < 10:
        try:
            pos = testMotor.get_present_position()
            break
        except:
            read_attempts += 1
    if pos is None:
        return 
    rots = pos//4096
    # clock_pos = int(pos/4096*NP_COUNT)
    clock_pos = int(pos%4096*NP_COUNT)
    clock_pos = min(NP_COUNT, clock_pos)
    print(clock_pos)
    for i in range(clock_pos):
        # np[i] = (1,0,0,0)
        np[i] = COLOR_TUPLES[rots]
    for j in range(clock_pos, NP_COUNT):
        np[j] = (0,0,0,0)
    np.write()

# 
def tick():
    t_start = time.time()
    pos_start = testMotor.get_present_position()
    # Calculate the set time in seconds
    total_time = pos_start/4096*60
    # get how much time to tick for
    # home position is 0
    while testMotor.get_present_position() > 10:
        t = time.time()
        del_t = 0
        while del_t < 1:
            del_t = t-t_start
        del_pos = int(del_t/60*4096)
        pos_t = pos_start - del_pos
        pos_t = max(2, pos_t)
        
        testMotor.set_torque_enable(True)
        testMotor.set_goal_position(pos_t)
        time.sleep(0.05)
        testMotor.set_torque_enable(False)
        listen_write()

def loop():
    while True:
        listen_write()
        time.sleep(0.1)
 
if __name__=="__main__":
    reset_motor()
    time.sleep(1)
    testMotor.set_torque_enable(False)
    time.sleep(1)
    # loop()
   
# if __name__ == '__main__':
    # single_motor_example()
