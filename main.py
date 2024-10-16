import time
from dynamixel_python import DynamixelManager
# from itertools import accumulate
import _thread

from neopixel_12 import np, NP_COUNT, NP_PIN, COLORS, COLOR_TUPLES

USB_PORT = '/dev/tty.usbmodem101'

BAUDRATE = 9600
DYNAMIXEL_MODEL = 'xl330-m288'
ID = 1

TIMESCALES = [
    60,         # seconds-minutes
    60,         # minutes-hours
    24,         # hours-days
    7,          # days-weeks
    4,          # weeks-months
    12,         # months-years
10,             # years-decade

]
# TIMESCALES_CUMU = list(accumulate(a, lambda x,y: x*y))
TIMESCALES_CUMU = [60,
    # 120,
3600, 86400, 604800, 2419200, 29030400, 290304000]


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

    # for i in range(3):
    #     testMotor.set_goal_position(0)
    #     time.sleep(0.5)

    #     testMotor.set_goal_position(1024)
    #     time.sleep(0.5)

    testMotor.set_torque_enable(False)
    testMotor.set_led(False)

def reset_motor():
    testMotor.set_torque_enable(False)
    testMotor.set_operating_mode(4)  
    testMotor.set_led(True)

    # while not testMotor.get_torque_enable():
    testMotor.set_torque_enable(True)
        # time.sleep(0.5) 
    testMotor.set_profile_velocity(262)
    testMotor.set_goal_position(2000)
    
    testMotor.set_torque_enable(False)

    testMotor.set_led(False)

def listen_write(pos=None):
    global breath_mult
    # read_attempts = 0
    # pos = None
    # while read_attempts < 10:
    #     try:
    #         pos = testMotor.get_present_position()
    #         break
    #     except:
    #         read_attempts += 1
    # if pos is None:
    #     return 
    if pos is None:
        pos = safe_read("present_position", max_attempts=10)
    if not pos:
        return
    rots = pos//4096
    rel_pos = pos%4096
    # clock_pos = int(pos/4096*NP_COUNT)
    # clock_pos = int((pos%4096)/4096*NP_COUNT)
    clock_pos = rel_pos/4096*NP_COUNT
    clock_pos = min(NP_COUNT, clock_pos)
    # print(clock_pos, pos)
    # if clock_pos>=0:
    #     if clock
    # clock_pos = round(clock_pos)
        # clock_pos
    # clock_pos = round(clock_pos)
    # if rots == clock_pos == 0:
    #     clock_pos = 
    total_time = TIMESCALES_CUMU[rots]*(rel_pos/4096)
    if rots > 0:
        total_time = max(TIMESCALES_CUMU[rots-1], total_time)
    print(total_time)
    if clock_pos < 1:
        if rots > 0 or clock_pos > 0.1:
            clock_pos = 1
        elif clock_pos < 0.1:
            clock_pos = 0
    else:
        clock_pos = round(clock_pos)
    color_tuple = COLOR_TUPLES[rots%len(COLOR_TUPLES)]
    color_tuple = tuple([int(i*breath_mult) for i in color_tuple])
    for i in range(clock_pos):
        # np[i] = (1,0,0,0)
        np[i] = color_tuple
    for j in range(clock_pos, NP_COUNT):
        np[j] = (0,0,0,0)
    np.write()

# def safe_read(attr, max_attempts=10):
def safe_read(attr, max_attempts=float('inf')):
    if not hasattr(testMotor, f"get_{attr}"):
        return
    get_attr_fn = getattr(testMotor, f"get_{attr}")
    read_attempts = 0
    ret = None
    while read_attempts < max_attempts:
        # print("reading")
        try:
            ret = get_attr_fn()
            break
        except:
            read_attempts += 1
            # time.sleep(0.05)
            time.sleep(0.1)

    return ret
    
def pos2time(pos):
    rots = pos//4096
    rel_pos = pos%4096
    norm_pos = rel_pos/4096
    scale = TIMESCALES_CUMU[rots]
    ret = norm_pos*scale
    return ret

def time2pos(t):
    rots = 0 
    while t > TIMESCALES_CUMU[rots]:
        rots += 1
    scale = TIMESCALES_CUMU[rots]
    pos = rots*4096
    pos += (t/scale)*4096
    return int(pos)
# 
def tick():
    t_start = t = time.time()
    # pos_start = testMotor.get_present_position()
    pos_start = pos = safe_read("present_position", )
    # Calculate the set time in seconds
    # total_time = pos_start/4096*60
    t_total = pos2time(pos_start)
    # get how much time to tick for
    # home position is 0
    # while safe_read("present_position") > 10:
    testMotor.set_operating_mode(4)
    while not testMotor.set_torque_enable(True):
        time.sleep(0.05)
        continue
    testMotor.set_profile_velocity(262)
    del_pos = del2_pos = 0
    del_t = 0
    last_scale = None
    while pos and pos > 10:
        print(pos)
        # rots = pos//4096
        # pos_rel = pos%4096
        # scale = TIMESCALES_CUMU[rots]
        # if last_scale is not None and last_scale != scale:
        #     # t_start += last_scale
        #     t_start = time.time()
        # total_time = pos_rel/4096*scale

        del_t = 0
        # wait until there would be a move command large
        # enough to be noticeable, 1/60th of a rotation
        THRESHOLD = .1
        # while del_t < scale/60*THRESHOLD:
        # del2_pos = 0
        # del2_t = 0
        # while del2_pos < 4096/60:

        # while del_t < 1:
        while time.time()-t < 1:
        # while del2_t < 1:
            continue
            # del_t = time.time()-t_start
            # del2_t = _del_t - del_t

            # _del_pos = int(del_t/scale*4096)
            # del2_pos = _del_pos-del_pos
            # print(del2_pos)
        t = time.time()
        del_t = time.time() - t_start
        # del_t = _del_t
        # del_pos = _del_pos
        # del_pos = 0
        # for i in range(rots+1):
        # for i,scale in enumerate(TIMESCALES_CUMU[:rots+1]):
        #     del_pos += int(del_t/TIMESCALES_CUMU[i]*4096  )
        # pos_t = pos_start - del_pos
        t_left = max(t_total-del_t, 0)
        pos_t = time2pos(t_left)
        pos_t = max(2, pos_t)
        print(del_t, pos, del_pos, pos_t)
        
        # testMotor.set_torque_enable(True)
        while not testMotor.set_goal_position(pos_t):
            time.sleep(0.1)
            continue
        # testMotor.set_torque_enable(False)
        pos = safe_read(
            "present_position",
            )
        listen_write(pos)
    while not testMotor.set_torque_enable(False):
        time.sleep(0.1)

# import functools
# disable = partial(testMotor.set_torque_enable(False))
# enable = partial(testMotor.set_torque_enable(True))
def disable():
    testMotor.set_torque_enable(False)
def enable():
    testMotor.set_torque_enable(True)


def loop():
    while True:
        listen_write()
        time.sleep(0.1)
 
breath_mult = 0.5
BREATH_MIN, BREATH_MAX, BREATH_PERIOD = 0.5, 1.5, 2
def breathe():
    global breath_mult
    
    # i = 0
    while True:
        # i %= 10
        # for n,_np in enumerate(np):
        #     np[n] = tuple([int(_n*(i+1)) for _n in _np])
        # # print([n for n in np])
        # np.write()
        # i += 1
        for i in range(10):
            breath_mult += 0.2
            time.sleep(0.1)
        for i in range(10):
            breath_mult -= 0.2
            time.sleep(0.1)
        # breath_mult = (breath_mult%10)+1

if __name__=="__main__":
    reset_motor()
    time.sleep(1)
    testMotor.set_torque_enable(False)
    time.sleep(1)
    # breathe_thread = _thread.start_new_thread(breathe, ())
    loop()
   
# if __name__ == '__main__':
    # single_motor_example()
