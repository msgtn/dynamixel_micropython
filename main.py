import time
from dynamixel_python import DynamixelManager
# from itertools import accumulate
import _thread
import asyncio
import gc
gc.enable()

from neopixel_12 import np, NP_COUNT, NP_PIN, COLORS, COLOR_TUPLES

USB_PORT = '/dev/tty.usbmodem101'

BAUDRATE = 9600
DYNAMIXEL_MODEL = 'xl330-m288'
ID = 1

POSITION_MIN, POSITION_MAX = 0, 2**32
VALUE_MIN, VALUE_MAX = 0, 2**32

POSITION_RESET_THRESHOLD = 20
VALUE_RESET_THRESHOLD = 20
POSITION_RESET = 10
# POSITION_RESET = VALUE_MAX - POSITION_RESET

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
"""
turn on a single dynamixel and sweep it between position 0 and position 1024 three times
"""
motors = DynamixelManager(USB_PORT, baud_rate=BAUDRATE)
motor = motors.add_dynamixel('motor', ID, DYNAMIXEL_MODEL)
motors.init()
# def safe_io(attr, _operation="get", attempts=float('inf'), *args, **kwargs):
def safe_io(attr, _operation="get", *args, **kwargs):
    print(attr, _operation, args, kwargs)
    if not hasattr(motor, f"{_operation}_{attr}"):
        return
    io_attr_fn = getattr(motor, f"{_operation}_{attr}")
    # read_attempts =
    ret = None
    # while read_attempts < attempts:
    attempts = 10
    attempt_ctr = 0
    while attempts > 0:
    # while True:
        # print("reading")
        # print(f"attempts: {attempt_ctr}")
        try:
            ret = io_attr_fn(*args, **kwargs)
            if _operation=="set" and ret:
                break
            elif _operation=="get":
                break
        except:
            # read_attempts += 1
           # time.sleep(0.05)
            time.sleep(0.1)
        attempts -= 1
        attempt_ctr += 1

    return ret


# def safe_get(attr, attempts=10):
def safe_set(attr, *args, **kwargs):
    return safe_io(attr, "set", *args, **kwargs)

def safe_get(attr, attempts=float('inf'), *args, **kwargs):
    return safe_io(attr, "get", *args, **kwargs)


ENABLED = False
def disable():
    _able(False)

def enable():
    _able(True)

def _able(_enable: bool):
    global ENABLED
    safe_set("led", _enable)
    safe_set("torque_enable", _enable)
    ENABLED = _enable


disable()
time.sleep(0.1)
safe_set("operating_mode", 4)
time.sleep(0.1)
safe_set("profile_velocity", 262)
time.sleep(0.1)
# safe_set("drive_mode", 8)
# DRIVE MODE
# [torque enable when goal, something, unused, reverse direction]
safe_set("drive_mode", 9)
time.sleep(0.1)

t_start = t_last = time.time()
last_moved = 0
def enum(**enums: int):
    return type('Enum', (), enums)
State = enum(Idle=1, Stopwatch=2, Timer=3)
state = State.Idle
def _get_position(**kwargs):
    global position
    position = safe_get('present_position', **kwargs)
    # position = POSITION_MAX - position
    return position
async def get_position():
    global position
    while True:
        position = _get_position()
        await asyncio.sleep_ms(1)
    # return  _get_position() 
position = _get_position()
async def reset_motor():
    await _reset_motor()

def _reset_motor():
    global position
    # disable()
    # time.sleep(0.1)
    motor.set_led(True)

    # while not motor.get_torque_enable():
    # motor.set_torque_enable(True)
    time.sleep(0.1)
    enable()
    time.sleep(0.1)
        # time.sleep(0.5) 
    # motor.set_profile_velocity(262)
    # motor.set_goal_position(POSITION_RESET)
    safe_set("goal_position", POSITION_RESET)
    while not at_reset():
        position = _get_position()
        
        print("Not at reset")

        time.sleep(0.1)

    disable()
    time.sleep(0.1)

    motor.set_led(False)




def global_position(fn):
    def _fn(*args, **kwargs):
        global position
        # if position is None:
        #     # # position = get_position()
        # return fn(position=position, *args, **kwargs)
        return fn(position=position)
    return _fn

@global_position
def position2rots_rel(position=None):
    rots = position//4096
    rel_pos = position%4096
    return rots, rel_pos
   

# def listen_write(pos=None):
@global_position
def position2led(position=None, *args, **kwargs):
    global breath_mult
    # read_attempts = 0
    # pos = None
    # while read_attempts < 10:
    #     try:
    #         pos = motor.get_present_position()
    #         break
    #     except:
    #         read_attempts += 1
    # if pos is None:
    #     return 
    # if position is None:
    #     position = safe_get("present_position", attempts=10)
    # if not position:
    #     return
    rots, rel_pos = position2rots_rel(position)
    # rots = position//4096
    # rel_pos = position%4096
    # clock_pos = int(pos/4096*NP_COUNT)
    # clock_pos = int((pos%4096)/4096*NP_COUNT)
    clock_pos = rel_pos/4096*NP_COUNT
    clock_pos = min(NP_COUNT, clock_pos)
    rots %= len(TIMESCALES_CUMU)
    # print(rots)
    total_time = TIMESCALES_CUMU[rots]*(rel_pos/4096)
    if rots > 0:
        total_time = max(TIMESCALES_CUMU[rots-1], total_time)
    # print(total_time)
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
   
@global_position
def pos2time(position=None):
    rots, rel_pos = position2rots_rel(position)
    norm_pos = rel_pos/4096
    scale = TIMESCALES_CUMU[min(rots, len(TIMESCALES_CUMU)-1)]
    ret = norm_pos*scale
    return ret

def time2pos(t):
    rots = 0 
    while t > TIMESCALES_CUMU[rots]:
        rots += 1
    scale = TIMESCALES_CUMU[rots]
    pos = rots*4096
    pos += (t/scale)*4096
    # pos = POSITION_MAX - pos
    return int(pos)
# 
def tick():
    global position
    t_start = t = time.time()
    # pos_start = motor.get_present_position()
    # pos_start = position = safe_get("present_position", )
    pos_start = position
    # Calculate the set time in seconds
    # total_time = pos_start/4096*60
    t_total = t_left = pos2time(pos_start)
    # get how much time to tick for
    # home position is 0
    # while safe_get("present_position") > 10:
    motor.set_operating_mode(4)
    while not motor.set_torque_enable(True):
        time.sleep(0.05)
        continue
    motor.set_profile_velocity(262)
    del_pos = del2_pos = 0
    del_t = 0
    last_scale = None
    # while pos and pos > 0:
    while position and t_left > 0:
        print(position)
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
        print(t_left, del_t, position, pos_t)
        
        # motor.set_torque_enable(True)
        while not motor.set_goal_position(pos_t):
            time.sleep(0.2)
            continue
        # motor.set_torque_enable(False)
        # position = safe_get(
        #     "present_position",
        # position
        # listen_write(position)
        # if t_left==0:
        #     break
    while not motor.set_torque_enable(False):
        time.sleep(0.1)

# atomic tick function
async def _tick(t_start, t_last, t_total):
    global position, ENABLED, state
    #if time.time()-t_last < 1:
    #    return t_last
    while time.time()-t_last < 1:
        continue
    t = time.time() - t_start

    if state == State.Timer:
        t = max(t_total - t, 0)
    position_t = time2pos(t)
    position_t = min(max(position_t, POSITION_MIN), POSITION_MAX)
    print(t_total, t, position, position_t)
    # while not motor.set_goal_position(position_t):
    # enable()
    time.sleep(0.1)
    ENABLED = True
    while not safe_set("goal_position", position_t):
        time.sleep(0.1)
    time.sleep(0.1)
    
    disable()
    time.sleep(0.1)
    return time.time()

# import functools
# disable = partial(motor.set_torque_enable(False))
# enable = partial(motor.set_torque_enable(True))
# def safe_write(attr, attempts=float('inf'))



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

def at_reset():
    global position
    # # position = get_position()
    return close_to_end(position)

def close_to_end(x,):
    print(x)
    del_x = min(abs(VALUE_MAX - x), abs(VALUE_MIN - x))
    return del_x < VALUE_RESET_THRESHOLD

MOVE_THRESHOLD = 2
def at_rest():
    global last_moved
    return time.time() - last_moved > MOVE_THRESHOLD

async def set_idle():
    global state
    disable()
    state = State.Idle

async def set_timer():
    global state, t_start, t_last, t_total, position
    enable()
    state = State.Timer
    t_total = pos2time(position)

async def set_stopwatch():
    global state, t_start, t_last, t_total
    _reset_motor()
    enable()
    state = State.Stopwatch
    time.sleep(0.1)
    t_total = pos2time(position)
    

def get_state():
    global position
    if position < POSITION_MAX//2:
        return State.Timer
    else:
        return State.Stopwatch
    

async def handle_state():
    global state, position, last_moved
    global t_start, t_last, t_total
    
    while True:
        print(f"State: {state}")
        if state == State.Idle:
            if not at_reset() and at_rest():
                t_start = t_last = time.time()
                if get_state() == State.Timer:
                    await set_timer()
                else:
                    await set_stopwatch()
            # If not at reset position
            # and last moved 2(?) seconds ago,
            # transition to Timer
            pass
        elif state == State.Timer:
            t_last = await _tick(t_start, t_last, t_total)
            # tick()
            if at_reset() or last_moved > t_start:
            # if at_reset() :
                _reset_motor()
                await set_idle()

            # if at reset position
            # transition to Idle
            pass
        elif state == State.Stopwatch:
            t_last = await _tick(t_start, t_last, t_total)
            if last_moved > t_start:
                _reset_motor()
                await set_idle()
        # await asyncio.sleep_ms(10)
        await asyncio.sleep_ms(1)



def moving_threshold(velocity):
    return close_to_end(velocity)

# def moving_thread():
async def moving_thread():
    global last_moved, mode
    global position
    global ENABLED
    while True:
        # position = get_position()
        # print("velocity")
        try:
            present_velocity = safe_get("present_velocity")
            # if present_velocity is not None and present_velocity > 0 and not ENABLED:
            if present_velocity is not None and not close_to_end(present_velocity) and not ENABLED:
                last_moved = time.time()
                
                print("VELOCITY", present_velocity)
        except:
            pass

        # await asyncio.sleep_ms(10)
        await asyncio.sleep_ms(1)
        # time.sleep(0.1)

async def loop_async():
    while True:
        # listen_write()
        position2led()
        await asyncio.sleep_ms(1)
        # await asyncio.sleep_ms(10)
        # time.sleep(0.1)

def main():
    _reset_motor()
    time.sleep(1)
    disable()
    time.sleep(1)
    # breathe_thread = _thread.start_new_thread(breathe, ())
    # _thread.start_new_thread(moving_thread, ())
    # async_loop = asyncio.get_event_loop()
    # async_loop.create_task(moving_thread())
    # async_loop.run_forever()
    
    async def _main():
        await asyncio.gather(
            moving_thread(),
            loop_async(),
            handle_state(),
            get_position(),
        )
    asyncio.run(_main())
    # loop()
   
# if __name__=="__main__":
#     break
#     main()
# if __name__ == '__main__':
    # single_motor_example()
