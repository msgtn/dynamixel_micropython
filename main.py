import os
import time
from dynamixel_python import DynamixelManager

# from itertools import accumulate
import _thread
import asyncio
from math import ceil
import gc

gc.enable()
import network

# import ntptime

# with open("./.network_config.txt", "r") as _file:
#     [ssid, key] = _file.read().split("\n")[:2]

# wifi = network.WLAN(network.STA_IF)
# wifi.active(True)
# wifi.connect(ssid, key)
# ntptime.settime()
TIMEZONE = 0


# OFFSET = TIMEZONE * 60 * 60
def set_offset(timezone):
    global OFFSET
    OFFSET = timezone * 60 * 60


set_offset(TIMEZONE)


from neopixel_12 import np12, NP12_COUNT, NP12_PIN, COLORS, COLOR_TUPLES
from neopixel_12 import np24, NP24_COUNT, NP24_PIN, COLORS, COLOR_TUPLES

INVERT_NEOPIXELS = False
INVERT_NEOPIXELS = True

USB_PORT = "/dev/tty.usbmodem101"


BAUDRATE = 9600
DYNAMIXEL_MODEL = "xl330-m288"
ID = 1

POSITION_MIN, POSITION_MAX = 0, 2**32
VALUE_MIN, VALUE_MAX = 0, 2**32

DEGREE_RESET_THRESHOLD = 5
POSITION_RESET_THRESHOLD = 4096 / 360 * DEGREE_RESET_THRESHOLD
VALUE_RESET_THRESHOLD = POSITION_RESET_THRESHOLD
POSITION_RESET = 0
# POSITION_RESET = VALUE_MAX - POSITION_RESET

TIMESCALES = [
    60,  # seconds-minutes
    60,  # minutes-hours
    24,  # hours-days
    7,  # days-weeks
    4,  # weeks-months
    12,  # months-years
    10,  # years-decade
    7.4,  # decades-life
]
TIMESCALES_CUMU = [TIMESCALES[0]]
for timescale in TIMESCALES[1:]:
    TIMESCALES_CUMU.append(TIMESCALES_CUMU[-1] * timescale)


"""
turn on a single dynamixel and sweep it between position 0 and position 1024 three times
"""
motors = DynamixelManager(USB_PORT, baud_rate=BAUDRATE)
motor = motors.add_dynamixel("motor", ID, DYNAMIXEL_MODEL)
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
    attempts = 4
    attempt_ctr = 0
    while attempts > 0:
        # while True:
        # print("reading")
        # print(f"attempts: {attempt_ctr}")
        try:
            ret = io_attr_fn(*args, **kwargs)
            if _operation == "set" and ret:
                break
            elif _operation == "get":
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


def safe_get(attr, attempts=float("inf"), *args, **kwargs):
    return safe_io(attr, "get", *args, **kwargs)


DEFAULT_NETWORK_CONFIG_FILE = "./.network_config.txt"
NETWORK_CONFIGS = "./.network_configs"
wifi = None


def connect_wifi(network_configs=NETWORK_CONFIGS):
    global wifi, TIMEZONE, OFFSET
    ssid = key = None
    for network_config_file in os.listdir(network_configs):
        try:
            _filepath = f"{network_configs}/{network_config_file}"
            with open(_filepath, "r") as _file:
                output = _file.read()
                print(output)
                # [ssid, key, TIMEZONE] = _file.read().split("\n")[:3]
                [ssid, key, TIMEZONE] = output.split("\n")[:3]
            print(f"SSID: {ssid}, key: {key}, timezone: {TIMEZONE}")

            wifi = network.WLAN(network.STA_IF)
            wifi.active(True)
            attempt_ctr = 10
            while attempt_ctr > 0 and not wifi.isconnected():
                wifi.connect(ssid, key)
                attempt_ctr -= 1

            import ntptime

            ntptime.settime()
            set_offset(int(TIMEZONE))

            break

        except:
            if ssid is not None:
                print(
                    f"Could not connect to network '{ssid}', are the credentials correct?"
                )
            elif network_config_file not in os.listdir(network_configs):
                print(
                    "Could not connect to wifi, is '{network_config_file}' formatted as '\{ssid\}\\n\{key\}'?"
                )
            else:
                print(
                    "Could not connect to wifi, could not find network config file {network_config_file}"
                )

    if wifi is None:
        return False
    return wifi.isconnected()


TIME_TUPLE_INDEX = ["year", "month", "day", "hour", "minute", "second", "millisecond"]


def time2led(*args, **kwargs):
    global state
    if state not in [State.Idle, State.Alarm]:
        return
    index_hour, index_minute = time2index()
    index2led(index_hour, index_minute, *args, **kwargs)


def index2led(index_hour, index_minute, colors=(0, 0, 0, 1), write_fn=None):
    for np, i in zip([np12, np24], [index_hour, index_minute]):
        if write_fn is None:
            np.fill((0, 0, 0, 0))
            np[_idx(i)] = colors
            np.write()
        else:
            new_vals = [(0, 0, 0, 0)] * len(np)
            new_vals[i] = colors
            write_fn(np, new_vals)


def _idx(i):
    return (not INVERT_NEOPIXELS) * i - INVERT_NEOPIXELS * (i)


def time2index(hour=None, minute=None):
    if hour is None or minute is None:
        hour, minute = get_time()
    hour %= 12
    index_hour = hour
    index_minute = int(minute // 2.5)
    return index_hour, index_minute


def get_time():
    global wifi

    if wifi is None:
        print("Wifi not initialized, time is not accurate")
        return -1, -1
    # while not wifi.isconnected():
    #     print("Wifi not connected, retrying")
    #     connect_wifi()

    #     time.sleep(0.1)

    t = time.localtime(time.time() + OFFSET)
    # t = time.localtime(time.time() + 0)
    return t[TIME_TUPLE_INDEX.index("hour")], t[TIME_TUPLE_INDEX.index("minute")]


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


t_start = t_last = time.time()
t_alarm = 0
last_moved = 0


def enum(**enums: int):
    return type("Enum", (), enums)


State = enum(Idle=1, Stopwatch=2, Timer=3, Alert=4, Alarm=5)
state = State.Idle


def _get_position(**kwargs):
    global position
    position = safe_get("present_position", **kwargs)
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
    position = _get_position()
    # disable()
    # time.sleep(0.1)
    motor.set_led(True)

    # while not motor.get_torque_enable():
    # motor.set_torque_enable(True)f
    time.sleep(0.1)
    enable()
    time.sleep(0.1)
    # time.sleep(0.5)
    # motor.set_profile_velocity(262)
    safe_set("profile_velocity", 262)
    # motor.set_goal_position(POSITION_RESET)
    while not at_reset():
        position = _get_position()
        if position > 2**16:
            safe_set("goal_position", VALUE_MAX)
        else:
            safe_set("goal_position", VALUE_MIN)

        # safe_set("goal_position", POSITION_RESET)

        print("Not at reset")

        time.sleep(0.1)

    disable()
    time.sleep(0.1)

    motor.set_led(False)


def global_position(fn):
    def _fn(*args, **kwargs):
        global position
        while position is None:
            position = _get_position()
        # if position is None:
        #     # # position = get_position()
        # return fn(position=position, *args, **kwargs)
        return fn(position=position)

    return _fn


@global_position
def position2rots_rel(position=None):
    rots = position // 4096
    rots %= len(TIMESCALES_CUMU)
    rel_pos = position % 4096
    return rots, rel_pos


# def listen_write(pos=None):
@global_position
def position2led(position=None, np=np12, *args, **kwargs):
    # def position2led(position=None, np=np24, *args, **kwargs):
    global breath_mult
    np_count = len(np)
    rots, rel_pos = position2rots_rel(position)
    clock_pos = rel_pos / 4096 * np_count
    rots %= len(TIMESCALES_CUMU)
    total_time = TIMESCALES_CUMU[rots] * (rel_pos / 4096)
    if rots > 0:
        total_time = max(TIMESCALES_CUMU[rots - 1], total_time)
    if clock_pos < 1:
        if rots > 0 or clock_pos > 0.1:
            clock_pos = 1
        elif clock_pos < 0.1:
            clock_pos = 0
    else:
        clock_pos = round(clock_pos)
    color_tuple = COLOR_TUPLES[rots % len(COLOR_TUPLES)]
    color_tuple = tuple([int(i * breath_mult) for i in color_tuple])
    print(f"{clock_pos: 05d}")
    for i in range(clock_pos):
        # np12[i] = (1,0,0,0)
        np[_idx(i)] = color_tuple
    for j in range(clock_pos, np_count):
        np[_idx(j)] = (0, 0, 0, 0)
    position2led_value(position)
    np.write()


@global_position
def position2led_value(position=None, np=np24, *args, **kwargs):
    [digit_1, digit_2] = position2digits(position)
    led_1 = digit_1 * 2
    led_2 = digit_2 * 2 + 1
    np.fill((0, 0, 0, 0))
    if digit_1 > 0:
        np[_idx(led_1)] = (1, 0, 0, 0)
    if not at_reset():
        np[_idx(led_2)] = (0, 0, 1, 0)
    np.write()


@global_position
def position2time(position=None):
    rots, rel_pos = position2rots_rel(position)
    norm_pos = rel_pos / 4096
    scale = TIMESCALES_CUMU[min(rots, len(TIMESCALES_CUMU) - 1)]
    ret = norm_pos * scale
    return ret


@global_position
def position2value(position=None):
    # convert a position to its mapped value on the given timescale
    rots, rel_pos = position2rots_rel(position)
    norm_pos = rel_pos / 4096
    ret = ceil(norm_pos * TIMESCALES[rots])
    return ret


@global_position
def position2digits(position=None):
    value = position2value(position)
    ret = f"{value:02d}"
    print(ret)
    ret = [int(i) for i in ret]
    return ret


def time2position(t):
    rots = 0
    while t > TIMESCALES_CUMU[rots]:
        rots += 1
    scale = TIMESCALES_CUMU[rots]
    pos = rots * 4096
    pos += (t / scale) * 4096
    # pos = POSITION_MAX - pos
    return int(pos)


def clock2leds():
    pass


#
def tick():
    global position
    t_start = t = time.time()
    # pos_start = motor.get_present_position()
    # pos_start = position = safe_get("present_position", )
    pos_start = position
    # Calculate the set time in seconds
    # total_time = pos_start/4096*60
    t_total = t_left = position2time(pos_start)
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
        THRESHOLD = 0.1
        # while del_t < scale/60*THRESHOLD:
        # del2_pos = 0
        # del2_t = 0
        # while del2_pos < 4096/60:

        # while del_t < 1:
        while time.time() - t < 1:
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
        t_left = max(t_total - del_t, 0)
        pos_t = time2position(t_left)
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
    # if time.time()-t_last < 1:
    #    return t_last
    while time.time() - t_last < 1:
        continue
    t = time.time() - t_start

    if state == State.Timer:
        t = max(t_total - t, 0)
    position_t = time2position(t)
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
        # for n,_np in enumerate(np12):
        #     np12[n] = tuple([int(_n*(i+1)) for _n in _np])
        # # print([n for n in np12])
        # np12.write()
        # i += 1
        for i in range(10):
            breath_mult += 0.2
            time.sleep(0.1)
        for i in range(10):
            breath_mult -= 0.2
            time.sleep(0.1)
        # breath_mult = (breath_mult%10)+1


AMPLITUDE_WAG = 400
POSITION_WAG = AMPLITUDE_WAG


async def _wag(n_wag=2, delay=0.5):
    global ENABLED
    ENABLED = True
    _reset_motor()
    time.sleep(delay)
    for _ in range(n_wag):
        for i in [1, -1]:
            safe_set("goal_position", i * AMPLITUDE_WAG)
            time.sleep(delay)

    safe_set("goal_position", 0)
    time.sleep(delay)
    disable()
    ENABLED = False


async def wag(delay=0.1):
    global POSITION_WAG, ENABLED

    ENABLED = True
    safe_set("goal_position", POSITION_WAG)
    time.sleep(delay)
    disable()
    ENABLED = False

    POSITION_WAG *= -1
    # POSITION_WAG += AMPLITUDE_WAG


def at_reset():
    global position
    # position = _get_position()
    return close_to_end(position)


def close_to_end(
    x,
):
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
    t_total = position2time(position)


async def set_alert():
    global state, t_start
    state = State.Alert


async def set_alarm():
    global state, position, t_alarm
    state = State.Alarm
    position = _get_position()
    t_alarm = position2alarm(position)


ALARM_TIME = 0


@global_position
def position2alarm(position=None):
    _, rel_pos = position2rots_rel(position)
    rots = position // 4096

    # calculate which rotation this position is at
    # max_rots = max_position / steps_per_rot
    # max_rots = 2**32 / 4096 = 2**32 / 2**12
    # max_rots = 2**20
    del_rots = 2**20 - rots
    print(f"del rots {del_rots}")

    # if outside of 2 or 3 revolutions away,
    # end timer mode
    if del_rots not in [2, 3]:
        return -1

    hour, minute = rel_pos2hour_min(rel_pos)
    # by default, hour will be in range [0, 12]
    # i.e. AM timer
    # if within the third revolution, PM timer,
    # so add 12
    colors = (1, 0, 0, 0)
    if del_rots == 3:
        hour += 12
        colors = (0, 1, 0, 0)

    t_alarm = get_time_alarm(hour, minute)

    index_hour, index_minute = time2index(hour, minute)
    index2led(index_hour, index_minute, colors, _merge_write)

    return t_alarm


def get_time_alarm(hour, minute):
    t_now = time.time()
    local_now = time.localtime(t_now + OFFSET)
    print(f"local now: {local_now}")
    for del_day in [0, 1]:
        local_alarm = list(local_now)
        local_alarm[2] = local_alarm[2] + del_day
        local_alarm[3] = hour
        local_alarm[4] = minute
        local_alarm[5] = 0
        # for i in range(5, len(local_alarm)):
        #     local_alarm[i] = 0
        local_alarm = tuple(local_alarm)
        t_alarm = time.mktime(local_alarm)

        # if alarm time is earlier than current time,
        # add another day
        print(t_alarm, t_now)
        print(local_alarm, local_now)
        if t_alarm > t_now:
            return t_alarm


def rel_pos2hour_min(rel_pos):
    _time = rel_pos / 4096 * 12
    hour = int(_time)
    minute = int((_time % 1) * 60)
    return hour, minute


async def set_stopwatch():
    global state, t_start, t_last, t_total
    _reset_motor()
    enable()
    state = State.Stopwatch
    time.sleep(0.1)
    t_total = position2time(position)


def get_state():
    global position
    if position < POSITION_MAX // 2:
        return State.Timer
    else:
        # If within the first rotation, stopwatch
        del_pos = VALUE_MAX - position
        rot_range = 4096
        if del_pos < 4096:

            return State.Stopwatch
        elif del_pos < 2 * 4096:
            return State.Alarm


async def handle_state():
    global state, position, last_moved
    global t_start, t_last, t_total, t_alarm

    while True:
        print(f"State: {state}")
        if state == State.Idle:
            # if not at_reset() and at_rest():
            #     t_start = t_last = time.time()
            #     if get_state() == State.Timer:
            #         await set_timer()
            #     else:
            #         await set_stopwatch()
            # elif time.time() - last_moved > 2:
            #     time2led()
            # else:
            #     position2led()

            if not at_reset():
                if at_rest():
                    t_start = t_last = time.time()
                    new_state = get_state()
                    if new_state == State.Timer:
                        await set_timer()
                    elif new_state == State.Stopwatch:
                        await set_stopwatch()
                    else:
                        await set_alarm()
                else:
                    position2led()
            else:
                time2led()
            # If not at reset position
            # and last moved 2(?) seconds ago,
            # transition to Timer
            pass
        elif state == State.Timer:
            t_last = await _tick(t_start, t_last, t_total)
            print("TIMER", t_start, t_last, t_total)
            # tick()
            # if at_reset() or last_moved > t_start:
            if last_moved > t_start:
                _reset_motor()
                await set_idle()

            elif at_reset():

                _reset_motor()
                # await set_idle()

                await set_alert()

        elif state == State.Stopwatch:
            t_last = await _tick(t_start, t_last, t_total)
            if last_moved > t_start:
                _reset_motor()
                await set_idle()
        elif state == State.Alert:
            await wag()
            if last_moved > t_start:
                _reset_motor()
                await set_idle()
        # await asyncio.sleep_ms(10)
        elif state == State.Alarm:
            position = _get_position()

            _t_alarm = position2alarm(position)
            if _t_alarm == -1:
                _reset_motor()
                await set_idle()
            else:
                time2led()
                print(f"state check, {time.time()}, {t_alarm}")
                if time.time() >= t_alarm:
                    _reset_motor()
                    await set_alert()
                else:
                    t_alarm = _t_alarm
                    print(f"Set alarm for {time.localtime(t_alarm)}")
        await asyncio.sleep_ms(1)


def _merge_write(np, new_vals):
    for n, new_val in enumerate(new_vals):
        np_n = list(np[n])
        for i, np_new in enumerate(new_val):
            np_n[i] = max(np_n[i], np_new)
        np[n] = tuple(np_n)
    np.write()


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
            if not ENABLED:

                present_velocity = safe_get("present_velocity")
                # if present_velocity is not None and present_velocity > 0 and not ENABLED:
                if present_velocity is not None and not close_to_end(present_velocity):
                    last_moved = time.time()
                    print("VELOCITY", present_velocity)
        except:
            pass

        # await asyncio.sleep_ms(10)
        await asyncio.sleep_ms(1)
        # time.sleep(0.1)


async def loop_pos2led():
    global state
    while True:
        # listen_write()
        # if state is not State.Idle and state is not State.Alert and:
        if state in [State.Timer, State.Stopwatch]:
            position2led()
        await asyncio.sleep_ms(1)
        # await asyncio.sleep_ms(10)
        # time.sleep(0.1)


def main():
    disable()
    time.sleep(0.1)
    safe_set("operating_mode", 4)
    time.sleep(0.1)
    safe_set("profile_velocity", 262)
    time.sleep(0.1)
    enable()
    # safe_set("goal_position", 0)
    # while not at

    time.sleep(1)
    # safe_set("drive_mode", 8)
    # DRIVE MODE
    # [torque enable when goal, something, unused, reverse direction]
    safe_set("drive_mode", 9)
    time.sleep(0.1)

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
            loop_pos2led(),
            handle_state(),
            get_position(),
        )

    asyncio.run(_main())
    # loop()


if __name__ == "__main__":
    connect_wifi()
    #     break
    main()
# if __name__ == '__main__':
# single_motor_example()
