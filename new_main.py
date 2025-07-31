import asyncio
import gc
import logging
import os
import time
from math import ceil

import network

from dynamixel_python import DynamixelManager
from neopixel_12 import COLOR_TUPLES, np12, np24

logging.basicConfig(level=logging.WARNING)

gc.enable()

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
MOVE_THRESHOLD = 2
AMPLITUDE_WAG = 400
POSITION_WAG = AMPLITUDE_WAG
# POSITION_RESET = VALUE_MAX - POSITION_RESET

TIMESCALES = [
    60,  # seconds-minutes
    60,  # minutes-hours
    24,  # hours-days
    7,  # days-weeks
    4,  # weeks-months
    12,  # months-years
    84,  # years-life
    # 10,  # years-decade
    # 8.4,  # decades-life
]
TIMESCALES_CUMU = [TIMESCALES[0]]
for timescale in TIMESCALES[1:]:
    TIMESCALES_CUMU.append(TIMESCALES_CUMU[-1] * timescale)

TIME_TUPLE_INDEX = ["year", "month", "day", "hour", "minute", "second", "millisecond"]


DEFAULT_NETWORK_CONFIG_FILE = "./.network_config.txt"
NETWORK_CONFIGS = "./.network_configs"
wifi = None

LED_OFFSET = 6


def enum(**enums: int):
    return type("Enum", (), enums)


State = enum(Idle=1, Stopwatch=2, Timer=3, Alert=4, Alarm=5)
state = State.Idle


def connect_wifi(network_configs=NETWORK_CONFIGS):
    global wifi, TIMEZONE, OFFSET
    ssid = key = None
    for network_config_file in os.listdir(network_configs):
        try:
            _filepath = f"{network_configs}/{network_config_file}"
            with open(_filepath, "r") as _file:
                output = _file.read()
                logging.warning(output)
                # [ssid, key, TIMEZONE] = _file.read().split("\n")[:3]
                [ssid, key, TIMEZONE] = output.split("\n")[:3]
            logging.warning(f"SSID: {ssid}, key: {key}, timezone: {TIMEZONE}")

            wifi = network.WLAN(network.STA_IF)
            wifi.active(True)
            attempt_ctr = 3
            while attempt_ctr > 0 and not wifi.isconnected():
                print("Trying to connect")
                wifi.connect(ssid, key)
                attempt_ctr -= 1

            import ntptime

            ntptime.settime()
            set_offset(int(TIMEZONE))

            break

        except:
            if ssid is not None:
                logging.warning(
                    f"Could not connect to network '{ssid}', are the credentials correct?"
                )
            elif network_config_file not in os.listdir(network_configs):
                logging.warning(
                    "Could not connect to wifi, is '{network_config_file}' formatted as '\{ssid\}\\n\{key\}'?"
                )
            else:
                logging.warning(
                    "Could not connect to wifi, could not find network config file {network_config_file}"
                )

    if wifi is None:
        return False
    return wifi.isconnected()


def np_clear(np):
    np.fill((0, 0, 0, 0))
    np.write()


def np_offset(np, offset):
    return np
    new_np = []

    for i in range(offset, len(np)):
        new_np.append(np[i])
    for i in range(0, offset):
        new_np.append(np[i])
    for i, row in enumerate(new_np):
        np[i] = row

    # return np


def index2led(
    index_hour, index_minute, colors=(0, 0, 0, 127), write_fn=None, write=True
):
    for np, i in zip([np12, np24], [index_hour, index_minute]):
        if write_fn is None:
            # np.fill((0, 0, 0, 0))
            np[_idx(i)] = colors
            if write:
                np_offset(np, int(len(np) / 2))
                np.write()
        else:
            new_vals = [(0, 0, 0, 0)] * len(np)
            new_vals[i] = colors
            write_fn(np, new_vals)


def _idx(i):
    # i += LED_OFFSET
    ret = (not INVERT_NEOPIXELS) * i - INVERT_NEOPIXELS * (i)
    # ret %= (11)
    return ret


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
        logging.warning("Wifi not initialized, time is not accurate")
        return -1, -1

    t = time.localtime(time.time() + OFFSET)
    # t = time.localtime(time.time() + 0)
    return t[TIME_TUPLE_INDEX.index("hour")], t[TIME_TUPLE_INDEX.index("minute")]


t_start = t_last = time.time()
t_alarm = 0
last_moved = 0


@global_position
def rots_relpos_from_position(position=None):
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
    rots, rel_pos = rots_relpos_from_position(position)
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
    logging.debug(f"Clock position: {clock_pos:032d}")
    for i in range(clock_pos):
        np[_idx(i)] = color_tuple
    for j in range(clock_pos, np_count):
        np[_idx(j)] = (0, 0, 0, 0)
    position2led_value(position)
    np_offset(np, int(len(np) / 2))
    np.write()


@global_position
def position2led_value(position=None, np=np24, *args, **kwargs):
    [digit_1, digit_2] = position2digits(position)
    led_1 = digit_1 * 2
    led_2 = digit_2 * 2 + 1
    np.fill((0, 0, 0, 0))
    if digit_1 > 0:
        np[_idx(led_1)] = (32, 0, 0, 0)
    if not at_reset():
        np[_idx(led_2)] = (0, 0, 32, 0)
    np_offset(np, int(len(np) / 2))
    np.write()


@global_position
def position2value(position=None):
    # convert a position to its mapped value on the given timescale
    rots, rel_pos = rots_relpos_from_position(position)
    norm_pos = rel_pos / 4096
    # Special case: if in decades-life, show age instead
    if rots == 6:
        norm_pos = 1 - norm_pos
        logging.warning("In decades-life, inverting to show age")
    ret = ceil(norm_pos * TIMESCALES[rots])
    return ret


@global_position
def position2digits(position=None):
    value = position2value(position)
    ret = f"{value:02d}"
    logging.warning(ret)
    ret = [int(i) for i in ret]
    return ret


position = _get_position()
last_rot, _ = rots_relpos_from_position(position)


async def reset_motor():
    await _reset_motor()


def _reset_motor():
    global position, ENABLED
    position = _get_position()
    # disable()
    # time.sleep(0.1)
    motor.set_led(True)

    # while not motor.get_torque_enable():
    # motor.set_torque_enable(True)f
    time.sleep(0.1)
    enable()
    ENABLED = True
    time.sleep(0.1)
    # time.sleep(0.5)
    # motor.set_profile_velocity(262)
    safe_set("profile_velocity", 262)
    # motor.set_goal_position(POSITION_RESET)
    while not at_reset():
        position = _get_position()
        # if position > 2**16:
        if position > POSITION_MAX // 2:
            safe_set("goal_position", VALUE_MAX)
        else:
            safe_set("goal_position", VALUE_MIN)

        # safe_set("goal_position", POSITION_RESET)

        logging.warning("Not at reset")

        time.sleep(0.1)

    disable()
    ENABLED = False
    time.sleep(0.1)

    motor.set_led(False)


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
        logging.warning(position)
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
            # logging.warning(del2_pos)
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
        logging.warning(f"{t_left}, {del_t}, {position}, {pos_t}")

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
    while time.time() - t_last < 1:
        continue
    t = time.time() - t_start

    if state == State.Timer:
        t = max(t_total - t, 0)
    position_t = time2position(t)
    position_t = min(max(position_t, POSITION_MIN), POSITION_MAX)
    logging.warning(f"{t_total}, {t}, {position}, {position_t}")
    ENABLED = True
    while not safe_set("goal_position", position_t):
        time.sleep(0.1)
    time.sleep(0.1)

    disable()
    time.sleep(0.1)
    return time.time()


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
        # # logging.warning([n for n in np12])
        # np12.write()
        # i += 1
        for i in range(10):
            breath_mult += 0.2
            time.sleep(0.1)
        for i in range(10):
            breath_mult -= 0.2
            time.sleep(0.1)
        # breath_mult = (breath_mult%10)+1


OFFSET_WAG = 100


async def wag(delay=0.1):
    global POSITION_WAG, ENABLED

    ENABLED = True
    safe_set("goal_position", POSITION_WAG + OFFSET_WAG)
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
    del_x = min(abs(VALUE_MAX - x), abs(VALUE_MIN - x))
    return del_x < VALUE_RESET_THRESHOLD


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
    position = _get_position()
    t_total = position2time(position)

    logging.warning(f"Starting timer with {t_total}s")


async def set_alert():
    global state, t_start
    state = State.Alert


async def set_alarm():
    global state, position, t_alarm
    state = State.Alarm
    position = _get_position()
    t_alarm = position2alarm(position)
    t_last = time.time()


ALARM_TIME = 0


@global_position
def position2alarm(position=None):
    _, rel_pos = rots_relpos_from_position(position)
    rots = position // 4096

    # calculate which rotation this position is at
    # max_rots = max_position / steps_per_rot
    # max_rots = 2**32 / 4096 = 2**32 / 2**12
    # max_rots = 2**20
    del_rots = 2**20 - rots
    logging.warning(f"del rots {del_rots}")

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
    t_now = time.time() + OFFSET
    local_now = time.localtime(t_now)
    logging.warning(f"local now: {local_now}")
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
        logging.warning(f"{t_alarm}, {t_now}")
        logging.warning(f"{local_alarm}, {local_now}")
        if t_alarm >= t_now:
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
    global state, position, last_moved, last_rot
    global t_start, t_last, t_total, t_alarm
    global ENABLED

    while True:
        logging.debug(f"State: {state}")
        if state == State.Idle:
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
            # tick()
            # if at_reset() or last_moved > t_start:
            if last_moved > t_start:
                if ENABLED:
                    disable()
                if at_rest():
                    position = _get_position()
                    if (POSITION_MAX // 2) < position < POSITION_MAX:
                        _reset_motor()
                        await set_idle()
                    else:
                        t_start = t_last = time.time()
                        await set_timer()
                # if cur_rot == last_rot:
                #     t_start = t_last = time.time()
                #     await set_timer()
                # else:
                #     _reset_motor()
                #     await set_idle()

            elif at_reset():
                _reset_motor()
                # await set_idle()

                await set_alert()
            else:
                t_last = await _tick(t_start, t_last, t_total)
                # if t_last == -1:
                #     _reset_motor()
                #     await set_idle()
                # else:
                logging.warning(f"TIMER, {t_start}, {t_last}, {t_total}")
                cur_rot, _ = rots_relpos_from_position(position)
                last_rot = cur_rot

        elif state == State.Stopwatch:
            t_last = await _tick(t_start, t_last, t_total)
            if last_moved > t_start:
                _reset_motor()
                await set_idle()
        elif state == State.Alert:
            await wag()
            if last_moved > t_start:
                logging.warning(f"Stopping because {last_moved} > {t_start}")
                _reset_motor()
                await set_idle()
        # await asyncio.sleep_ms(10)
        elif state == State.Alarm:
            position = _get_position()

            _t_alarm = position2alarm(position)
            _t_last = time.time() + OFFSET
            if _t_alarm == -1:
                _reset_motor()
                await set_idle()
            else:
                time2led(t_alarm=_t_alarm)
                logging.warning(f"state check, {_t_last}, {t_alarm}")
                # if _t_last >= t_alarm:
                # If within 10 seconds either side of the alarm, trigger
                if abs(_t_last - _t_alarm) < 10:
                    _reset_motor()
                    t_start = time.time()
                    await set_alert()
                elif _t_last > (t_last + 4):
                    t_alarm = _t_alarm
                    t_last = t_alarm
                    logging.warning(f"Set alarm for {time.localtime(t_alarm)}")
        await asyncio.sleep_ms(1)


def _merge_write(np, new_vals):
    for n, new_val in enumerate(new_vals):
        np_n = list(np[n])
        for i, np_new in enumerate(new_val):
            np_n[i] = max(np_n[i], np_new)
        np[n] = tuple(np_n)

    np_offset(np, int(len(np) / 2))

    np.write()


def moving_threshold(velocity):
    return close_to_end(velocity)


# def moving_thread():
async def moving_thread():
    global last_moved, mode
    global position
    global ENABLED
    while True:
        try:
            if not ENABLED:
                present_velocity = safe_get("present_velocity")
                # if present_velocity is not None and present_velocity > 0 and not ENABLED:
                if present_velocity is not None and not close_to_end(present_velocity):
                    last_moved = time.time()
                    logging.warning(f"VELOCITY, {present_velocity}")
        except:
            pass

        # await asyncio.sleep_ms(10)
        await asyncio.sleep_ms(1)


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


class Tokidoki:
    def __init__(self):
        self.motors = DynamixelManager(USB_PORT, baud_rate=BAUDRATE)
        self.motor = self.motors.add_dynamixel("motor", ID, DYNAMIXEL_MODEL)
        self.motors.init()
        self.np12 = np12
        self.np24 = np24
        self.state = State.Idle
        self._enabled = False
        self.rotations = 0
        self.position = 0
        self.rel_position = 0

    def _able(self, enable: bool):
        self.safe_set("led", enable)
        self.safe_set("torque_enable", enable)

    def disable(self):
        self._able(False)

    def enable(self):
        self._able(True)

    @property
    def enabled(self):
        self._enabled = self.safe_get("torque_enable")
        return self._enabled

    def safe_io(self, attr, _operation="get", *args, **kwargs):
        # logging.warning(attr, _operation, args, kwargs)
        if not hasattr(self.motor, f"{_operation}_{attr}"):
            return
        io_attr_fn = getattr(self.motor, f"{_operation}_{attr}")
        # read_attempts =
        ret = None
        # while read_attempts < attempts:
        attempts = 10
        attempts = 4
        attempt_ctr = 0
        while attempts > 0:
            # while True:
            # logging.warning("reading")
            # logging.warning(f"attempts: {attempt_ctr}")
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

    def safe_set(self, attr, *args, **kwargs):
        return self.safe_io(attr, "set", *args, **kwargs)

    def safe_get(self, attr, attempts=float("inf"), *args, **kwargs):
        return self.safe_io(attr, "get", *args, **kwargs)

    def time2led(self, t_alarm=None, clear=True, *args, **kwargs):
        if self.state not in [State.Idle, State.Alarm]:
            return
        index_hour, index_minute = time2index()
        np12_idxs, np24_idxs, colors = [index_hour], [index_minute], [(0, 0, 0, 16)]

        if t_alarm is not None:
            [alarm_hour, alarm_minute] = time.localtime(t_alarm)[3:5]
            [index_alarm_hour, index_alarm_minute] = time2index(
                alarm_hour, alarm_minute
            )

            alarm_colors = (16, 0, 0, 0) if alarm_hour < 12 else (0, 0, 16, 0)
            np12_idxs.append(index_alarm_hour)
            np24_idxs.append(index_alarm_minute)
            colors.append(alarm_colors)
            # index2led(index_alarm_hour, index_alarm_minute, colors=alarm_colors, write=False)
        # if clear:
        #     np_clear(np12)
        #     np_clear(np24)
        # index2led(index_hour, index_minute, write=True)
        # logging.warning(f"NP12: {np12_idxs}")
        # logging.warning(f"NP24: {np24_idxs}")
        for np, idxs in zip([self.np12, self.np24], [np12_idxs, np24_idxs]):
            # np_clear(np)
            np.fill((0, 0, 0, 0))
            for i, color in zip(idxs, colors):
                np_i = list(np[_idx(i)])
                np_i = tuple([n + c for n, c in zip(np_i, color)])
                np[_idx(i)] = np_i
            np_offset(np, int(len(np) / 2))
            np.write()

        # np12.write()
        # np24.write()

    def check_moved(self): ...

    async def tick(self, hz: int = 60):
        ts_start = time.time()
        period = 1 / hz
        while True:
            ts = time.time()
            await self.render(ts - ts_start)
            delta_ts = time.time() - ts
            if delta_ts < period:
                await asyncio.sleep(period - delta_ts)

    async def update_position(self, hz: 120):
        period = 1 / hz
        while True:
            self.position = self.safe_get("present_position")

            self.rotations = (self.position // 4096) % len(TIMESCALES_CUMU)
            self.rel_position = self.position % 4096
            await asyncio.sleep(period)

    async def render(self):
        match self.state:
            case State.Idle:
                # render time
                ...
            case State.Stopwatch:
                # render passed time
                ...
            case State.Timer:
                # render remaining time
                ...
            case State.Alert:
                # render flashing lights?
                ...
            case State.Alarm:
                # render time like State.Idle
                # but also alarm
                ...

    def position2time(
        self,
        position=None
    ):
        if position is None:
            position = self.position
        rots, rel_pos = rots_relpos_from_position(position)
        norm_pos = rel_pos / 4096
        scale = TIMESCALES_CUMU[min(rots, len(TIMESCALES_CUMU) - 1)]
        ret = norm_pos * scale
        return ret


def main():
    disable()
    time.sleep(0.1)
    td = Tokidoki()
    td.safe_set("operating_mode", 4)
    time.sleep(0.1)
    td.safe_set("profile_velocity", 262)
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

    connect_wifi()

    async def _main():
        await asyncio.gather(
            moving_thread(),
            loop_pos2led(),
            handle_state(),
            update_position(),
        )

    asyncio.run(_main())
    # loop()


if __name__ == "__main__":
    main()
