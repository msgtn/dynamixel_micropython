import time
from collections import OrderedDict

import machine
import neopixel

NP12_PIN = 2
NP24_PIN = 4
NP12_COUNT = 12
NP24_COUNT = 24
np12 = neopixel.NeoPixel(machine.Pin(NP12_PIN), NP12_COUNT, bpp=4)
np24 = neopixel.NeoPixel(machine.Pin(NP24_PIN), NP24_COUNT, bpp=4)
COLORS = OrderedDict()
COLORS.update({'R': (255, 0, 0, 0)})
COLORS.update({'O': (255, 127, 0, 0)})
COLORS.update({'Y': (255, 255, 0, 0)})
# COLORS.update({   'spring_green':(127,255,0,0)})
COLORS.update({'G': (0, 255, 0, 0)})
# COLORS.update({   'turquoise':(0,255,127,0)})
COLORS.update({'cyan': (0, 255, 255, 0)})
# COLORS.update({   'ocean':(0,127,255,0)})
COLORS.update({'B': (0, 0, 255, 0)})
COLORS.update({'I': (75, 0, 130, 0)})
# COLORS.update({   'violet':(127,0,255,0)})
# COLORS.update({   'V':(238,130,238,0)})
COLORS.update({'white': (255, 255, 255, 0)})
# COLORS.update({   'magenta':(255,0,255,0)})
# COLORS.update({   'raspberry':(255,0,127,0)})
# 'I':(75,0,130,0),
# 'V':(148,0,211,0),
COLOR_TUPLES = list(COLORS.values())
COLOR_TUPLES = [tuple([t//16 for t in tup]) for tup in COLOR_TUPLES]


def red_ring(delay=0.05, scale=1):
    for i in range(2):
        for j in range(NP12_COUNT):
            np12[j] = (i*scale, 0, 0, 0)
            np12.write()
            time.sleep(delay)


def rainbow(delay=0.05):
    pass


def write_all(tup):
    for j in range(NP12_COUNT):
        np12[j] = tup
    np12.write()


def write_to(tup_0, tup_1, steps=10, delay=0.05):
    print(tup_0, tup_1)
    del_tup = tuple([t1-t0 for t1, t0 in zip(tup_1, tup_0)])
    for i in range(steps):
        tup_i = tuple([t0+int(i/steps*dt) for t0, dt in zip(tup_0, del_tup)])
        # print(tup_i)
        np12.fill(tup_i)
        np12.write()
        time.sleep(delay)


if __name__ == "__main__":
    np12.fill(COLORS['R'])
    np12.write()
    STEPS = 10
    DELAY = 0.01

    while True:
        # red_ring()
        for tup_start, tup_goal in zip(COLOR_TUPLES[:-1], COLOR_TUPLES[1:]):
            write_to(tup_start, tup_goal, steps=STEPS, delay=DELAY)

        write_to(COLOR_TUPLES[-1], COLOR_TUPLES[0], steps=STEPS, delay=DELAY)
