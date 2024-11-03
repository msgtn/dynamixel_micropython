
TIMESCALES = [
    60,         # seconds-minutes
    60,         # minutes-hours
    24,         # hours-days
    7,          # days-weeks
    4,          # weeks-months
    12,         # months-years
    10,         # years-decade
    7.4,        # decades-life
]
TIMESCALES_CUMU = [TIMESCALES[0]]
for timescale in TIMESCALES[1:]:
    TIMESCALES_CUMU.append(TIMESCALES_CUMU[-1]*timescale)

print(TIMESCALES_CUMU)

TIMESCALES_CUMU = [
    60,
    # 120,
    3600,
    86400,
    604800,
    2419200,
    29030400,
    290304000,
    2148249600
    ]
print(TIMESCALES_CUMU)
