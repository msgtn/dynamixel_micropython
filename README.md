A MicroPython port of [rdiverdi's dynamixel_python](https://github.com/rdiverdi/dynamixel_python/tree/master) library, itself a wrapper over the [official Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK). Currently working with the Raspberry Pi Pico as a replacement for the [Dynamixel U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/), with the [XL330-M288 motors](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/).

# Installation
## Hardware
The pinout, with the female end of the Dynamixel 3-line cable pointing up and the probe points facing you, is `data, v_in, ground`.

Connect the data line to both the TX and RX lines of the desired UART on the board. On the Pico, UART 0 has these lines on GPIOs 0 and 1, the top-leftmost pins if the microUSB connector is facing up.


## Software
You will need to know the motor's model, baudrate, and ID. Apart from assuming that the settings are at the defaults, this information is available through the [Dynamixel Wizard tool](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/), though this requires a U2D2.

- Connect the board
- Enter [`rshell`](https://github.com/dhylands/rshell)
- Copy all of the top-level Python files to the board: `cp *.py /pyboard`
- Copy the single-motor example to the board as the main script: `cp single_motor.py /pyboard/main.py` 
- Create the directory for the control tables: `mkdir /pyboard/control_tables`
- Copy only the necessary control tables, e.g. `cp control_tables/xl330-m288.json`. The board may not have enough memory to store all of the tables.
- Enter the `repl`
- `Ctrl+D` to reboot. The motor should execute the example by sweeping back and forth.


# TODO
- [ ] Implement tick in either direction
    - Snap to closest multiple of 5
- [ ] Implement multi-turn
    - [Docs for operating modes](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#operating-mode)
    - 3 (default) = position control, 4 = extended position control
    - Must turn off torque before setting mode 
- [ ] Address intermittent packet dropouts, specifically on reads. Maybe due to wiring TX and RX together
- [ ] Find way to prevent shutdown when motor is moving particularly fast. Might be back-emf going into the Pico.