import board
import time

import digitalio
import neopixel
import rotaryio
import pwmio

# from ds28e05  import DS28E05
from irsensor import IRSensors
import adafruit_motor.motor as motor
from analogio import AnalogIn

from distance import Distance

### SANITY - LAB 2 ###
def sanity():
    """ Peripherals """

    # debug
    led = digitalio.DigitalInOut(board.LED)
    led.direction = digitalio.Direction.OUTPUT

    but = digitalio.DigitalInOut(board.GP3)
    but.pull = digitalio.Pull.UP

    rgb = neopixel.NeoPixel(board.GP4, 1)

    # extra stuffs
    # eeprom = DS28E05(board.GP2)

    # IR sensors
    ir = IRSensors(
        board.GP7,  board.GP5,  board.GP6,  board.GP28, # left
        board.GP9,  board.GP10, board.GP11, board.GP26, # center
        board.GP21, board.GP20, board.GP22, board.GP27  # right
    )

    # encoders
    lenc = rotaryio.IncrementalEncoder(board.GP12, board.GP13)
    renc = rotaryio.IncrementalEncoder(board.GP19, board.GP18)

    # motors
    lmot = motor.DCMotor(
        pwmio.PWMOut(board.GP16, frequency=20000),
        pwmio.PWMOut(board.GP17, frequency=20000)
    )
    rmot = motor.DCMotor(
        pwmio.PWMOut(board.GP15, frequency=20000),
        pwmio.PWMOut(board.GP14, frequency=20000)
    )
    lmot.decay_mode = motor.SLOW_DECAY
    rmot.decay_mode = motor.SLOW_DECAY

    # debug
    print("Hello World! Press button to begin tests.")
    while (but.value):
        pass

    print("Blinking the LED underneath your micromouse...")
    for _ in range(6):
        led.value = not led.value
        time.sleep(0.25)

    print("Turning on Neopixel on top of your micromouse...")
    rgb[0] = (255, 0, 0)
    time.sleep(0.5)
    rgb[0] = (0, 255, 0)
    time.sleep(0.5)
    rgb[0] = (0, 0, 255)
    time.sleep(0.5)
    rgb[0] = (1, 1, 1)

    # extra stuffs
    # if eeprom.devices:
    #     print("Detected the following 1-Wire devices...")
    #     for d in eeprom.devices:
    #         print("\t", "".join("{:02x}".format(x) for x in d.rom))

    #     eeprom.write(0, 0, b'MMv3.1')
    #     if not eeprom.read(0, 0, 6) == b'MMv3.1':
    #         print("FAIL! EEPROM write mismatch!")
    # else:
    #     print("FAIL! No 1-Wire devices detected!")

    # IR sensors
    print("Scanning IR sensors! Press button to start and stop.")
    while (but.value):
        pass
    time.sleep(0.05)
    while (not but.value):
        pass

    while (but.value):
        ir.scan()
        print("lir_a:", ir.lir_a, "\t", "lir_b:", ir.lir_b, "\t",
                "cir_a:", ir.cir_a, "\t", "cir_b:", ir.cir_b, "\t",
                "rir_a:", ir.rir_a, "\t", "rir_b:", ir.rir_b
        )
        time.sleep(0.05)
    time.sleep(0.05)
    while (not but.value):
        pass

    # encoders
    print("Reading encoders! Press button to start and stop.")
    while (but.value):
        pass
    time.sleep(0.05)
    while (not but.value):
        pass

    while (but.value):
        print("lenc:", lenc.position, "\t", "renc:", renc.position)
        time.sleep(0.05)
    time.sleep(0.05)
    while (not but.value):
        pass

    # motors
    print("Testing motors! Plug in battery and turn on switch. Press button to start.")
    while (but.value):
        pass

    print("Full speed forward")
    lmot.throttle = 1
    rmot.throttle = 1
    time.sleep(0.5)
    print("Brake")
    lmot.throttle = 0
    rmot.throttle = 0
    time.sleep(0.5)
    print("Full speed backward")
    lmot.throttle = -1
    rmot.throttle = -1
    time.sleep(0.5)
    print("Brake")
    lmot.throttle = 0
    rmot.throttle = 0
    time.sleep(0.5)
    print("Low speed forward")
    lmot.throttle = 0.25
    rmot.throttle = 0.25
    time.sleep(0.5)
    print("Brake")
    lmot.throttle = 0
    rmot.throttle = 0
    time.sleep(0.5)
    print("Low speed backward")
    lmot.throttle = -0.25
    rmot.throttle = -0.25
    time.sleep(0.5)
    print("Brake")
    lmot.throttle = 0
    rmot.throttle = 0
    time.sleep(0.5)

    print("Done with tests!")

    print("Just gonna drive straight now :P")
    while True:
        delta = lenc.position - renc.position
        u = 0.001 * delta
        lmot.throttle = 0.25 - u
        rmot.throttle = 0.25 + u
        print(delta, a.value, b.value)
        time.sleep(0.05)

    return

### ENCODERS - LAB 3 ###
def encoders():
    import board
    import digitalio

    # a and b Hall effect sensors of left encoder
    a = digitalio.DigitalInOut(board.GP12)
    b = digitalio.DigitalInOut(board.GP13)

    counter = 0
    position = 0
    a_prev = a.value

    def leftEncoderRisingEdge():
        """TODO increment or decrement position depending on which way the motor is spinning
        Try experimenting with the instance variables of a and b (we only need either a or b). Refer to the
        waveform code from earlier for an idea, think about rising edges"""
        nonlocal position # access global position
        position += 1 if b.value else -1

    while True:
        a_val = a.value
        if a_val and not a_prev: # rising edge of a
            leftEncoderRisingEdge()
        a_prev = a_val

        if counter % 1000 == 0:
            print(position)
        counter += 1

### IR DETECTORS - LAB 4 ###
def one_detector():
    # adc 
    l_adc = AnalogIn(board.GP28)

    # emitter 
    l_en = digitalio.DigitalInOut(board.GP7)
    l_en.direction = digitalio.Direction.OUTPUT
    l_en.value = False

    # sensors
    lir_a = digitalio.DigitalInOut(board.GP5)
    lir_a.direction = digitalio.Direction.OUTPUT
    lir_a.drive_mode = digitalio.DriveMode.OPEN_DRAIN
    lir_a.value = True # high Z mode

    lir_b = digitalio.DigitalInOut(board.GP6)
    lir_b.direction  = digitalio.Direction.OUTPUT
    lir_b.drive_mode = digitalio.DriveMode.OPEN_DRAIN
    lir_b.value = True

    while True:
        l_en.value = True

        lir_a.value = False
        time.sleep(0.001)
        print(l_adc.value , end=" ")
        lir_a.value = True

        lir_b.value = False
        time.sleep(0.001)
        print(l_adc.value , end="\n")
        lir_b.value = True

        l_en.value = False
        time.sleep(0.05)

def detectors():
    # IR sensors
    ir = IRSensors(
        board.GP7,  board.GP5,  board.GP6,  board.GP28, # left
        board.GP9,  board.GP10, board.GP11, board.GP26, # center
        board.GP21, board.GP20, board.GP22, board.GP27  # right
    )
    ''' Please ensure you change these to your own, these are for another mouse'''
    dist = Distance(ir,
        0.0299, -57.2, 0.0275, -50.7,
        0.0295, -54.4, 0.0237, -50.9,
        0.0258, -67.6, 0.0292, -60.8,
    )
    """Make Sure to allign the constants in the right place:
            la_a, la_b, lb_a, lb_b,
            ca_a, ca_b, cb_a, cb_b,
            ra_a, ra_b, rb_a, rb_b,
    """
    while True:
        dist.scan()
        # print("lir_a:", dist.la, "\t", "lir_b:", dist.lb, "\t",
        #         "cir_a:", dist.ca, "\t", "cir_b:", dist.cb, "\t",
        #         "rir_a:", dist.ra, "\t", "rir_b:", dist.rb)
        print(dist.lb)
        time.sleep(0.5)

### ODOMETRY & MOTOR CONTROL - LAB 5 ###
def odometry():
    lenc = rotaryio.IncrementalEncoder(board.GP12, board.GP13)
    renc = rotaryio.IncrementalEncoder(board.GP19, board.GP18)

    ENCODER_TICKS_PER_REVOLUTION = 217.0
    WHEELBASE_DIAMETER = 86.0
    WHEEL_DIAMETER = 34.0 # mm

    # while True:
    #     K = (3.14159 * WHEEL_DIAMETER)/ENCODER_TICKS_PER_REVOLUTION
    #     left_dist  = lenc.position * K
    #     right_dist = renc.position * K

    #     dist  = (left_dist + right_dist)/2.0
    #     theta = (right_dist - left_dist)/WHEELBASE_DIAMETER

    #     print(dist, theta)
    #     time.sleep(0.05)

    # lmot_in1 = pwmio.PWMOut(board.GP16, frequency=20000)
    # lmot_in2 = pwmio.PWMOut(board.GP17, frequency=20000)

    # while True:
    #     lmot_in1.duty_cycle = 65535
    #     lmot_in2.duty_cycle = 65535-16384

    # motors
    lmot = motor.DCMotor(
        pwmio.PWMOut(board.GP16, frequency=20000),
        pwmio.PWMOut(board.GP17, frequency=20000)
    )
    rmot = motor.DCMotor(
        pwmio.PWMOut(board.GP15, frequency=20000),
        pwmio.PWMOut(board.GP14, frequency=20000)
    )
    lmot.decay_mode = motor.SLOW_DECAY
    rmot.decay_mode = motor.SLOW_DECAY

    while True:
        print("Full speed forward")
        lmot.throttle = 1
        rmot.throttle = 1
        time.sleep(1)
        print("Brake")
        lmot.throttle = 0
        rmot.throttle = 0
        time.sleep(1)
        print("25 percent speed backward")
        lmot.throttle = -0.25
        rmot.throttle = -0.25
        time.sleep(1)
        print("Brake")
        lmot.throttle = 0
        rmot.throttle = 0
        time.sleep(1)

### PID - LAB 6 7 ###
def proportional():
    return NotImplementedError
def pid_controller():
    return NotImplementedError

### MAZE SOLVING & FLOOD FILL - LAB 8 ###
def floodfill():
    return NotImplementedError

if __name__ == "__main__":
    LAB = 2

    if LAB == 2:
        sanity()
    elif LAB == 3:
        encoders()
    elif LAB == 4:
        detectors()
    elif LAB == 5:
        odometry()
    elif LAB == 6:
        proportional()
    elif LAB == 7:
        pid_controller()
    elif LAB == 8:
        floodfill()

