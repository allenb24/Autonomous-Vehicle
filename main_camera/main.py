# OpenMV[1] Main Code - Line Detection, Movement Adjustment

import sensor, image, time, pyb, math
from pyb import UART
from pyb import Pin, Timer

'''
UART SETUP FOR 2ND OPENMV INTERFACE
Sets up UART1 at 115200 baud rate for communication with second OpenMV
'''
uart = UART(1,115200)
uart.init(115200, bits=8, parity=None,
stop=1, flow=0)

'''
CAMERA SETUP
Configures the camera with QVGA resolution and applies flip/mirror settings
Uses grayscale for efficiency
'''
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # Grayscale
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(2)
sensor.set_hmirror(True)
clock = time.clock()

# All lines also have `x1()`, `y1()`, `x2()`, and `y2()` methods to get their end-points
# and a `line()` method to get all the above as one 4 value tuple for `draw_line()`.
high_threshold = (89, 255) # THRESHOLD FOR CAM SENSITIVITY
tim = time.ticks_ms()

lane_positions = []
max_lane_positions = 5  # Adjust this value based on your specific requirements

# Deflection angle
def_array = [0] * 4
max_def_buffer = 4

# Distance
dist_array = [0] * 4
max_dist_buffer = 4
avg_distance = 0
threshold = 30 

# Pulse width
pw = 0

'''
line_detection()
Processes the image to detect lane lines, calculates center of lane,
computes deflection angle, and updates the servo accordingly.
'''
def line_detection(lane_positions, def_array, max_def_buffer_buffer, max_lane_positions):
    global pw
    img = sensor.snapshot()
    img = img.rotation_corr(z_rotation=180.00)

    roi=(15,125,290,75)
    img = img.binary([high_threshold])
    img.draw_rectangle(roi, color = (0,255,0), thickness=3)

    max_left = 0
    max_left_line = 0
    max_right = 320
    max_right_line = 0
    theta_left = 0
    theta_right = 0

    lines = img.find_lines(roi=roi,threshold = 1000,x_stride=2, y_stride=1)
    for l in lines:
        if l.theta() != 89 and l.theta() != 0:
            if l.x1() >= max_left:
                max_left = l.x1()
                max_left_line = l
                theta_left = l.theta()
            if l.x2() <= max_right:
                max_right = l.x2()
                max_right_line = l
                theta_right = l.theta()

    if max_right_line != 0:
        a = img.draw_line(max_right_line.line(), color = (255, 0, 0), thickness=3)
        line_right = max_right_line.x1()
    else:
        line_right = 0

    if max_left_line != 0:
        a = img.draw_line(max_left_line.line(), color = (255, 255, 0), thickness=3)
        line_left = max_left_line.x1()
    else:
        line_left = 0

    lane_center = 0

    if line_right != [] and line_left != []:
        lane_center = (line_left+line_right)//2
        print('both')
        #print(line_right)
        #print(line_left)
    elif line_right == 0:
        lane_center = line_left
        print('left')
    elif line_left == 0:
        lane_center = line_right
        print('right')

    # Store lane positions in the buffer
    if lane_center != 0:
        lane_positions.append(lane_center)
        if len(lane_positions) > max_lane_positions:
            lane_positions = lane_positions[-max_lane_positions:]  # Keep only the most recent positions

    # Calculate average lane position from historical data
    if len(lane_positions) > 0:
        average_lane_position = sum(lane_positions) // len(lane_positions)
    else:
        average_lane_position = 0

    # Draw arrow in OpenMV to represent direction
    img.draw_arrow(160, 240, average_lane_position, 120, (0,0,255), thickness=3)

    # Use average lane position for control
    deflection_angle = math.atan((average_lane_position - 160) / 120)
    deflection_angle = math.degrees(deflection_angle)//2 # //2 might be bad

    # Store deflection angles in buffer
    if deflection_angle != 0:
        def_array.append(deflection_angle)
        if len(def_array) > max_def_buffer:
            def_array = def_array[-max_def_buffer:]  # Keep only the most recent positions

    FPS = clock.fps()

    # Set turning angle based on deflection angle
    pw = servo_control_system(def_array, deflection_angle, FPS)
    updateServo(pw)


'''
MOTOR SETUP
PWM motor control using timer and direction pins
'''
inA = pyb.Pin('P2', pyb.Pin.OUT_PP)
inB = pyb.Pin('P3', pyb.Pin.OUT_PP)

# Generate a 1.5KHz square wave on TIM4 with 50%, 75% and 50% duty cycles on channels 1, 2 and 3 respectively.
tim_motor = Timer(2, freq=1500) # Frequency for motor (duty cycle)
ch3_motor = tim_motor.channel(3, Timer.PWM, pin=Pin("P4"))  # Set channel, pins, PW


'''
updateMotor()
Controls motor direction and speed
'''
def updateMotor(duty,direction):
    if direction == 'forward':
        inA.high()
        inB.low()
    elif direction == 'coast':
        duty = 0
        inA.low()
        inB.low()
    elif direction == 'brake':
        duty = 0
        inA.high()
        inB.high()
    elif direction == 'reverse':
        inA.low()
        inB.high()

    ch3_motor.pulse_width_percent(duty)


'''
SERVO SETUP
PWM for steering control
'''
tim_servo = Timer(4, freq = 100)    # Frequency for servo (PW)
src_freq_servo = tim_servo.source_freq()
prescale_servo = tim_servo.prescaler()
period_servo = tim_servo.period()
myfreq_servo = tim_servo.freq()
ch3_servo = tim_servo.channel(3, Timer.PWM, pin=Pin("P9"))  # set channel and pins

'''
updateServo()
Updates servo position based on pulse width
'''
def updateServo(pw):
    #time.sleep(0.2)
    x_servo = pw / ((prescale_servo + 1) * 1000000 / src_freq_servo)
    ch3_servo.pulse_width(int(x_servo))


'''
ULTRASONIC
Reads distance using ultrasonic sensor (not actively used in final demo)
'''
trig = 'P5'
echo = 'P6'
def ultrasonic(trig, echo):
    trigPin = pyb.Pin(trig, pyb.Pin.OUT_PP)
    echoPin = pyb.Pin(echo, pyb.Pin.IN)

    trigPin.value(True)
    time.sleep_us(10)
    trigPin.value(False)

    while echoPin.value() == 0:
        StartTime = time.ticks_us()

    while echoPin.value() == 1:
        StopTime = time.ticks_us()

    TimeElapsed = time.ticks_diff(time.ticks_us(), StartTime) * 10**-4
    distance = (TimeElapsed * 340) / 2

    return distance

'''
PID Control
P - immediate error
I - accumulated error (set to 0 here)
D - future error prediction
'''
# P determines the strength of controller's response to the current error
def p_control(deflection_angle):
    Kp = 25  # Poportional gain, 25 current best wth P alone

    # Calculate the control signal
    control_signal = Kp * deflection_angle

    # Apply the control signal to the servo
    pw = int(1500 - control_signal)

    return pw

# Note: Too much i_correction leads to oscillations and instability
# I reduces the steady state error, but increases overshoot
def i_control(def_array, FPS):
    time_step = 1/FPS # avg FPS (sample rate) is 19 so T is 1/f
    Ki = 0  # 0.41, best rn, Adjust the integral gain (Ki) value according to your requirements
    i_correction = Ki * sum(def_array) * time_step  # Sum of the deflection angle values

    return i_correction

# For tight turns
# D should fix overshoots, predicts future error
def d_control(def_array, deflection_angle, FPS):
    def_array[3] = def_array[2]
    def_array[2] = def_array[1]
    def_array[1] = def_array[0]
    def_array[0] = deflection_angle

    Kd = 0.375 # 0.3-0.375 is best, adjust camera position accordingly
    time_step = 1/FPS
    d_correction = ((Kd * (1/6))/time_step) *(def_array[3] - def_array[0] + 3*def_array[2] - 3*def_array[1])

    return d_correction

# Combine PID control to determine action for servo
def servo_control_system(def_array, deflection_angle, FPS):
    # Proportional control
    p_correction = p_control(deflection_angle)

    # Integral control
    i_correction = i_control(def_array, FPS)

    # Derivative control
    d_correction = d_control(def_array, deflection_angle, FPS)

    # Combine the errors and perform the control action
    servo_control_action = p_correction + i_correction + d_correction

    # Perform the desired action based on the control action value
    return servo_control_action



'''
MAIN LOOP
Listens for command from the secondary camera and updates steering
'''
pw = 1500
x_servo = pw / ((prescale_servo + 1) * 1000000 / src_freq_servo)
duty = 17
direction = 'forward'

while True:
    clock.tick()

    # Interface code for second OpenMV
    if uart.any():
        ch = uart.readchar()
        key = chr(ch)
        print(f"Received Char: {key}")

        if key == 'R':
            updateMotor(duty,'brake')
            time.sleep(3)
            updateMotor(duty, 'forward')
            time.sleep(1)
            key = 'B'
        elif key == 'B':
            updateMotor(duty,'forward')
        else:
            print("Unknown character received")

    line_detection(lane_positions, def_array, max_def_buffer, max_lane_positions)
    