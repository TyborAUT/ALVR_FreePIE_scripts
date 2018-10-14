#           _ __      _______    ______             _____ _____ ______ 
#     /\   | |\ \    / /  __ \  |  ____|           |  __ \_   _|  ____|
#    /  \  | | \ \  / /| |__) | | |__ _ __ ___  ___| |__) || | | |__   
#   / /\ \ | |  \ \/ / |  _  /  |  __| '__/ _ \/ _ \  ___/ | | |  __|  
#  / ____ \| |___\  /  | | \ \  | |  | | |  __/  __/ |    _| |_| |____ 
# /_/    \_\______\/   |_|  \_\ |_|  |_|  \___|\___|_|   |_____|______|
#
# Oculus Go Controller (right) + PS Move Controller (left)
#
# Button mapping:
#  - Go Controller (right)
#      trigger  -> "trigger"
#      trackpad -> "trackpad" or
#                  fly mode (touch up/down to fly forward/backward of controller direction,
#                  click center to reset movement)  
#      back     -> "application_menu"
#
#  - PS Move Controller (left)
#      trigger  -> "trigger"
#      square   -> "application_menu"
#      triangle -> "back"
#      cross    -> "grip"
#      circle   -> "start"
#      move btn -> selected mode dependent:
#                    default mode -> "trackpad click"
#                    fly mode with Go Controller trackpad & direction
#                    arm rotation with PS Move Controller up/down rotation
#      PS btn   -> "system"
#      start
#      select   -> switch between modes: MODE_DEFAULT, MODE_FLY, MODE_ARM
#
# v3
#  - added trackpad click in default mode
#  - code cleanup
#
# v2
#  - implemented new virtual arm model for both controllers
#  - new mode for arm rotation with PS Move Controller up/down rotation
#  - changed GoC "back" btn mapping
#
# v1
#  - initial script
#

import math
from System import Array

MODE_DEFAULT = 0
MODE_FLY  = 1
MODE_ARM  = 2

g_mode_list = [
    [MODE_DEFAULT, "Default"]
    ,[MODE_FLY, "Fly Mode"]
    ,[MODE_ARM, "Arm Mode"]
]


def sign(x): return 1 if x >= 0 else -1

# conjugate quaternion
def conj(q):
    return [-q[0], -q[1], -q[2], q[3]]

# multiplication of quaternion
def multiply(a, b):
    x0, y0, z0, w0 = a
    x1, y1, z1, w1 = b
    return [x1 * w0 - y1 * z0 + z1 * y0 + w1 * x0,
            x1 * z0 + y1 * w0 - z1 * x0 + w1 * y0,
            -x1 * y0 + y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0]

# convert quaternion to euler
def quaternion2euler(q):
    yaw_pitch_roll = [0.0, 0.0, 0.0]
    # roll (x-axis rotation)
    sinr = +2.0 * (q[3] * q[0] + q[1] * q[2])
    cosr = +1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1])
    yaw_pitch_roll[2] = math.atan2(sinr, cosr)

    # pitch (y-axis rotation)
    sinp = +2.0 * (q[3] * q[1] - q[2] * q[0])
    if (math.fabs(sinp) >= 1):
        yaw_pitch_roll[1] = math.copysign(M_PI / 2, sinp)
    else:
        yaw_pitch_roll[1] = math.asin(sinp)

    # yaw (z-axis rotation)
    siny = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    cosy = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    yaw_pitch_roll[0] = math.atan2(siny, cosy);

    return yaw_pitch_roll

# convert euler to quaternion
def euler2quaternion(yaw_pitch_roll):
    cy = math.cos(yaw_pitch_roll[0] * 0.5);
    sy = math.sin(yaw_pitch_roll[0] * 0.5);
    cr = math.cos(yaw_pitch_roll[2] * 0.5);
    sr = math.sin(yaw_pitch_roll[2] * 0.5);
    cp = math.cos(yaw_pitch_roll[1] * 0.5);
    sp = math.sin(yaw_pitch_roll[1] * 0.5);

    return [cy * sr * cp - sy * cr * sp, # x
    cy * cr * sp + sy * sr * cp,         # y
    sy * cr * cp - cy * sr * sp,         # z
    cy * cr * cp + sy * sr * sp]         # w

# convert PS Move euler angles to quaternion
def psm_euler2quaternion(yaw_pitch_roll):
    cy = math.cos(yaw_pitch_roll[0] * 0.5);
    sy = math.sin(yaw_pitch_roll[0] * 0.5);
    cr = math.cos(yaw_pitch_roll[2] * 0.5);
    sr = math.sin(yaw_pitch_roll[2] * 0.5);
    cp = math.cos(yaw_pitch_roll[1] * 0.5);
    sp = math.sin(yaw_pitch_roll[1] * 0.5);

    return [cy * cr * sp + sy * sr * cp, # x
    sy * cr * cp + cy * sr * sp,         # y
    cy * sr * cp - sy * cr * sp,         # z
    cy * cr * cp - sy * sr * sp]         # w

# extract the rotation of a given axis from a quaternion
def q_extract_axis(quat, axis):
    # q[0] = x; q[1] = y; q[2] = z; q[3] = w
    q = [quat[0], quat[1], quat[2], quat[3]]
    if axis == 0:
        i = 1; j = 2;
    elif axis == 1:
        i = 0; j = 2;
    elif axis == 2:
        i = 0; j = 1;
    else:
        return q
    q[i] = 0
    q[j] = 0
    mag = math.sqrt(q[3]*q[3] + q[axis]*q[axis])
    q[axis] /= mag
    q[3] /= mag
    return q

# rotate specified vector using yaw_pitch_roll
def rotatevec(yaw_pitch_roll, vec):
    q = euler2quaternion(yaw_pitch_roll)
    return multiply(multiply(q, vec), conj(q))

# rotate specified vector using a quaternion
def q_rotatevec(q, vec):
    return multiply(multiply(q, vec), conj(q))

# to overcome a flipped rotation beyond 90° we "normalize" the rotation
# by rotating the other axis back to 0° -> now we get the desired angle
# independent from orientation
def get_normalized_roll(yaw_pitch_roll):
    q = euler2quaternion(yaw_pitch_roll)
    q_norm = multiply(conj(q_extract_axis(q, 1)), q)
    q_norm = multiply(conj(q_extract_axis(q, 2)), q_norm)
    [yaw, pitch, roll] = quaternion2euler(q_norm)
    return roll

# calculate virtual arm model with shoulder, elbow, arm forward rotation and hand
def calc_arm_model(controller_orientation, leftright, upper_arm_roll):
    
    # hand stretches when arm is rotated forward to reach further
    hand = [0.0, 0.0, -0.25 - 0.25*math.sin(upper_arm_roll), 0.0]
    hand = rotatevec(controller_orientation, hand)
    
    # left/right shoulder is fixed
    shoulder = [0.2*leftright, -0.10, -0.05]
    
    # rotate virtual elbow according to arm roll
    roll = get_normalized_roll(controller_orientation)
    if roll < 0.0:
        arm_roll = upper_arm_roll + roll # your elbow limits your possible hand rotation
    else:
        arm_roll = upper_arm_roll
    if arm_roll < 0.0:
        arm_roll = 0.0
    elbow = rotatevec([0.0, 0.0, arm_roll], [0.0, -0.25, 0.0, 0.0])
    elbow[0] += shoulder[0]
    elbow[1] += shoulder[1]
    elbow[2] += shoulder[2]
    #diagnostics.watch(roll)
    #diagnostics.watch(elbow[1])
    
    # get head orientation and calculate quaternion
    q_head_orientation = euler2quaternion(alvr.input_head_orientation)
    # extract only pitch out of quaternion
    q_head_pitch = q_extract_axis(q_head_orientation, 1)
    # rotate virtual elbow according to head pitch
    elbow = q_rotatevec(q_head_pitch, elbow)
    
    # return Controller position as Array[float]
    controller_position = Array.CreateInstance(float, 3);
    controller_position[0] = alvr.head_position[0] + elbow[0] + hand[0]
    controller_position[1] = alvr.head_position[1] + elbow[1] + hand[1]
    controller_position[2] = alvr.head_position[2] + elbow[2] + hand[2]
    return controller_position

def updatePSMove():
    global g_PSM_orientation, g_active_mode, g_selected_mode, g_mode_list, g_arm_roll, g_arm_roll_old, g_origin_roll, g_buttons
    
    # get PS Move controller orientation
    yaw   = freePieIO[0].yaw    
    pitch = freePieIO[0].pitch
    roll  = freePieIO[0].roll
    #diagnostics.watch(pitch)
    #diagnostics.watch(roll)
    #diagnostics.watch(yaw)
    
    # virtual hand orientation (convert from PS Move euler convention to ALVR)
    q_hand_orientation = psm_euler2quaternion([yaw, pitch, roll])
    [yaw, pitch, roll] = quaternion2euler(q_hand_orientation)
    
    #diagnostics.watch(pitch)
    #diagnostics.watch(roll)
    #diagnostics.watch(yaw)
    
    # set PSM Controller orientation
    g_PSM_orientation[0] = yaw
    g_PSM_orientation[1] = pitch
    g_PSM_orientation[2] = roll
    
    # get PS Move controller trigger & buttons
    trigger = freePieIO[3].yaw
    buttons = int(freePieIO[3].x)
    
    # map PS Move buttons to 2nd controller
    alvr.buttons[1][alvr.Id("trigger")] = trigger
    alvr.trigger[1] = trigger
    alvr.buttons[1][alvr.Id("application_menu")] = buttons & 0b00000001 > 0 # square
    alvr.buttons[1][alvr.Id("back")]             = buttons & 0b00000010 > 0 # triangle
    alvr.buttons[1][alvr.Id("grip")]             = buttons & 0b00000100 > 0 # cross
    alvr.buttons[1][alvr.Id("start")]            = buttons & 0b00001000 > 0 # circle
    
    if buttons & 0b00010000 > 0: # move btn
        if g_active_mode == MODE_DEFAULT:
            if g_mode_list[g_selected_mode][0] == MODE_ARM:
                # fetch current controller orientation and normalize to roll axis
                g_origin_roll = get_normalized_roll([yaw, pitch, roll])
                g_arm_roll_old = g_arm_roll
        g_active_mode = g_mode_list[g_selected_mode][0]
    else:
        g_active_mode = MODE_DEFAULT
    if g_active_mode == MODE_DEFAULT:
        alvr.buttons[1][alvr.Id("trackpad_click")] = buttons & 0b00010000 > 0 # move btn
    alvr.buttons[1][alvr.Id("system")]           = buttons & 0b00100000 > 0 # PS btn
    
    diagnostics.watch(buttons)
    if buttons & 0b10000000 > 0: # select
        if g_buttons & 0b10000000 == 0:
            # pressed btn
            g_selected_mode += 1
            if g_selected_mode >= len(g_mode_list):
                g_selected_mode = 0
            alvr.message = "-> " + g_mode_list[g_selected_mode][1]
    else:
        if g_buttons & 0b10000000 > 0:
            # released btn
            alvr.message = ""
    
    # save button state for next run
    g_buttons = buttons
    diagnostics.watch(g_selected_mode)

if starting:
    g_active_mode = MODE_DEFAULT
    g_selected_mode = 0
    g_origin_roll = 0.0
    g_arm_roll = 0.0
    g_arm_roll_old = 0.0
    g_buttons = 0
    offset = [0.0, 0.0, 0.0]
    g_PSM_orientation = [0.0, 0.0, 0.0]
    
    # enable 2 controllers in ALVR
    alvr.two_controllers = True
    # override controller position & orientation
    alvr.override_controller_position = True
    alvr.override_controller_orientation = True
    alvr.override_head_position = True
    
    # add update function for PS Move controller
    freePieIO[0].update += updatePSMove

# use default mapping of ALVR for Oculus Go controller
# set 1st Controller position & orientation
alvr.controller_position[0] = calc_arm_model(alvr.input_controller_orientation, +1, g_arm_roll)
alvr.controller_orientation[0][0] = alvr.input_controller_orientation[0]
alvr.controller_orientation[0][1] = alvr.input_controller_orientation[1]
alvr.controller_orientation[0][2] = alvr.input_controller_orientation[2]

alvr.buttons[0][alvr.Id("trigger")] = alvr.input_buttons[alvr.InputId("trigger")]
alvr.buttons[0][alvr.Id("application_menu")] = alvr.input_buttons[alvr.InputId("back")]
alvr.trigger[0] = 1.0 if alvr.buttons[0][alvr.Id("trigger")] else 0.0

# set 2nd Controller position
alvr.controller_position[1] = calc_arm_model(g_PSM_orientation, -1, g_arm_roll)
# set 2nd Controller orientation
alvr.controller_orientation[1][0] = g_PSM_orientation[0]
alvr.controller_orientation[1][1] = g_PSM_orientation[1]
alvr.controller_orientation[1][2] = g_PSM_orientation[2]

if g_active_mode == MODE_FLY:
    alvr.message = "Fly Mode"
    # fly mode
    # press upper half of trackpad to forward into controller direction. bottom half to fly backward
    if alvr.input_buttons[alvr.InputId("trackpad_touch")]:
        outvec = rotatevec(alvr.input_controller_orientation, [0, 0, -1, 0])
        speed = 0.0
        #diagnostics.watch(alvr.input_trackpad[1])
        if (alvr.input_trackpad[1] > 0.5) or (alvr.input_trackpad[1] < -0.5):
            speed = 0.002 * sign(alvr.input_trackpad[1])
        #diagnostics.watch(speed)
        offset[0] += speed * outvec[0]
        offset[1] += speed * outvec[1]
        offset[2] += speed * outvec[2]
    # reset movement by pressing trackpad in the center
    if alvr.input_buttons[alvr.InputId("trackpad_click")]:
        offset = [0.0, 0.0, 0.0]

elif g_active_mode == MODE_ARM:
    # fetch current controller orientation and normalize to roll axis
    c_roll = get_normalized_roll(alvr.controller_orientation[1])
 
    g_arm_roll = g_arm_roll_old + (c_roll - g_origin_roll)
     
    if g_arm_roll > math.pi/2:
        g_arm_roll = math.pi/2
    elif g_arm_roll < 0.0:
        g_arm_roll = 0.0
    
    alvr.message = "Arm " + "%.0f deg" % math.degrees(g_arm_roll)
    
else:
    # map trackpad to controller
    alvr.buttons[0][alvr.Id("trackpad_touch")] = alvr.input_buttons[alvr.InputId("trackpad_touch")]
    alvr.buttons[0][alvr.Id("trackpad_click")] = alvr.input_buttons[alvr.InputId("trackpad_click")]
    alvr.trackpad[0][0] = alvr.input_trackpad[0]
    alvr.trackpad[0][1] = alvr.input_trackpad[1]
    
    if g_buttons == 0:
        alvr.message = ""

alvr.head_position[0] = alvr.input_head_position[0] + offset[0]
alvr.head_position[1] = alvr.input_head_position[1] + offset[1]
alvr.head_position[2] = alvr.input_head_position[2] + offset[2]

