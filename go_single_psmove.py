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
#      back     -> "system"
#
#  - PS Move Controller (left)
#      trigger  -> "trigger"
#      square   -> "application_menu"
#      triangle -> "back"
#      cross    -> "grip"
#      circle   -> "start"
#      move btn -> activate movement (fly mode with Go Controller trackpad & direction)
#      PS btn   -> "system"
#      start
#      select
#
# v1
#  - initial script
#

import math

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
def q_extract_axis(q, axis):
    # q[0] = x; q[1] = y; q[2] = z; q[3] = w
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

def updatePSMove():
    global movement_active
    
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
    l_hand = q_rotatevec(q_hand_orientation, [0, 0, -0.25, 0])

    # get head orientation and calculate quaternion
    q_head_orientation = euler2quaternion(alvr.input_head_orientation)
    # extract only pitch out of quaternion
    q_head_pitch = q_extract_axis(q_head_orientation, 1)
    # rotate virtual left elbow according to head pitch
    l_elbow = q_rotatevec(q_head_pitch, [-0.2, -0.35, -0.05, 0])
    #diagnostics.watch(l_elbow[0])
    #diagnostics.watch(l_elbow[1])
    #diagnostics.watch(l_elbow[2])
    
    # set 2nd Controller position
    alvr.controller_position[1][0] = alvr.head_position[0] + l_elbow[0] + l_hand[0]
    alvr.controller_position[1][1] = alvr.head_position[1] + l_elbow[1] + l_hand[1]
    alvr.controller_position[1][2] = alvr.head_position[2] + l_elbow[2] + l_hand[2]
    # set 2nd Controller orientation
    alvr.controller_orientation[1][0] = yaw
    alvr.controller_orientation[1][1] = pitch
    alvr.controller_orientation[1][2] = roll
    
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
    movement_active                              = buttons & 0b00010000 > 0 # move btn
    alvr.buttons[1][alvr.Id("system")]           = buttons & 0b00100000 > 0 # PS btn

if starting:
    movement_active = False
    offset = [0.0, 0.0, 0.0]
    
    # enable 2 controllers in ALVR
    alvr.two_controllers = True
    # override controller position & orientation
    alvr.override_controller_position = True
    alvr.override_controller_orientation = True
    alvr.override_head_position = True
    
    # add update function for PS Move controller
    freePieIO[0].update += updatePSMove

# use default mapping of ALVR for Oculus Go controller
alvr.controller_position[0][0] = alvr.input_controller_position[0] + offset[0]
alvr.controller_position[0][1] = alvr.input_controller_position[1] + offset[1]
alvr.controller_position[0][2] = alvr.input_controller_position[2] + offset[2]
alvr.controller_orientation[0][0] = alvr.input_controller_orientation[0]
alvr.controller_orientation[0][1] = alvr.input_controller_orientation[1]
alvr.controller_orientation[0][2] = alvr.input_controller_orientation[2]

alvr.buttons[0][alvr.Id("trigger")] = alvr.input_buttons[alvr.InputId("trigger")]
alvr.buttons[0][alvr.Id("system")] = alvr.input_buttons[alvr.InputId("back")]
alvr.trigger[0] = 1.0 if alvr.buttons[0][alvr.Id("trigger")] else 0.0

#diagnostics.watch(movement_active)
if movement_active:
    # use trackpad for movement
    # fly mode
    # press upper half of trackpad to forward. bottom half to back
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
else:
    # map trackpad to controller
    alvr.buttons[0][alvr.Id("trackpad_touch")] = alvr.input_buttons[alvr.InputId("trackpad_touch")]
    alvr.buttons[0][alvr.Id("trackpad_click")] = alvr.input_buttons[alvr.InputId("trackpad_click")]
    alvr.trackpad[0][0] = alvr.input_trackpad[0]
    alvr.trackpad[0][1] = alvr.input_trackpad[1]

alvr.head_position[0] = alvr.input_head_position[0] + offset[0]
alvr.head_position[1] = alvr.input_head_position[1] + offset[1]
alvr.head_position[2] = alvr.input_head_position[2] + offset[2]

