#           _ __      _______    ______             _____ _____ ______ 
#     /\   | |\ \    / /  __ \  |  ____|           |  __ \_   _|  ____|
#    /  \  | | \ \  / /| |__) | | |__ _ __ ___  ___| |__) || | | |__   
#   / /\ \ | |  \ \/ / |  _  /  |  __| '__/ _ \/ _ \  ___/ | | |  __|  
#  / ____ \| |___\  /  | | \ \  | |  | | |  __/  __/ |    _| |_| |____ 
# /_/    \_\______\/   |_|  \_\ |_|  |_|  \___|\___|_|   |_____|______|
#
# PS Move Controller (right) + PS Move Controller (left) + PS Move Controller Position (Head)
#
#
#  - PS Move Controller (left)
#      trigger  -> "trigger"
#      square   -> "application_menu"
#      triangle -> "back"
#      cross    -> "grip"
#      circle   -> "start"
#      move btn -> "touchpad_click"
#      PS btn   -> "system"
#      start
#      select
#
# v1
#  - Modified from original AVLRFreePie Script
#

import math


LEFT_CONTROLLER = 0
RIGHT_CONTROLLER = 1
HMD = 2
HAND_OFFSET = .3 #Hands will be too close to the floor (value is in meters)



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
    
    #Track head position //Offset magic numbers aqquired from the PSMoveService issues section
    alvr.head_position[0] = freePieIO[HMD].x /100
    alvr.head_position[1] = (freePieIO[HMD].y /100) + .1
    alvr.head_position[2] = (freePieIO[HMD].z /100) - 0.1
    diagnostics.watch(alvr.head_position[0])

    # get Left PS Move controller orientation
    left_yaw   = freePieIO[LEFT_CONTROLLER].yaw    
    left_pitch = freePieIO[LEFT_CONTROLLER].pitch
    left_roll  = freePieIO[LEFT_CONTROLLER].roll
    diagnostics.watch(left_pitch)
    diagnostics.watch(left_roll)
    diagnostics.watch(left_yaw)
    
    # get Right PS Move controller orientation
    right_yaw   = freePieIO[RIGHT_CONTROLLER].yaw    
    right_pitch = freePieIO[RIGHT_CONTROLLER].pitch
    right_roll  = freePieIO[RIGHT_CONTROLLER].roll
    diagnostics.watch(right_pitch)
    diagnostics.watch(right_roll)
    diagnostics.watch(right_yaw)
    
    # virtual hand orientation (convert from PS Move euler convention to ALVR)
    q_hand_orientation = psm_euler2quaternion([left_yaw, left_pitch, left_roll])
    [left_yaw, left_pitch, left_roll] = quaternion2euler(q_hand_orientation)
    q_hand_orientation = psm_euler2quaternion([right_yaw, right_pitch, right_roll])
    [right_yaw, right_pitch, right_roll] = quaternion2euler(q_hand_orientation)
	
    # set left Controller orientation
    alvr.controller_orientation[0][0] = left_yaw
    alvr.controller_orientation[0][1] = left_pitch
    alvr.controller_orientation[0][2] = left_roll
    diagnostics.watch(alvr.controller_orientation[0][0])
    
    #set right controller orientation
    alvr.controller_orientation[1][0] = right_yaw
    alvr.controller_orientation[1][1] = right_pitch
    alvr.controller_orientation[1][2] = right_roll
    diagnostics.watch(alvr.controller_orientation[1][0])
    
    # get PS Move controller trigger & buttons
    left_trigger = freePieIO[3].yaw
    left_buttons = int(freePieIO[3].x)
    right_trigger = freePieIO[3].pitch
    right_buttons = int(freePieIO[3].y)
    diagnostics.watch(left_buttons)
    diagnostics.watch(left_trigger)
    diagnostics.watch(right_buttons)
    diagnostics.watch(right_trigger)
    
    # map PS Move buttons to left controller
    #alvr.buttons[0][alvr.Id("trigger")] = left_trigger
    alvr.trigger[0] = left_trigger
    alvr.buttons[0][alvr.Id("application_menu")] = left_buttons & 0b00000001 > 0 # square
    alvr.buttons[0][alvr.Id("back")]             = left_buttons & 0b00000010 > 0 # triangle
    alvr.buttons[0][alvr.Id("grip")]             = left_buttons & 0b00000100 > 0 # cross
    alvr.buttons[0][alvr.Id("start")]            = left_buttons & 0b00001000 > 0 # circle
    alvr.buttons[0][alvr.Id("trackpad_click")]   = left_buttons & 0b00010000 > 0 # needs better touchpad emulation code so just a click for now
    alvr.buttons[0][alvr.Id("system")]           = left_buttons & 0b00100000 > 0 # PS btn
    
    # map PS Move buttons to right controller
    #alvr.buttons[1][alvr.Id("trigger")] = right_trigger
    alvr.trigger[1] = right_trigger
    alvr.buttons[1][alvr.Id("application_menu")] = right_buttons & 0b00000001 > 0 # square
    alvr.buttons[1][alvr.Id("back")]             = right_buttons & 0b00000010 > 0 # triangle
    alvr.buttons[1][alvr.Id("grip")]             = right_buttons & 0b00000100 > 0 # cross
    alvr.buttons[1][alvr.Id("start")]            = right_buttons & 0b00001000 > 0 # circle
    alvr.buttons[1][alvr.Id("trackpad_click")]   = right_buttons & 0b00010000 > 0 # needs better touchpad emulation code so just a click for now
    alvr.buttons[1][alvr.Id("system")]           = right_buttons & 0b00100000 > 0 # PS btn

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

# Map the orientation and position to the left controller
alvr.controller_position[0][0] = freePieIO[LEFT_CONTROLLER].x/100
alvr.controller_position[0][1] = freePieIO[LEFT_CONTROLLER].y/100 + HAND_OFFSET
alvr.controller_position[0][2] = freePieIO[LEFT_CONTROLLER].z/100
diagnostics.watch(alvr.controller_position[0][0])

alvr.buttons[0][alvr.Id("trigger")] = alvr.input_buttons[alvr.InputId("trigger")]
#alvr.buttons[0][alvr.Id("system")] = alvr.input_buttons[alvr.InputId("back")]
diagnostics.watch(alvr.trigger[0])
#alvr.trigger[0] = 1.0 if alvr.buttons[0][alvr.Id("trigger")] else 0.0

#Map the orientation and position to the right controller
alvr.controller_position[1][0] = freePieIO[RIGHT_CONTROLLER].x/100
alvr.controller_position[1][1] = freePieIO[RIGHT_CONTROLLER].y/100 + HAND_OFFSET
alvr.controller_position[1][2] = freePieIO[RIGHT_CONTROLLER].z/100
diagnostics.watch(alvr.controller_position[1][0])

alvr.buttons[1][alvr.Id("trigger")] = alvr.input_buttons[alvr.InputId("trigger")]
#alvr.buttons[1][alvr.Id("system")] = alvr.input_buttons[alvr.InputId("back")]
diagnostics.watch(alvr.trigger[1])
#alvr.trigger[1] = 1.0 if alvr.buttons[1][alvr.Id("trigger")] else 0.0

