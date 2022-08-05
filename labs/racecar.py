"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import enum

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

class Mode(enum.IntEnum):
    follow = 0
    pass_car = 1
    align = 2
    passing = 3
    reverse = 4



# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# Area of the cone contour when we are the correct distance away (must be tuned)
GOAL_AREA = 26000

# Area of the cone contour when we should switch to reverse while aligning
REVERSE_AREA = GOAL_AREA * 0.4

# Area of the cone contour when we should switch to forward while aligning
FORWARD_AREA = GOAL_AREA * 0.2


RED = ((170, 50, 50), (10, 255, 255))  # The HSV range for the color red

# If desired speed/angle is under these thresholds, they are considered "close enough"
SPEED_THRESHOLD = 0.04
ANGLE_THRESHOLD = 0.1

GOAL_DISTANCE = 30
PASS_DIST = 10

# Speeds
MAX_ALIGN_SPEED = 0.8
MIN_ALIGN_SPEED = 0.4
PASS_SPEED = 1
FIND_SPEED = 0.2
REVERSE_SPEED = -0.2
NO_CONES_SPEED = 0.4

# Times
REVERSE_BRAKE_TIME = 0.25
SHORT_PASS_TIME = 10
LONG_PASS_TIME = 5

# Cone finding parameters
MIN_CONTOUR_AREA = 100
MAX_DISTANCE = 250
REVERSE_DISTANCE = 50
STOP_REVERSE_DISTANCE = 60

CLOSE_DISTANCE = 10
FAR_DISTANCE = 120

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
cur_mode = Mode.follow
counter = 0

prev_distance = 0
distance = 0

########################################################################################
# Functions
########################################################################################


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()
    

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, RED[0], RED[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        # rc.display.show_color_image(image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global cur_mode

    # Initialize variables
    speed = 0
    angle = 0.5

    counter = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Begin in "follow" mode
    cur_mode = Mode.follow



def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global counter
    global speed
    global angle
    global cur_mode
    global GOAL_DISTANCE
    global contour_center
    global prev_dist
    global distance

    prev_dist = distance

    counter += rc.get_delta_time()

    if counter <= 2:
        return

    # Search for contours in the current color image
    update_contour()

    depth_image = rc.camera.get_depth_image()
    rc.display.show_depth_image(depth_image)
    scan = rc.lidar.get_samples()

    # if counter >= 5:
    #     cur_mode = Mode.pass_car

    # # if we have time
    # if counter >= 8:
    #     cur_mode = Mode.follow

    # Find the distance of the cone contour
    if contour_center is not None:
        distance = rc_utils.get_pixel_average_distance(depth_image, contour_center)
    else:
        distance = 0

    

    if contour_center is None:
        print("can't see car")
        speed = 1
        angle = 0.5
        print(cur_mode)

        cur_mode == Mode.passing


    else:
        if cur_mode == Mode.follow:
            if distance < 5:
                speed = -1
        
            # Use proportional control to set wheel angle based on contour x position
            angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1, True)
            speed = rc_utils.remap_range(distance, GOAL_DISTANCE * 2, GOAL_DISTANCE, 1.0, -1.0, True)
                
        is_safe= True
        right_angle, right_dist = rc_utils.get_lidar_closest_point(scan, (40, 50))

        if right_dist >= 20:
            is_safe = True
        else:
            is_safe = False



        if cur_mode == Mode.pass_car:
            
            if not is_safe:
                print("not safe")
                speed = -1
                cur_mode = Mode.follow

            

            # else:
            #     contour_center = (contour_center[0], contour_center[1] + int(distance * PASS_DIST)) #could cause out of bounds
            #     speed = 0.9
            #     angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1, True)

            
            # Align ourselves to smoothly approach and pass the red cone while it is in view
            if cur_mode == Mode.pass_car:
                # Once the red cone is out of view, enter Mode.red_pass
                # if (
                #     contour_center is None
                #     or distance == 0
                #     or distance - prev_distance > CLOSE_DISTANCE
                # ):
                #     if 0 < prev_distance < FAR_DISTANCE:
                #         counter = max(SHORT_PASS_TIME, counter)
                #         cur_mode = Mode.passing
                #     else:
                #         print("line 239")
                #         cur_mode = Mode.follow

                # # If it seems like we are not going to make the turn, enter Mode.red_reverse
                # elif (
                #     distance < REVERSE_DISTANCE
                #     and contour_center[1] > rc.camera.get_width() // 4
                # ):
                #     counter = REVERSE_BRAKE_TIME
                #     cur_mode = Mode.reverse
                if(1==0):
                    pass

                # Align with the cone so that it gets closer to the left side of the screen
                # as we get closer to it, and slow down as we approach
                else:
                    goal_point = rc_utils.remap_range(
                        distance,
                        CLOSE_DISTANCE,
                        FAR_DISTANCE,
                        0,
                        rc.camera.get_width() // 4,
                        True,
                    )

                    angle = rc_utils.remap_range(
                        contour_center[1], goal_point, rc.camera.get_width() // 2, 0, 1
                    )
                    angle = rc_utils.clamp(angle, -0.05, 1)
                    print(angle)

                    # speed = rc_utils.remap_range(
                    #     distance,
                    #     CLOSE_DISTANCE,
                    #     FAR_DISTANCE,
                    #     MIN_ALIGN_SPEED,
                    #     MAX_ALIGN_SPEED,
                    #     True,
                    # )
                    speed = 1

                # Curve around the cone at a fixed speed for a fixed time to pass it
            if cur_mode == Mode.passing:
                # angle = rc_utils.remap_range(counter, 1, 0, 0, -0.1, True)
                angle = 0
                speed = PASS_SPEED
                counter -= rc.get_delta_time()

                # After the counter expires, enter Mode.blue_align if we see the blue cone,
                # and Mode.blue_find if we do not
                if counter <= 0:
                    #cur_mode = Mode.blue_align if blue_distance > 0 else Mode.blue_find
                    print("out of time to pass the car")
                    speed = 0.5
                    angle = 0


            # # If we know we are supposed to be aligning with a red cone but do not see one,
            # # turn to the right until we find it
            # elif cur_mode == Mode.red_find:
            #     angle = 1
            #     speed = FIND_SPEED
            #     if red_distance > 0:
            #         cur_mode = Mode.red_align

             # If we are not going to make the turn, reverse while keeping the cone in view
            elif cur_mode == Mode.reverse:
                if counter >= 0:
                    counter -= rc.get_delta_time()
                    speed = -1
                    angle = 1
                else:
                    angle = -1
                    speed = REVERSE_SPEED
                    if (
                        red_distance > STOP_REVERSE_DISTANCE
                        or red_center[1] < rc.camera.get_width() // 10
                    ):
                        counter = LONG_PASS_TIME
                        cur_mode = Mode.align

        print(cur_mode)    
        rc.drive.set_speed_angle(speed, angle)






    #     # PARK MODE: Move forward or backward until contour_area is GOAL_AREA
    #     if cur_mode == Mode.park:
    #         speed = rc_utils.remap_range(
    #             contour_area, GOAL_AREA / 2, GOAL_AREA, 1.0, 0.0
    #         )
    #         speed = rc_utils.clamp(speed, -PARK_SPEED, PARK_SPEED)

    #         # If speed is close to 0, round to 0 to "park" the car
    #         if -SPEED_THRESHOLD < speed < SPEED_THRESHOLD:
    #             speed = 0

    #         # If the angle is no longer correct, choose mode based on area
    #         if abs(angle) > ANGLE_THRESHOLD:
    #             cur_mode = Mode.forward if contour_area < FORWARD_AREA else Mode.reverse

    #     # FORWARD MODE: Move forward until area is greater than REVERSE_AREA
    #     elif cur_mode == Mode.forward:
    #         speed = rc_utils.remap_range(
    #             contour_area, MIN_CONTOUR_AREA, REVERSE_AREA, 1.0, 0.0
    #         )
    #         speed = rc_utils.clamp(speed, 0, ALIGN_SPEED)

    #         # Once we pass REVERSE_AREA, switch to reverse mode
    #         if contour_area > REVERSE_AREA:
    #             cur_mode = Mode.reverse

    #         # If we are close to the correct angle, switch to park mode
    #         if abs(angle) < ANGLE_THRESHOLD:
    #             cur_mode = Mode.park

    #     # REVERSE MODE: move backward until area is less than FORWARD_AREA
    #     else:
    #         speed = rc_utils.remap_range(
    #             contour_area, REVERSE_AREA, FORWARD_AREA, -1.0, 0.0
    #         )
    #         speed = rc_utils.clamp(speed, -ALIGN_SPEED, 0)

    #         # Once we pass FORWARD_AREA, switch to forward mode
    #         if contour_area < FORWARD_AREA:
    #             cur_mode = Mode.forward

    #         # If we are close to the correct angle, switch to park mode
    #         if abs(angle) < ANGLE_THRESHOLD:
    #             cur_mode = Mode.park

    #     # Reverse the angle if we are driving backward
    #     if speed < 0:
    #         angle *= -1

    # rc.drive.set_speed_angle(speed, angle)

    # # Print the current speed and angle when the A button is held down
    # if rc.controller.is_down(rc.controller.Button.A):
    #     print("Speed:", speed, "Angle:", angle)

    # # Print the center and area of the largest contour when B is held down
    # if rc.controller.is_down(rc.controller.Button.B):
    #     if contour_center is None:
    #         print("No contour found")
    #     else:
    #         print("Center:", contour_center, "Area:", contour_area)

    # # Print the current mode when the X button is held down
    # if rc.controller.is_down(rc.controller.Button.X):
    #     print("Mode:", cur_mode)





########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()