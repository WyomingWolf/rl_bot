# *******************************************************************************
#  zed_camera.py 
#
#  Author: James Mock
#  Email: james.w.mock@protonmail.com
#  Date: 01/21/2022
# *******************************************************************************
"""

import pyzed.sl as sl
import numpy as np
import time

def transform_pose(pose, Taw) :
    pose = Taw * pose 
    return pose

def initializeCamera(params, fail):
    zed = sl.Camera()
    status = zed.open(params)
    attempts = 1
    while status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        zed.close()
        if attempts == 3:
            fail.set()
            exit()
        time.sleep(1.0)
        zed = sl.Camera()
        status = zed.open(params)
        attempts += 1
        
    print("ZED Camera: Online")

    tracking_params = sl.PositionalTrackingParameters(_enable_pose_smoothing=True)
    zed.enable_positional_tracking(tracking_params)
    return zed


def GetZEDPosition(botActive, posEst, memLock, active, fail, reset):
    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.VGA,
                                    camera_fps=100,
                                    coordinate_units=sl.UNIT.METER)

    zed = initializeCamera(init_params, fail)
    zed_model = zed.get_camera_information().camera_model
    zed_serial = zed.get_camera_information().serial_number
    print("Camera Model: {}".format(zed_model))
    print("Serial Number: {}".format(zed_serial))

    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()
    sensors_data = sl.SensorsData()
    old_translation = np.zeros(3)
    old_orientation = np.zeros(3)

    transform_data = sl.Transform()
    transform_data.set_identity()

    #print(camera_info)
    lost_count = 0
    #reset_count = 0
    dt = time.monotonic()
    while(botActive.is_set()): 
        dt = time.monotonic() - dt
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(camera_pose)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                active.set()
                new_pose = sl.Transform()
                new_pose.init_matrix(transform_pose(camera_pose.pose_data(sl.Transform()), transform_data))
                translation = new_pose.get_translation().get()
                orientation = new_pose.get_orientation().get()

                linear_vel = (translation - old_translation)/dt
                #angular_vel = (new_pose.get_euler_angles() - old_orientation)/dt
             
                zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE) # Retrieve only frame synchronized data             
                imu_data = sensors_data.get_imu_data() # Extract IMU data
                # Retrieve linear acceleration and angular velocity
                linear_acc = imu_data.get_linear_acceleration()
                angular_vel = imu_data.get_angular_velocity()

                
                # populate shared array
                memLock.acquire()
                posEst[:] = np.concatenate((translation, orientation, linear_acc, angular_vel, linear_vel), -1)
                memLock.release()

                old_translation = translation
                old_orientation = new_pose.get_euler_angles()
                lost_count = 0

            else:
                
                lost_count += 1
                if lost_count == 100:
                    print("Error: Position lost. Restarting camera.")
                    if active.is_set():
                        active.clear()
                    else:
                        print("Error: Unknown issue with camera.")
                        fail.set()
                        exit()

                    if zed.is_opened():
                        #print("Attempting to close camera...")
                        zed.close()
                        #print("Camera closed.")

                    lost_count = 0
                    zed = initializeCamera(init_params, fail)
                    transform_data.set_identity()
        else:
            status = zed.grab(runtime)
            print(repr(status))
            exit()
        
        if reset.is_set():
            '''
            reset_count += 1
            if reset_count >= 10:
                reset_count = 0
                if zed.is_opened():
                        active.clear()
                        zed.close()
                        zed = initializeCamera(init_params)
                        transform_data.set_identity()
                else:
                        print("Error: Unknown issue with camera.")
                        zed.close()
                        exit()
            '''
            transform_data = camera_pose.pose_data(sl.Transform())
            transform_data.inverse()
            reset.clear()

    zed.close()

