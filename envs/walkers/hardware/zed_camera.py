 ########################################################################
#
# Copyright (c) 2021, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    This sample shows how to track the position of the ZED camera 
    and displays it in a OpenGL window.
"""

import sys
import pyzed.sl as sl
import numpy as np

def transform_pose(pose, Taw) :
    pose = Taw * pose 
    return pose

def initializeCamera(params):
    zed = sl.Camera()
    status = zed.open(params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    tracking_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(tracking_params)
    return zed


def GetZEDPosition(botActive, posEst, memLock, active, reset):
    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.VGA,
                                    camera_fps=100,
                                    coordinate_units=sl.UNIT.METER)

    zed = initializeCamera(init_params)

    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()

    #camera_info = zed.get_camera_information()

    transform_data = sl.Transform()
    transform_data.set_identity()

    #print(camera_info)
    lost_count = 0
    print("ZED Camera: Online")

    while(botActive.is_set()): 
        
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(camera_pose)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                active.set()
                new_pose = sl.Transform()
                new_pose.init_matrix(transform_pose(camera_pose.pose_data(sl.Transform()), transform_data))
                rotation = new_pose.get_rotation_vector()
                translation = new_pose.get_translation()
                lost_count = 0
                # populate shared array
                memLock.acquire()
                posEst[:] = np.append(translation.get(), rotation)
                memLock.release()
            else:
                
                lost_count += 1
                if lost_count == 100:
                    print("Error: Position lost. Restarting camera.")
                    if active.is_set():
                        active.clear()
                    else:
                        print("Error: Unknown issue with camera.")
                        zed.close()
                        exit()
                    '''
                    try:
                        zed.close()
                    except Exception as e:
                        print(e)
                        exit()
                    '''
                    if zed.is_opened():
                        print("Attempting to close camera...")
                        zed.close()
                        print("Camera closed.")

                    lost_count = 0
                    zed = initializeCamera(init_params)
                    transform_data.set_identity()
        else:
            status = zed.grab(runtime)
            print(repr(status))
            exit()
        
        if reset.is_set():
            transform_data = camera_pose.pose_data(sl.Transform())
            transform_data.inverse()

            reset.clear()

    zed.close()

