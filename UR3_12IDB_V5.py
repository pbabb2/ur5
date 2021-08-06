from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox

import math
import time
RDK = Robolink()

robot = RDK.Item("UR3e")
frame = RDK.Item("UR3e Base")


class MyTask:

    def __init__(self, robot, TCP2CAMdistance=60.0):
        '''
        Performs tasks of the UR3e robot
        ----------
        robot : RoboDK UR3e robot file
            All robots, objects and tools used in a RoboDK project are saved as a RoboDK station (RDK file)

        TCP2CAMdistance : float
            Distance from the Tool Center Position (TCP) to the camera
       
        '''

        self.TCP2CAMdistance = TCP2CAMdistance
        self.robot = robot

    def align_robot_axes(self):
        '''
        Align the robot axes such that the tranlate/rotation is performed in the yz-plane

        '''  
        basepose = frame.Pose()
        pose1 = robot.Pose()
        poselist1 = Pose_2_UR(pose1)
        x2,y2,z2,u2,v2,w2 = poselist1       #positions, angles of robot
        xyz_list = [x2,y2,z2,math.pi,0,0]   #keep positions from robot, set angles (pointing to y-axis)

        xyz_pose = UR_2_Pose(xyz_list)
        robot.setPose(xyz_pose)


        #align with x axis
        #xyz_pose = xyz_pose*rotz(pi/2)
        #robot.setPose(xyz_pose)
        return

    def arm_movex(self,mm):
        '''
        translate TCP in the x direction

        '''  
        pose = self.robot.Pose()            # retrieve the current robot position as a pose (position of the active tool with respect to the active reference frame)
        xyzuvw = Pose_2_UR(pose)            # Read the 4x4 pose matrix as [X,Y,Z, u,v,w] representation (position in mm and orientation vector in radians): same representation as Universal Robots
        x,y,z,u,v,w = xyzuvw
        xyzuvw2 = [x+mm,y,z,u,v,w] 
        pose= UR_2_Pose(xyzuvw2)            # Convert the XYZABC array to a pose (4x4 matrix).rotation wrt tool flange (end effector)
        self.robot.setPose(pose)
        return 

    def arm_movey(self,mm):
        '''
        translate TCP in the y direction

        '''  
        pose = self.robot.Pose()            # retrieve the current robot position as a pose (position of the active tool with respect to the active reference frame)
        xyzuvw = Pose_2_UR(pose)            # Read the 4x4 pose matrix as [X,Y,Z, u,v,w] representation (position in mm and orientation vector in radians): same representation as Universal Robots
        x,y,z,u,v,w = xyzuvw
        xyzuvw2 = [x,y+mm,z,u,v,w] 
        pose= UR_2_Pose(xyzuvw2)            # Convert the XYZABC array to a pose (4x4 matrix).rotation wrt tool flange (end effector)
        self.robot.setPose(pose)            #used setPose instead of MoveL
        return 

    def arm_movez(self,mm):
        '''
        translate TCP in the z direction

        '''  
        pose = self.robot.Pose()            # retrieve the current robot position as a pose (position of the active tool with respect to the active reference frame)
        xyzuvw = Pose_2_UR(pose)            # Read the 4x4 pose matrix as [X,Y,Z, u,v,w] representation (position in mm and orientation vector in radians): same representation as Universal Robots
        x,y,z,u,v,w = xyzuvw
        xyzuvw2 = [x,y,z+mm,u,v,w] 
        pose= UR_2_Pose(xyzuvw2)            # Convert the XYZABC array to a pose (4x4 matrix).rotation wrt tool flange (end effector)
        self.robot.setPose(pose)
        return 

    def rotate_cam(self):
        '''
        rotate camera -30 degrees about the TCP 

        '''  
        pose3 = self.robot.Pose()
        pose4 = pose3*rotx(-pi/6)           #rotate arm to make cam to face down
        self.robot.setPose(pose4)           
        return

    def rotate_cam_back(self):
        '''
        rotate camera +30 degrees about the TCP

        ''' 
        pose4 = self.robot.Pose()
        pose5 = pose4*rotx(+pi/6)           #rotate arm to make gripper face down
        self.robot.setPose(pose5)
        return

    def translate(self):
        '''
        translate TCP to spotted target

        '''   
        dist = -self.TCP2CAMdistance*math.cos(pi/6)
        self.arm_movey(dist)
        return


    def translate_back(self):  
    
        '''
        translate TCP back from spotted target

        '''  
        dist = self.TCP2CAMdistance*math.cos(pi/6)
        self.arm_movey(dist)
        return

    def function1(self):
        '''
        align axes and perform translate/rotation sequence

        '''  
        self.align_robot_axes()
        self.rotate_cam()
        self.translate()
        #self.take_picture() 
        time.sleep(5)
        return

    def function2(self):
        '''
        align axes and perform translate/rotation sequence

        '''
        self.rotate_cam_back()
        self.translate_back()
        #self.grip_sample()
        return


if __name__ == "__main__":

    mytask =  MyTask(robot, TCP2CAMdistance=60.0)
    mytask.function1()
    mytask.function2()


    #mytask.align_robot_axes()
    #mytask.rotate_cam()
    #mytask.translate()
    #take_picture() 
    #time.sleep(5)
    #mytask.rotate_cam_back()
    #mytask.translate_back()
    #grip_sample()