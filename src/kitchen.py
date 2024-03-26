import os, sys 
import time
import pybullet as p
import pybullet_data  
import numpy as np
import matplotlib.pyplot as plt
import math




class Kitchen:
    def __init__(self): 
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        #p.setAdditionalSearchPath(model_path+'/models') 
        #p.setAdditionalSearchPath(pybullet_data.getDataPath())
        kitchen_path = model_path +'/models/kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        p.setGravity(0, 0, -9.81)  
        self.floor = p.loadURDF(model_path +'/models/floor/floor.urdf',useFixedBase=True)
        self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
        self.table = p.loadURDF(model_path +'/models/table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True) 
        self.mug = p.loadURDF(model_path +'/models/kitchenware/mug/mug.urdf',[1.0,0,1.0], p.getQuaternionFromEuler([0,0,0]), useFixedBase=False) 
        self.plate = p.loadURDF(model_path +'/models/kitchenware/plate/plate.urdf',[1.0,0.3,1.0], p.getQuaternionFromEuler([0,0,0]), useFixedBase=False)
        startPos = [0,0,1]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.boxId = p.loadURDF(model_path +'/models/r2d2/r2d2.urdf',startPos, startOrientation) 
        self.drawer_to_joint_id = {1: 18, 
                                   2: 22, 
                                   3: 27, 
                                   4: 31,
                                   5: 37, 
                                   6: 40, 
                                   7: 48, 
                                   8: 53, 
                                   9: 56, 
                                   10: 58, 
                                   11: 14}
        self.drawer_to_joint_limits = {1: (0, 1.57), 
                                       2: (-1.57, 0), 
                                       3: (-1.57, 0), 
                                       4: (0, 1.57),
                                       5: (0.0, 0.4), 
                                       6: (0.0, 0.4), 
                                       7: (0, 1.57), 
                                       8: (-1.57, 0), 
                                       9: (0.0, 0.4), 
                                       10: (0.0, 0.4), 
                                       11: (0, 1.57)}
        
        # robot_nr_joints = p.getNumJoints(boxId)
        # for i in range(0, robot_nr_joints):
        #     info = p.getJointInfo(boxId,i)
        #     print(info)

        # joint_name = joint_name.decode("utf8")
        # part_name = part_name.decode("utf8")
    def open_drawer(self, drawer_id):
        joint_id = self.drawer_to_joint_id[drawer_id]
        open_angle = self.drawer_to_joint_limits[drawer_id][1]
        p.setJointMotorControl2(bodyIndex=self.kitchen, jointIndex=joint_id, controlMode=p.POSITION_CONTROL, targetPosition=open_angle, maxVelocity=0.5) 
        p.stepSimulation()


    def close_drawer(self, drawer_id):
        joint_id = self.drawer_to_joint_id[drawer_id]
        close_angle = self.drawer_to_joint_limits[drawer_id][0]
        p.setJointMotorControl2(bodyIndex=self.kitchen, jointIndex=joint_id, controlMode=p.POSITION_CONTROL, targetPosition=close_angle, maxVelocity=0.5) 
        p.stepSimulation()


### Example usage
if __name__ == '__main__':
    client = p.connect(p.GUI)
    #p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) 

    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)
    distance = 10000
    img_w, img_h = 120, 80

    
    wheels = [2,3]
    steering = [4, 6]

    targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)
    maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)
    steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0)
    
 

    #initialize kitchen object
    kitchen = Kitchen()

   
    # camera = p.getJointInfo(kitchen.boxId, 15) 
    # head = p.getJointInfo(kitchen.boxId, 13)
    # _, joint_name, _, _, _, _, _, _, _, _, _, _, _, _, pos_camera, ori_camera, _ = p.getJointInfo(kitchen.boxId, 15)
    # _, joint_name, _, _, _, _, _, _, _, _, _, _, _, _, pos_head, ori_head, _ = p.getJointInfo(kitchen.boxId, 13)



    
    p.addUserDebugText()
    # for wheel in inactive_wheels:
    #     p.setJointMotorControl2(kitchen.boxId, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    while True:
        robot_pos, robot_ori = p.getBasePositionAndOrientation(kitchen.boxId)
        robot_yaw = p.getEulerFromQuaternion(robot_ori)[-1]
        xA, yA, zA = robot_pos
        yA += 0.1
        zA += 0.3
        
        xB = xA + math.cos(robot_yaw)*distance
        yB = yA + math.sin(robot_yaw)*distance
        zB = zA

        view_matrix = p.computeViewMatrix(cameraEyePosition=[xA,yA,zA],
                                          cameraTargetPosition=[xB,yB,zB],
                                          cameraUpVector=[0,0,1.0])
        
        projection_matrix = p.computeProjectionMatrixFOV(fov=90, aspect=1.0, nearVal=0.02, farVal=3.5)

        img = p.getCameraImage(img_w, img_h, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

        maxForce = p.readUserDebugParameter(maxForceSlider)
        targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
        steeringAngle = p.readUserDebugParameter(steeringSlider)
        #print(targetVelocity)

        for wheel in wheels:
            _,joint_vel,_,_ = p.getJointState(kitchen.boxId, wheel)
            s = ''.join(str(x) for x in joint_vel)
            p.addUserDebugText(s, [1,1,1])
            
            p.setJointMotorControl2(kitchen.boxId,
                                    wheel,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=targetVelocity,
                                    force=maxForce)

        p.resetDebugVisualizerCamera(cameraDistance=0.0, cameraYaw=0, cameraPitch=0, cameraTargetPosition=kitchen.camera_pos)
        #img = p.getCameraImage(224, 224, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        time.sleep(30)
        #open all drawers
        for i in range(10):
            drawer_id = i+1
            kitchen.open_drawer(drawer_id)
            time.sleep(1)

        time.sleep(10)

        #close all drawers
        for i in range(10):
            drawer_id = i+1
            kitchen.close_drawer(drawer_id)
            time.sleep(1)

        time.sleep(10)




