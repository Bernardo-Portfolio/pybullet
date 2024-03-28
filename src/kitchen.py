import os, sys 
import time
import pybullet as p
import pybullet_data  
import numpy as np
import matplotlib.pyplot as plt
import math
from PIL import Image
import csv

class JointState:
    fr =[]
    br=[]
    fl = []
    bl=[]
    time_ = time.time()
    target_velocity = 2
    time_i = time.time()
 


class Kitchen:
    def __init__(self): 
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        kitchen_path = model_path +'/models/kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'  
        self.floor = p.loadURDF(model_path +'/models/floor/floor.urdf',useFixedBase=True)
        self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
        self.table = p.loadURDF(model_path +'/models/table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True) 
        self.mug = p.loadURDF(model_path +'/models/kitchenware/mug/mug.urdf',[1.0,0,1.0], p.getQuaternionFromEuler([0,0,0]), useFixedBase=False) 
        self.plate = p.loadURDF(model_path +'/models/kitchenware/plate/plate.urdf',[1.0,0.3,1.0], p.getQuaternionFromEuler([0,0,0]), useFixedBase=False)
        self.fridge = p.loadURDF(model_path +'/models/fridge/fridge.urdf',[-5,3,0.0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=False)

        startPos = [0,0,1]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.boxId = p.loadURDF(model_path +'/models/r2d2/r2d2.urdf',startPos, startOrientation) 
        
        self.drawer_to_joint_id = {2: 22}
        self.drawer_to_joint_limits = {2: (-1.57, 0)}
        

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
    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)
    
    distance = 100
    img_w, img_h = 120, 80
    
    wheels = [2,3,6,7]
    steering = [4, 6]
    counter = 0

    #initialize kitchen object
    kitchen = Kitchen()
    joint_state = JointState()
    counter = 0
    my_list = [joint_state.fr, joint_state.br, joint_state.fl, joint_state.bl]
    while True:
        if (joint_state.time_ - joint_state.time_i > 120):
            with open('joint_velocity.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                for joint in my_list:
                    writer.writerow(joint)
            p.disconnect()

        robot_pos, robot_ori = p.getBasePositionAndOrientation(kitchen.boxId)
        robot_yaw = 1.57
        xA, yA, zA = robot_pos
        yA += 0.25 #0.3
        zA += 0.45
        
        xB = xA + math.cos(robot_yaw)*distance
        yB = yA + math.sin(robot_yaw)*distance
        zB = zA

        view_matrix = p.computeViewMatrix(cameraEyePosition=[xA,yA,zA],
                                          cameraTargetPosition=[xB,yB,zB],
                                          cameraUpVector=[0,0,1.0])
        
        projection_matrix = p.computeProjectionMatrixFOV(fov=90, aspect=1.0, nearVal=0.03, farVal=3.5)

        img = p.getCameraImage(img_w, img_h, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb_opengl = (np.reshape(img[2], (img_h, img_w, 4)))
        rgbim = Image.fromarray(rgb_opengl)
        rgbim_no_alpha = rgbim.convert('RGB')
        rgbim_no_alpha.save('dataset/image' + str(counter)+'.jpg')
        counter +=1


        _,joint_vel1,_,_ = p.getJointState(kitchen.boxId, wheels[0])
        _,joint_vel2,_,_ = p.getJointState(kitchen.boxId, wheels[1])
        _,joint_vel3,_,_ = p.getJointState(kitchen.boxId, wheels[2])
        _,joint_vel4,_,_ = p.getJointState(kitchen.boxId, wheels[3])

        joint_state.fr.append(joint_vel1)
        joint_state.br.append(joint_vel2)
        joint_state.fl.append(joint_vel3)
        joint_state.bl.append(joint_vel4)

        current_time = time.time()
        if (current_time-joint_state.time_) > 20:
            joint_state.target_velocity *= -1
            joint_state.time_ = current_time

        for wheel in wheels:
             
            p.setJointMotorControl2(kitchen.boxId,
                                    wheel,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=joint_state.target_velocity,
                                    force=10)
        
        #open all drawers
        
        drawer_id = 2
        kitchen.open_drawer(drawer_id)
        time.sleep(4)


        #close all drawers
        kitchen.close_drawer(drawer_id)
        time.sleep(4)

        # time.sleep(10)
        # maxForce = p.readUserDebugParameter(maxForceSlider)
        # targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
        # steeringAngle = p.readUserDebugParameter(steeringSlider)
        #p.resetDebugVisualizerCamera(cameraDistance=0.0, cameraYaw=0, cameraPitch=0, cameraTargetPosition=kitchen.camera_pos)
        # _, joint_name, _, _, _, _, _, _, _, _, _, _, _, _, pos_camera, ori_camera, _ = p.getJointInfo(kitchen.boxId, 15)
        # _, joint_name, _, _, _, _, _, _, _, _, _, _, _, _, pos_head, ori_head, _ = p.getJointInfo(kitchen.boxId, 13)
        # p.addUserDebugText(str(joint_vel), [0, 0, 1], lifeTime=0.25, textSize=2.5, parentObjectUniqueId=kitchen.boxId)
        # robot_yaw = p.getEulerFromQuaternion(robot_ori)[-1]
        # robot_nr_joints = p.getNumJoints(boxId)
        # for i in range(0, robot_nr_joints):
        #     info = p.getJointInfo(boxId,i)
        #     print(info)
        # joint_name = joint_name.decode("utf8")
        # part_name = part_name.decode("utf8")
        #p.setAdditionalSearchPath(model_path+'/models') 
        #p.setAdditionalSearchPath(pybullet_data.getDataPath())


