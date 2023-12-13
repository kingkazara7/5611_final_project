import numpy as np
from gym import utils
from numpy.lib.function_base import angle
from mujoco_kinova.envs import mujoco_env
from scipy.spatial.transform import Rotation
import time
import pandas as pd


GOAL_BOUNDS = 0.3
Z_OFFSET = -0.1
DEG_TO_RAD = np.pi/180.0

class csci5611_kinova(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self,c1=-500,c2=1,c3=0,delta=0.03,p=8,cx=10,robot='kinova_arm',goal_mode='full',demonstration=None,set_joints_direct=None,enable_ros=False,expert_path=None):
        self.t = 0
        self.initTime = time.time()
        self.c1 = c1
        self.c2 = c2
        self.delta = delta
        # self.c4 = c4
        self.robot = robot
        self.goal_mode = goal_mode
        self.demonstration = demonstration
        self.set_joints_direct = set_joints_direct
        enable_ros = False
        self.enable_ros = enable_ros
        # self.tra = []
        num_quaternion = 0
        self.num_revolute = 5

        self.joint_pos_index = 4 * num_quaternion + self.num_revolute
        self.joint_vel_index = 3 * num_quaternion + self.num_revolute
        self.goal_index = self.joint_pos_index + 3

        self.workspace = pd.read_csv('/media/haoyi/samsung/kinova_urdf/kinova_workspaceSample.csv')
        print(len(self.workspace))
    
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, robot + ".xml", 2)

    def step(self, a):
        ######Update simulation######
        playback_speed = 2 #1=realtime, 2=halftime, etc.
        vicon_fps = 120
        if self.set_joints_direct:
            self.t = int((time.time()-self.initTime)*vicon_fps//playback_speed%self.jointsDirect.shape[0])
        else:
            self.t = int((time.time()-self.initTime)*vicon_fps//playback_speed)

        if self.demonstration is not None:
            goal = self.goalPts[self.t].copy()            
        else:
            goal = self.sim.data.qpos[self.joint_pos_index:self.goal_index]

        qpos = self.sim.data.qpos
        qpos[self.joint_pos_index:self.goal_index] = goal
        
        if self.set_joints_direct is not None:

            qvel = np.zeros(self.sim.data.qvel.shape)
        else:
            qvel = self.sim.data.qvel


        self.set_state(qpos, qvel)

        ######Compute reward######
        d_ref = 0.09
        d= np.linalg.norm(self.get_body_com("fingertip")-self.get_body_com("target"))

        
        # d = d1+d2+d3
        d = d
        if d < self.delta:
            R_T = 1/2*d**2
        else:
            R_T = self.delta*(np.abs(d)-1/2*self.delta)
            

        R_A = - np.linalg.norm(a)**2 # a comes from main.py, Line 152-157

        reward = self.c1*R_T + self.c2*R_A
        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        done = False

       
        return ob, reward, done, dict(reward_dist=self.c1*R_T, reward_ctrl=self.c2*R_A, d_t=d)
    

    def viewer_setup(self):
        self.viewer.cam.trackbodyid = 0

    def reset_model(self):

        self.t = 0
        self.initTime = time.time()
        
        
        qpos = self.np_random.uniform(low=-0.1, high=0.1, size=self.model.nq) + self.init_qpos
        qvel = self.init_qvel + self.np_random.uniform(low=-.005, high=.005, size=self.model.nv)
        self.goal = self.workspace.sample().to_numpy()
        # self.goal = np.array([[0,0,0]])
    
        qpos[self.joint_pos_index:self.goal_index] = self.goal[0,0:3] # wants goal's info
        qvel[self.joint_vel_index:] = 0
        self.set_state(qpos, qvel)
        
        return self._get_obs()


    def _get_obs(self):
        theta = self.sim.data.qpos.flat[self.joint_pos_index-self.num_revolute:self.joint_pos_index]

        return np.concatenate([
            np.cos(theta), #revolute joint
            np.sin(theta),
            self.sim.data.qpos.flat[:self.joint_pos_index-self.num_revolute], #ball joints
            self.sim.data.qpos.flat[self.joint_pos_index:self.goal_index], # goal position
            self.sim.data.qvel.flat[:self.joint_vel_index],
            self.get_body_com("fingertip")-self.get_body_com("target")
        ])


