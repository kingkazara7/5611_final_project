from mujoco_py import load_model_from_path, MjSim, MjViewer
import mujoco_py
import numpy as np
import pandas as pd



SLIDER_STEP = 50
HINGE_STEP = 30

model = load_model_from_path("./MJCF/kinova_arm.xml")
sim = mujoco_py.MjSim(model)

j1 = sim.model.joint_name2id('joint_1')
j2 = sim.model.joint_name2id('joint_2')
j3 = sim.model.joint_name2id('joint_3')
j4 = sim.model.joint_name2id('joint_4')
j5 = sim.model.joint_name2id('joint_5')
j6 = sim.model.joint_name2id('joint_6')
j7 = sim.model.joint_name2id('joint_7')

j1_limit = (-3.14,3.14)
j3_limit = (-3.14,3.14)
j5_limit = (-3.14,3.14)
j7_limit = (-3.14,3.14)

j2_limit = (-1.72,1.72)
j4_limit = (-1.72,1.72)
j6_limit = (-1.72,1.72)

res = []

for pos1 in range(int(j1_limit[0]*100), int(j1_limit[1]*100), SLIDER_STEP):
    sim.data.qpos[j1] = pos1 / 100.0

    for pos2 in range(int(j2_limit[0]*100), int(j2_limit[1]*100), HINGE_STEP):
        sim.data.qpos[j2] = pos2 / 100.0

        for pos3 in range(int(j3_limit[0]*100), int(j3_limit[1]*100), SLIDER_STEP):
            sim.data.qpos[j3] = pos3 / 100.0

            for pos4 in range(int(j4_limit[0]*100), int(j4_limit[1]*100), HINGE_STEP):
                sim.data.qpos[j4] = pos4 / 100.0

                for pos5 in range(int(j5_limit[0]*100), int(j5_limit[1]*100), SLIDER_STEP):
                    sim.data.qpos[j5] = pos5 / 100.0

                    for pos6 in range(int(j6_limit[0]*100), int(j6_limit[1]*100), HINGE_STEP):
                        sim.data.qpos[j6] = pos6 / 100.0

                        for pos7 in range(int(j7_limit[0]*100), int(j7_limit[1]*100), SLIDER_STEP):
                            sim.data.qpos[j7] = pos7 / 100.0

                            sim.step()
                            # viewer.render()
                            res+=[sim.data.get_body_xpos("robotiq_85_base_link").tolist()]
    


df = pd.DataFrame(res,columns=['x','y','z'])
df.to_csv('kinova_workspace.csv',index=False)
