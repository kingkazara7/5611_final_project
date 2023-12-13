# 5611_final_project

### Discuss the key algorithms and approaches you used.
Our project shows success in using deep reinforcement learning, which uses a neural network to simulate value function train control policy for robots in a virtual environment and do trajectory planning for robots in the real world. Moreover, we successfully applied the recent DRL algorithm, Proximal Policy Optimization. In this project, We use DRL techniques to train a control policy with a 7-degree-of-freedom manipulator that performs the object-reaching task. We wrote our own Mujoco module, and the penetrator for our model could run with the deep learning library.  
In the algorithm part, we used a sim-to-real strategy, which trains a control policy in the virtual environment. In the virtual environment, we need to represent the robot using the robot model, which has the same action space as the real robot and an object as a target for the robot end-effector to reach. Moreover, We need to map the action space and observation space into the DRL algorithm, which can used during policy training. During training, the target will be randomly generated within Kinova's reachable workspace. Therefore, we limit the target spawn location based on the manipulator workspace settings. 

### Difficulties
During training, the target will randomly generate within Kinova reachable workspace. Therefore, we limit the target spawn location based on the manipulator workspace settings. As shown in the figure, this is the workspace for kinova arm, to make the target location only be spawned in the space, we brute force to use for loop to rotate each joint in the simulation environment and record every time end-effector’s location.
![image]()


### Sketch of the project:  
![image](https://github.com/kingkazara7/5611_final_project/assets/150294493/1c52e821-29d9-4d5b-b07d-fb590d58e477)

### Feedback
The feedback we get from some of our peers is most positive; many are unfamiliar with Mujoco and Deep learning, but they show us some related robot movement links for reference.  

### Relationship between your work and the state-of-the-art both seen in the papers
Inspired by the paper on the reinforcement learning-based reactive obstacle avoidance method for redundant manipulators and open ai Mujoco-py, we created custom Mujoco environments for training robot arms with deep-learning algorithms. Our work provides a robot arm and a gazebo (a ROS simulation environment) training in simpler procedures and absorbing basic ideas of robot arm movements in the class.  

### Future work
In future work, we might extend this project for multiple arms collaboration to pass the red ball. This might achieved by setting the void "red ball: for arm1, and after arm1 moves to the destination with the red ball, arm2 two would grab the red ball to the new destination for the ball in sequence.  
