# 5611_final_project

### Discuss the key algorithms and approaches you used.
Our project shows success in using deep reinforcement learning, which uses a neural network to simulate value function train control policy for robots in a virtual environment and do trajectory planning for robots in the real world. Moreover, we successfully applied the recent DRL algorithm, Proximal Policy Optimization. In this project, We use DRL techniques to train a control policy with a 7-degree-of-freedom manipulator that performs the object-reaching task. We wrote our own Mujoco module, and the penetrator for our model could run with the deep learning library.  
In the algorithm part, we used a sim-to-real strategy, which trains a control policy in the virtual environment. In the virtual environment, we need to represent the robot using the robot model, which has the same action space as the real robot and an object as a target for the robot end-effector to reach. Moreover, We need to map the action space and observation space into the DRL algorithm, which can used during policy training. During training, the target will be randomly generated within Kinova's reachable workspace. Therefore, we limit the target spawn location based on the manipulator workspace settings. 

### Difficulties
During training, the target will randomly generate within Kinova reachable workspace. Therefore, we limit the target spawn location based on the manipulator workspace settings. As shown in the figure, this is the workspace for kinova arm, to make the target location only be spawned in the space, we brute force to use for loop to rotate each joint in the simulation environment and record every time end-effector’s location.
![image](https://github.com/kingkazara7/5611_final_project/assets/150294493/e188a544-a028-448d-9c60-184bc9799b44)

Our reward function combines two components:
- Distance Reward: A positive reward proportional to the reduction in distance between the goal and the end-effector.
- Complexity Penalty: A negative reward proportional to the complexity of the motor actions taken.
Therefore, we need to find optimal reward function parameter before start our training pipeline.
Here are two bad example of reward function parameter.
1. smaller penalty rate for distance.
![reward_tuning(distance penalty)](https://github.com/kingkazara7/5611_final_project/assets/114500333/f3ecd855-1a9b-4041-ba39-e59c4ef62fe3)
2. smaller penalty rate for action
![out](https://github.com/kingkazara7/5611_final_project/assets/114500333/8b929380-aa6d-4e83-8a1f-2131c3fdc14c)



### Sketch of the project:  
![image](https://github.com/kingkazara7/5611_final_project/assets/150294493/1c52e821-29d9-4d5b-b07d-fb590d58e477)

### Feedback
The feedback we get from some of our peers is most positive; many are unfamiliar with Mujoco and Deep learning, but they show us some related robot movement links for reference.  

### Relationship between your work and the state-of-the-art both seen in the papers
Inspired by the paper on the reinforcement learning-based reactive obstacle avoidance method for redundant manipulators and open ai Mujoco-py, we created custom Mujoco environments for training robot arms with deep-learning algorithms. Our work provides a robot arm and a gazebo (a ROS simulation environment) training in simpler procedures and absorbing basic ideas of robot arm movements in the class.  

### Future work
In future work, we might extend this project for multiple arms collaboration to pass the red ball. This might achieved by setting the void "red ball: for arm1, and after arm1 moves to the destination with the red ball, arm2 two would grab the red ball to the new destination for the ball in sequence.  
