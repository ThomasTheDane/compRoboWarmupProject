#Warmup Project Writeup

##Which behaviors did you implement?
We ended up implementing a fairly advanced wall following, a person follower, and a obstacle avoider. 

##For each behavior, what strategy did you use to implement the behavior? 
For the wall follower we first attempt to get within the right distance to the wall, then orient ourselves parallel to the wall, and then continue moving parallel to the wall. When moving towards the wall we proportional control for the angle but if we ever get too far from parallel we pause and move more towards parallel, preventing the robot from driving head first into the wall. We use the range can and the associated angles to see how close to parallel we are to the wall and if we are too far we correct using proportional control. 

For the person following….. <toni check it out>
To separate person following from wall following, we implemented methods that would identify walls and other objects depending on their point scan width. If a swathe of points was larger than n particles (n set to around 20), then the robot considered it a wall. If it was less, then that point cloud represented another object. These methods even accounted for small holes (1-5 particles) in the walls and objects. Scan data lists containing only wall point clouds were saved, to be used by the robot for wall following at a certain distance. Although incomplete, the same method could be used for following a person -- a list of data containing only objects that are potentially people could be saved, and the robot could pick out the closest of these objects to follow.

For the obstacle avoidance, we first attempted the vector field approach where each obstacle contributed towards a vector that the robot would attempt to follow. While this was initially promising, the odometry simply turned out to be too inaccurate. Instead we went with a very simple structure by which the robot will turn 90 degrees when it gets too close to an obstacle, turn back when it has passed the obstacle, and then continue on. 

##For the finite state controller, what were the states?  What did the robot do in each state?  How did you combine and how did you detect when to transition between behaviors?
We especially utilized finite state control in the wall follower. These states are described in the first paragraph of the second question. The states were combined mostly through checking the laser scanner and it would transition from different states based on the distance to the wall as well as the angle of the robot compared to the wall. 

##How did you structure your code?
We split each of our behavior into separate files and then within each file had an initialization phase and then a main loop from which we ran our various states and actions. 

##What if any challenges did you face along the way?
As mentioned previously, trying to rely on the odometry of the robot proved practically impossible as well as very difficult to debug. 

##What would you do to improve your project if you had more time?
We would have tried to fully implement the vector field obstacle avoidance, potentially by using a more accurate method of discerning the odometry of the robot. 

##Did you learn any interesting lessons for future robotic programming projects?
We learned a lot about the fickleness of the information that you often get from the robot and how having such incomplete information oftentimes ruins the ideal plan that you had in your head of how something ought to work. 
