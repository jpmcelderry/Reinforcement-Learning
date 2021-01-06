#Reinforcement Learning

An implementation of Reinforcement Learning using Value Iteration, Q-Learning, and SARSA in Java

Assignment written in Eclipse for Java, Version: 2019-12 (4.14.0)

The program for Intro to Machine Learning Programming Assignment 5 can be run as follows:

>java ML6 L-track.txt

Output files for each input dataset will be written as '<algorithm name>-out.txt'. For example, QLearn-out.txt. For QLearn and SARSA, output contains information on the number of moves required to complete the course progressively over iterations, and an example run is printed to the console. 

All information on value iteration is printed to the console.


The rules for this game are as follows:

The car must move from the starting line to the finish line, with the goal to complete each course in as few moves as possible. The speed of the car in both the x and y directions must be in the range {-5,5} at all times. At each discreet time interval, the following steps occur in this order:

-An acceleration is applied for both the x and y directions, in the range {-1,1} each (accelerations have a 20% chance of failure, resulting in no speed change).
-Acceleration is applied, and the car moves on the track according to its new speed. Collisions are detected using Bresenham's line drawing algorithm.
-Two scenarios for collision handling were tested: either the car stops at the spot of the collision and its speed is reset, or the car must restart the course.
-If at any time the car touches the finish line, it immediately stops and the trial is concluded.
