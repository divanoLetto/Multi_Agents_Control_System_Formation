# Analysis and Simulation of Multi-agents System Formation Control.

<img src="https://github.com/divanoLetto/Multi_Agents_Control_System_Formation/blob/master/Images/icon.png" width="100%" height="100%"/>
## Abstract 
This project is focused on the analysis of automatic control laws that allow a multi-agents system of robots to reach desired formations in the 3D space. 
Fist we perform a study on the state-of-art techniques for multi-agents control, then we develop a graphical software application to simulate such methods and compare them to establish benefits and drawbacks of each ones.

## How to run the code 
Run main.py then choose in the graphical interface the starting conditions of the simulation.  
The application let choose the following settings:
- Number of robots: number of agents in the formation.
- Formation: the desired formation. 
- Control law: the desired control law for the agents.
- Agents model: the model of the agents that establish theirs behaviours.
- Step time: sampling time of the simulation.
- Max time: stop time of the simulation.
- Space: 2D (for land vehicles) or 3D (for flying vehicles) simulation.
<br> </br>  
<img src="https://github.com/divanoLetto/MultiAgentsControlSystemFormation/blob/master/Images/presentazione.gif" width="100%" height="100%"/>

## Formations
The application provide some basic formations such as Square, Cubic and Linear formations.   
It's avaible also a tool to draw new desired formations or specify them by 3D cordinates points.
