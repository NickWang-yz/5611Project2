# 5611Project2

### 1 Mutiple Ropes(45 points)
Mutiple Ropes was implemnted
<img src="VideoSrc/mutiRope.mov" alt="drawing" width="50%"/> <br />

### 2 Cloth Simulation(20 points)

### 3 3D Simulation(10 points)

### 4 High0quality Rendering(5 points)

### 5 Air Darg for Cloth(10 points)

### 6 Ripping/Tearing(10 points)

### 7

### 8
# Difficulties we encoutred. 
One of the hardest difficulties we encountered is how to determine which part of the cloth should be ripped. After discussion, we arrived at three possible methods. One is to check the current wind speed. When the wind speed reaches the curtain threshold, half of the cloth will be ripped off. Second one is to check the velocity of each node. Once it reaches a certain threshold, half of the cloth will be ripped off. The third one, which is the most sophisticated one, is to set a limit on the distance between two adjoint nodes. Once the displacement exceeds that limit, the bond between those two nodes will break. The problem with this is we have to rewrite most of the functions to accommodate this change. We also need to consider the scenarios in which only one of the two strings is attached to the node.
Another issue that arises is how to apply air buoyancy and air resistance on cloth since those two forces also play an important role in cloth simulation. 

# List of tools/Library used
No tools and libraies are used besides what's provided to us. 




