# Autonomous urban uber system with considering traffic density
# Coworker: Timothy Werder, Shihao Zhang

This project aims to simulate an autonomous passenger lifting system applying in NewYork city. With the knowledge of the map and the statistic traffic information of each road and street, the path planning module will take this information as cost-to-go. The module will also update the cost-to-go with the agent's experience, and use that as the cost-to-go for the next planning process. 

There are two levels of planning: street-to-street planning and within-street planning. A* is chose for both planning algorithm for promising optimal path, while the heuristic function of the first level planning is choosing to be the ratio of the time a car would take to pass through the street currently to the averages, in the manner of "Manhattan Distance" or "L1 norm" of between two states; the heuristic function of the second level planning is choosing to be the "Euclidean Distance". The state space is defined as follow:
1. Each street with one direction and beside one block is treated as a node 
2. A crossroad can be separated into na * nb nodes, where a is the number of directions the street a has; nb is in the similar convention, therefore the possible nodes a crossroad has is in the set (1, 2, 4).

[Demonstration of the state space in street-to-street planning](http://github.com/AnnnnnLIiiiiii/uber_lazy/raw/master/state_space_exploration.png)

Currently the first level planning has been built with simulation envrionment being set up in V-Rep.
