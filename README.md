# Autonomous urban uber system with considering traffic density
# Coworker: Timothy Werder, Shihao Zhang

This project aims to simulate an autonomous uber system applying in New York city. With the knowledge of the map and the statistic traffic information of each roads and streets, the path planning module will take this information as cost-to-go. The module will also update the cost-to-go with its own experience, and use that as the cost-to-go for next planning process. 

There are two level of planning: street-to-street planning and witin-street planning. Both planning algorithm is chose to be regular A* for promissing optimal paht, while the heuristic funtion of the first level planning is chose to be, in the manner of "Manhattan Distance" or "L1 norm" of between states, the ratio of the current time a car would take to pass through the stree to the averages, the heuristic function of the second level planning is chose to be the "Euclidean Distance". The state space is defined as follow:
1. Each street with one direction and beside one block is treated as a node 
2. A croosroad can be separated into na * nb nodes, where a is the number of directions the street a has; nb is in similar convention, therfore the possible nodes a crossroad has is in the set (1, 2, 4).

[Image text](http://github.com/AnnnnnLIiiiiii/uber_lazy/raw/master/state_space_exploration.png)

