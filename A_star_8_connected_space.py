import cv2
import numpy as np
import math
import heapq as hpq
from P5_map import map_generate


'''Parameter Initialize'''
scaling = 10 ** 2
r = 3.8
L = 27
clearance = 34.5
grid_size = 2
k_range = 5


def wheel_speed_related():
    mode = int(input("Enter the mode you want to use for rpm pairs (0 for default setting and 1 for user define): "))
    if mode == 1:
        rpm1 = int(input("Enter the first wheel velocity (in rpm): "))
        rpm2 = int(input("Enter the second wheel velocity (in rpm): "))
    else:
        rpm1 = 30 * L / 2 / r
        rpm2 = 30 * L / 3 / r
    rad1 = rpm1 * math.pi / 30
    rad2 = rpm2 * math.pi / 30

    switch = [rad1, rad2, 0]
    term = [rpm1, rpm2, 0]
    actions = {}
    for i in range(3):
        for j in range(3):
            if i != 2 or j != 2:
                actions[(term[i], term[j])] = [switch[i], switch[j]]

    if mode == 1:
        t = (L * math.pi) / (2 * r * max(switch))
    else:
        t = 1

    dt = t / k_range
    map = np.zeros((int(100 / grid_size), int(100 / grid_size), 3))
    goal = [0, (100 - 1) / grid_size]

    print("The time step (t) of the algorithm is: %s" % t)

    def action_update(y, x, theta, cost2come, action):
        # unit cm
        dtheta = (r / L) * (action[0] - action[1]) * dt
        theta += dtheta
        dx = r / 2 * (action[0] + action[1]) * math.cos(theta) * dt / grid_size
        dy = r / 2 * (action[0] + action[1]) * math.sin(theta) * dt / grid_size
        dcost = (dx ** 2 + dy ** 2) ** 0.5

        x += dx
        y += dy
        cost2come += dcost

        return y, x, theta, cost2come

    for action in actions:
        print(action)
        arc = []
        y, x, theta, cost = 50 / grid_size, 5 / grid_size, 0, 0
        y_act, x_act, theta_act, cost_act = y, x, theta, cost
        for k in range(k_range):
            y_act, x_act, theta_act, cost_act = action_update(y_act, x_act, theta_act, cost_act, actions[action])
            arc.append([int(y_act), int(x_act)])
        print('----------')
        print("Action " + str(action))
        print("X travling (%scm): %s" % (grid_size, abs(x - x_act)))
        print("Y travling (%scm): %s" % (grid_size, abs(y - y_act)))
        print("Angle change (degree): %s" % (theta_act * 180 / math.pi))
        cost2come = r * (actions[action][0] + actions[action][1]) * t / 2 / grid_size
        print("cost2come (%scm): %s" % (grid_size, cost2come))
        priority = ((y_act - goal[0]) ** 2 + (x_act - goal[1]) ** 2) ** 0.5 + cost2come
        print("priority (%scm): %s" % (grid_size, priority))
        for k in range(k_range):
            if k != k_range - 1:
                updateMap(arc[k][0], arc[k][1], [0, 255, 0], map,
                          "8 connected space showcase (unit in %dcm)" % grid_size)
            else:
                updateMap(arc[k][0], arc[k][1], [255, 0, 0], map,
                          "8 connected space showcase (unit in %dcm)" % grid_size)
                print("Node on planning map: (y, x) = (%s, %s)" % (arc[k][0] - y, arc[k][1] - x))

    updateMap(int(goal[0]), int(goal[1]), [0, 0, 255], map, "8 connected space showcase (unit in %dcm)" % grid_size)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return actions, t, dt


def updateMap(y, x, color, map_to_show, win_nm):
    map_to_show[y, x] = color
    cv2.namedWindow(win_nm, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_nm, 666, 606)
    cv2.imshow(win_nm, map_to_show)
    cv2.waitKey(1)


def generate_map():
    scaling = 10 ** 2

    map = np.zeros((round(10.1*scaling), round(11.1*scaling)))

    def draw_Minkowski_sum(simplex_img):
        simplex_pts = np.where(simplex_img != 0)
        simplex_pts = np.array(simplex_pts).T
        for pt in simplex_pts:
            simplex_img = cv2.circle(simplex_img, (pt[1], pt[0]), round((L + clearance)/2), 255, -1)
        return simplex_img

    class obstacle:

        def __init__(self, y=0, x=0, r=0, a=0, b=0, top_left=(0,0), bottom_right=(0,0)):
            self.x = round(x*scaling)
            self.y = round(y*scaling)
            self.r = round(r*scaling)
            self.a = round(a*scaling)
            self.b = round(b*scaling)
            self.top_left = (round(top_left[0]*scaling), round(top_left[1]*scaling))
            self.bottom_right = (round(bottom_right[0]*scaling), round(bottom_right[1]*scaling))

        @property
        def circle(self):

            circle = map.copy()
            for x in range(0, circle.shape[1]):
                for y in range(0, circle.shape[0]):
                    if (x - self.x) ** 2 + (y - self.y) ** 2 - self.r ** 2 <= 0:
                        circle[y, x] = 1

            return circle

        @property
        def rectangle(self):

            f1 = map.copy()
            for x in range(0, f1.shape[1]):
                if x >= self.top_left[1]:
                    f1[:, x] = 1

            f2 = map.copy()
            for x in range(0, f2.shape[1]):
                if x <= self.bottom_right[1]:
                    f2[:, x] = 1

            f3 = map.copy()
            for y in range(0, f3.shape[0]):
                if y >= self.top_left[0]:
                    f3[y, :] = 1

            f4 = map.copy()
            for y in range(0, f4.shape[0]):
                if y <= self.bottom_right[0]:
                    f4[y, :] = 1

            rectangle = f1 * f2 * f3 * f4

            return rectangle

    # Circles
    circle_1 = obstacle(y=0.45,x=3.9,r=0.81/2).circle
    circle_2 = obstacle(y=2.74, x=4.38, r=0.81/2).circle
    circle_3 = obstacle(y=10.1-0.35-1.52-0.87, x=4.38, r=0.81/2).circle
    circle_4 = obstacle(y=10.1-0.45, x=3.9, r=0.81/2).circle

    # boundary: top, bottom, left, right
    map[0, :] = 1
    map[map.shape[0]-1, :] = 1
    map[:, 0] = 1
    map[:, map.shape[1]-1] = 1

    map = map + circle_1 + circle_2 + circle_3 + circle_4

    # Rectangles
    rectangle_1 = obstacle(top_left=(0, 11.1-1.92-0.86), bottom_right=(1.83, 11.1-1.92)).rectangle
    rectangle_2 = obstacle(top_left=(0, 11.1-0.84-0.43), bottom_right=(0.91, 11.1-0.84)).rectangle
    rectangle_3 = obstacle(top_left=(3.13, 11.1-3.66), bottom_right=(3.13+0.76, 11.1)).rectangle
    rectangle_4 = obstacle(top_left=(3.13+0.76+0.555, 11.1-0.58), bottom_right=(3.13+0.76+0.555+1.17, 11.1)).rectangle
    rectangle_5 = obstacle(top_left=(3.13+0.76+0.555+1.17, 11.1-0.91),
                           bottom_right=(3.13+0.76+0.555+1.17+0.86, 11.1)).rectangle
    rectangle_6 = obstacle(top_left=(3.13+0.76+0.555+1.17+0.86+0.6725, 11.1-0.58),
                           bottom_right=(3.13+0.76+0.555+1.17+0.86+0.6725+1.17, 11.1)).rectangle
    rectangle_7 = obstacle(top_left=(10.1-0.35-0.76, 11.1-1.83), bottom_right=(10.1-0.35, 11.1)).rectangle
    rectangle_8 = obstacle(top_left=(10.1-0.35, 11.1-4.25), bottom_right=(10.1, 11.1)).rectangle
    rectangle_9 = obstacle(top_left=(10.1-0.35-0.58, 11.1-1.83-0.31-1.17),
                           bottom_right=(10.1-0.35, 11.1-1.83-0.31)).rectangle
    rectangle_10 = obstacle(top_left=(10.1-0.35-1.52, 11.1-1.83-0.31-1.17-0.31-2.74),
                            bottom_right=(10.1-0.35, 11.1-1.83-0.31-1.17-0.31)).rectangle
    rectangle_11 = obstacle(top_left=(10.1-0.35-1.52-0.87-0.81/2-1.83, 4.38),
                            bottom_right=(10.1-0.35-1.52-0.87-0.81/2, 4.38+0.91)).rectangle
    rectangle_12 = obstacle(top_left=(10.1-0.35-1.52-0.78-0.76, 4.38+0.91),
                            bottom_right=(10.1-0.35-1.52-0.78, 4.38+0.91+1.83)).rectangle
    rectangle_13 = obstacle(top_left=(10.1-0.35-1.52-0.8-1.17, 4.38+0.91+1.83+0.725),
                            bottom_right=(10.1-0.35-1.52-0.8, 4.38+0.91+1.83+0.725+1.52)).rectangle

    map = map + rectangle_1 + rectangle_2 + rectangle_3 + rectangle_4 + rectangle_5 + rectangle_6 + rectangle_7 \
          + rectangle_8 + rectangle_9 + rectangle_10 +rectangle_11 + rectangle_12 + rectangle_13

    # Big table
    square = obstacle(top_left=(1, 0.7+1.599/2), bottom_right=(1+1.599, 0.7+1.599*3/2)).rectangle
    left_circle = obstacle(y=1+1.599/2, x=0.7+1.599/2, r=1.599/2).circle
    right_circle = obstacle(y=1+1.599/2, x=0.7+1.599/2*3, r=1.599/2).circle
    map = map + left_circle + right_circle + square
    map = draw_Minkowski_sum(map)

    for y in range(0, map.shape[0]):
        for x in range(0, map.shape[1]):
            if map[y, x] != 0:
                map[y, x] = 255

    return map


def map_initialize(map, grid_size):

    'Resize map by grid size 5 cm: each grid will be a 5cm by 5 cm cell'
    map_size = (int(map.shape[1]/grid_size), int(map.shape[0]/grid_size))
    map = cv2.resize(map, map_size)
    cv2.namedWindow("Setting start and goal points ...", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Setting start and goal points ...", 666, 606)

    print("The resolution of obstacle space is")
    print(map.shape)
    print(map)
    print("where each grid is a 5cm by 5 cm cell")
    '''Generate the GUI map'''
    map_show = np.stack((np.zeros(map.shape), np.zeros(map.shape), np.zeros(map.shape)), axis=2)
    for x in range(0, map_show.shape[1]):
        for y in range(0, map_show.shape[0]):
            if map[y, x] != 0:
                map_show[y, x] = [255,255,255]

    ''' Set start and goal node'''
    while 1:
        cv2.imshow("Setting start and goal points ...", map_show)
        start_pt = eval(input("Enter the coordinate of start point in y,x with top-left as origin, separated by a comma : "))
        x_start, y_start = int(start_pt[1]/grid_size), int(start_pt[0]/grid_size)
        start_pt = (y_start, x_start)
        if 0 <= x_start < map.shape[1] and 0 <= y_start < map.shape[0]:
            if map[y_start, x_start] == 0:
                break
            else: print("Cannot start within a obstacles!")
        else: print("Star point not in the map!")
        cv2.destroyAllWindows()
    updateMap(y_start, x_start, [0, 0, 255], map_show, "Setting start and goal points ...")
    while 1:
        cv2.imshow("Setting start and goal points ...", map_show)
        goal_pt = eval(input("Enter the coordinate of goal point in y,x with top-left as origin, separated by a comma: "))
        x_goal, y_goal = int(goal_pt[1]/grid_size), int(goal_pt[0]/grid_size)
        goal_pt = (y_goal, x_goal)
        if 0 <= x_goal < map.shape[1] and 0 <= y_goal < map.shape[0]:
            if map[y_goal, x_goal] == 0:
                break
            else: print("Cannot go into a obstacles!")
        else: print("Goal point not in the map!")
        cv2.destroyAllWindows()
    updateMap(y_start, x_start, [0, 0, 255], map_show, "Setting start and goal points ...")
    cv2.imshow("Setting start and goal points ...", map_show)
    print("Press any key on the map to continue")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return start_pt, goal_pt, map, map_show


def A_star_8(start_pt, goal_pt, map, map_show, grid_size, actions):
    '''
    version 5
     a. Update the cost2come by arc length
     b. Add wheel velocities pair into dictionary
     c. (minus index for last-in first out for same priority)

     version 6
     a. load actions from the output of wheel_realted()

    '''
    y_start, x_start = start_pt[0], start_pt[1]
    y_goal, x_goal = goal_pt[0], goal_pt[1]
    updateMap(y_start, x_start, [255, 0, 0], map_show, "Exploring")
    updateMap(y_goal, x_goal, [255, 0, 0], map_show, "Exploring")

    start_angle = math.atan((y_goal - y_start) / (x_goal - x_start))
    # threshold = r / 2 * (max(switch) + max(switch)) * t / grid_size
    threshold = 50/grid_size

    def cost2Go(y, x):
        return ((y - y_goal) ** 2 + (x - x_goal) ** 2) ** 0.5

    def find_chcek(node):
        if (node[0] - y_goal) ** 2 + (node[1] - x_goal) ** 2 <= threshold ** 2:
            return True

    def action_update_in(y, x, theta, action, grid_size):
        dtheta = (r / L) * (action[0] - action[1]) * dt
        theta += dtheta
        dx = r / 2 * (action[0] + action[1]) * math.cos(theta) * dt / grid_size
        dy = r / 2 * (action[0] + action[1]) * math.sin(theta) * dt / grid_size
        x += dx
        y += dy


        return y, x, theta

    pq = []
    'Each element in the pq should have the form [priority(total cost), node_xy_tuple, theta, cost2come, parent, w_pair]'
    hpq.heappush(pq, [0 + cost2Go(y_start, x_start), start_pt, 0, 0, None, None])
    'Dictionary as a close set with the form node_dict[node] = [cost2come, theta, parentp, parent_w_pair]'
    node_dict = {}

    while pq:
        ppp, node, theta, cost2come, parent, parent_w_pair = hpq.heappop(pq)
        y, x = float(node[0]), float(node[1])
        # updateMap(int(y), int(x), [255, 0, 0], map_show, "Exploring")
        if node not in node_dict:
            node_dict[node] = [cost2come, theta, parent, parent_w_pair]
            updateMap(int(y), int(x), [0, 255, 0], map_show, "Exploring")
            parent = node
            for action in actions:
                y_act, x_act, theta_act, cost2come_act = y, x, theta, cost2come
                arc = []
                for k in range(k_range):
                    y_act, x_act, theta_act = action_update_in(y_act, x_act, theta_act, actions[action], grid_size)
                    # If the end node or a arc point isn't in a obstacle or out of the boundaries
                    if int(x_act) in range(map.shape[1]) and int(y_act) in range(map.shape[0]) and map[int(y_act), int(x_act)] == 0:
                        arc.append([int(y_act), int(x_act)])
                    else:
                        break
                if len(arc) != k_range:
                    pass
                else:
                    'Set the float exploration into integer node (cell)'
                    child = (int(y_act), int(x_act))
                    'cost2go will always be calculated between node(cell)'
                    cost2go_act = cost2Go(child[0], child[1])
                    'cost2come will always a float arc length'
                    cost2come_update = (r*t/2/grid_size) * (actions[action][0]+actions[action][1])
                    priority_act = cost2come_act + cost2come_update + cost2go_act
                    hpq.heappush(pq, [priority_act, child, theta_act, cost2come_act + cost2come_update, parent, actions[action]])
                    # show the arc and node
                    # for k in range(k_range):
                    #     if k != k_range - 1:
                    #         updateMap(arc[k][0], arc[k][1], [0, 255, 0], map_show, "Exploring")
                    #     else:
                    #         updateMap(arc[k][0], arc[k][1], [255, 0, 0], map_show, "Exploring")
                    if find_chcek(child):
                        node_dict[child] = [cost2come_act + cost2come_update, theta_act, parent, actions[action]]
                        updateMap(y_start, x_start, [0, 255, 255], map_show, "Exploring")
                        updateMap(y_goal, x_goal, [0, 0, 255], map_show, "Exploring")
                        return node_dict, child
        else:
            cost = node_dict[node][0]
            if cost2come < cost:
                '''
                If the current pop node has lower cost2come than what the same node has in the node_dict:
                (1) Update the information of the node in node_dict
                (2) Calculate new priority for the node
                (3) Push the same node with update priority and its information into pq
                '''
                node_dict[node] = [cost2come, theta, parent, parent_w_pair]
                priority = cost2come + cost2Go(node[0], node[1])
                hpq.heappush(pq, [priority, node, theta, cost2come, parent, parent_w_pair])
        # cv2.waitKey(0)
        updateMap(y_start, x_start, [0, 255, 255], map_show, "Exploring")
        updateMap(y_goal, x_goal, [0, 0, 255], map_show, "Exploring")

        # cv2.waitKey(0)
    return None, None




map = map_generate()
print(map)
# map = cv2.bitwise_not(map)
cv2.namedWindow("map", cv2.WINDOW_NORMAL)
cv2.resizeWindow("map", 666, 606)
cv2.imshow("map", map)
print("Press any key on the map to continue")
cv2.waitKey(0)
cv2.destroyAllWindows()

start_point, goal_point, modified_map, map_show = map_initialize(map, grid_size)
actions, t, dt = wheel_speed_related()
gotcha, end_node = A_star_8(start_point, goal_point, modified_map, map_show, grid_size, actions)
cv2.destroyAllWindows()

y_start, x_start = int(start_point[0]), int(start_point[1])
y_goal, x_goal = int(goal_point[0]), int(goal_point[1])
updateMap(y_start, x_start, [0, 255, 255], map_show, "Exploring")
updateMap(y_goal, x_goal, [0, 255, 255], map_show, "Exploring")

if gotcha:
    parent = end_node
    path = [goal_point]
    speed_ls = []
    while parent != start_point:
        child = parent
        _, _, parent, parent_w_pair = gotcha[child]
        path.append(child)
        '''This speed list doesn't consider the initial angle!!! Make sure to mention it in read me file'''
        speed_ls.append(parent_w_pair)
    path.append(start_point)
    path_show = path.copy()
    path_show = [ele for ele in reversed(path_show)]
    speed_ls = [ele for ele in reversed(speed_ls)]

    for node in path_show:
        y, x = int(node[0]), int(node[1])
        updateMap(y, x, [0, 0, 255], map_show, "Exploring")
    updateMap(y_start, x_start, [0, 255, 255], map_show, "Exploring")
    updateMap(y_goal, x_goal, [0, 255, 255], map_show, "Exploring")
    print("The path contains following point: ")
    print(path_show)
    print("The speed list: ")
    print(speed_ls)
    print("Planning End.")
    print("Start node (y,x) is %s with the top-left corner be the origin", start_point[0]*5, start_point[1]*5)
    print("The initial angle is 0 with respect to x axis.")
    print("Goal node (y,x) is %s with the top-left corner be the origin", goal_point[0]*5, goal_point[1]*5)
    print("Press any key on the map to continue simulation. Make sure the v-rep is open and running.")
    cv2.waitKey(0)
    'API to implement simulation'
else:
    print("!!! Cannot find a feasible path !!!")
    cv2.waitKey(0)
cv2.destroyAllWindows()


