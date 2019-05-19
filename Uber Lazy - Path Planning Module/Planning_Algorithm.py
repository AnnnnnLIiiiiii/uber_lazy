import numpy as np
import cv2
import heapq as hpq


def map_generate():
    '''
    Generating maps and tables for planning
    :return:
    f: reconstruction of v-rep map with different coordinates system
    f_min: result of Mincowski sum based on f and the setting clarence
    f_name: same size of map f. Each grid contains the relative inforamtion [ave_name, street_name, block_beside]
        of street the grid belong to
    check_table: a dictionary for converting the list of relative street information into absolute integer street
        section
    '''
    def line_eq(p1, p2):
        m = (p1[1] - p2[1]) / (p1[0] - p2[0])
        b = p1[1] - m * p1[0]
        return m, b

    def boundary_transform():

        # Boundaries
        bottom_left = [-10.4, 11.4]
        top_left = [29.4, 11.4]
        top_right = [29.4, -18.9]
        bottom_right = [-10.4, -18.9]
        boundary_list = np.array([bottom_left.copy(), top_left.copy(), top_right.copy(), bottom_right.copy()])

        # Translation vector
        T = [10.4, -11.4]
        boundary_list += T

        # Scaling by 10 and Rotate
        Rx_180 = np.array([[1,0],[0,-1]])
        Rz_90 = np.array([[0,1],[-1,0]])
        R = np.dot(Rz_90, Rx_180)

        for i in range(len(boundary_list)):
            boundary_list[i] = np.dot(R, boundary_list[i])*10


        # Translate to cv2 show
        Tcv = np.array([0, 39.8 * 10])

        boundary_list += Tcv
        boudnary_dict = {"bottom_left": boundary_list[0], "top_left": boundary_list[1], "top_right": boundary_list[2],
                         "bottom_right": boundary_list[3]}

        return boudnary_dict

    class transform2CV:

        def __init__(self, original_coordinates):
            self.original_coordinates = np.array(original_coordinates)
            # for pair in self.original_coordinates:
            #     self.pair = pair

        @property
        def transform(self):
            T = [10.4, -11.4] # Translation to set bottom left to be origin on map
            Rx_180 = np.array([[1, 0], [0, -1]])
            Rz_90 = np.array([[0, 1], [-1, 0]])
            R = np.dot(Rz_90, Rx_180) # Rotation for cv2 show
            Tcv = np.array([0, 39.8 * 10]) # Translation for cv2 show
            result = R @ (self.original_coordinates + T) * 10 + Tcv
            # return (np.dot(R, (np.array(self.original_coordinates) + T)) * scaling + Tcv).astype(int)
            return result.astype(int)

    boud_dict = boundary_transform()
    map = np.zeros((int(boud_dict["bottom_right"][1]), int(boud_dict["bottom_right"][0])))

    def draw_rect(tl, br):
        r1 = map.copy()
        for x in range(0, map.shape[1]):
            if x >= tl[0]:
                r1[:, x] = 1
        r2 = map.copy()
        for y in range(0, f2.shape[0]):
            if y >= tl[1]:
                r2[y, :] = 1
        r3 = map.copy()
        for x in range(0, map.shape[1]):
            if x <= br[0]:
                r3[:, x] = 1
        r4 = map.copy()
        for y in range(0, f2.shape[0]):
            if y <= br[1]:
                r4[y, :] = 1
        r = r1 * r2 * r3 * r4
        return r

    def draw_Minkowski_sum(img):
        # Robot size + clarence is treated as 10*10
        simplex_img = img.copy()
        simplex_img[0,:] = 1
        simplex_img[:, 0] = 1
        simplex_img[-1,:] = 1
        simplex_img[:,-1] = 1
        simplex_pts = np.where(simplex_img != 0)
        simplex_pts = np.array(simplex_pts).T
        for pt in simplex_pts:
            # simplex_img = cv2.circle(simplex_img, (pt[1], pt[0]), round((L + clearance)/2), 255, -1)
            simplex_img = cv2.rectangle(simplex_img, (pt[1]-5, pt[0]-5), (pt[1]+5, pt[0]+5), 255, -1)
        return simplex_img

    # Define garage
    aw = transform2CV([-8.5, 9.4]).transform
    f1 = map.copy()
    for x in range(0, f1.shape[1]):
        if x > aw[0]:
            f1[:, x] = 1
    f2 = map.copy()
    for y in range(0, f2.shape[0]):
        if y >= aw[1]:
            f2[y, :] = 1

    ew = transform2CV([7.7, -18.9]).transform
    fw = transform2CV([29.4, -13.1]).transform

    f3 = map.copy()
    m3, b3 = line_eq(ew, fw)
    for x in range(0, f3.shape[1]):
        for y in range(0, f3.shape[0]):
            if y - m3 * x - b3 >= 0:
                f3[y, x] = 1
    f = (f1 + f2) * f3

    #A
    A_tl = transform2CV([1.5, 7.0]).transform
    A_br = transform2CV([-8.5, -3.0]).transform
    A = draw_rect(A_tl, A_br)

    # Bcords:
    B_tl = transform2CV([15.5, 7.0]).transform
    B_br = transform2CV([5.5, -3.0]).transform
    B = draw_rect(B_tl, B_br)

    # Ccords:
    C_tl = transform2CV([27.5, 7.0]).transform
    C_br = transform2CV([17.5, -3.0]).transform
    C = draw_rect(C_tl, C_br)

    # Dcords:
    D_bl = transform2CV([17.4, -6.9]).transform
    D_tl = transform2CV([27.5, -6.9]).transform
    D_tr = transform2CV([27.5, -11.6]).transform
    D_br = transform2CV([17.4, -14.2]).transform
    d1 = map.copy()
    for x in range(0, d1.shape[1]):
        for y in range(0, d1.shape[0]):
            if x >= D_bl[0] and y <= D_bl[1]:
                d1[y, x] = 1
    d2 = map.copy()
    for y in range(0, d2.shape[0]):
        if y >= D_tl[1]:
            d2[y, :] = 1
    d3 = map.copy()
    md, bd = line_eq(D_tr, D_br)
    for x in range(0, d3.shape[1]):
        for y in range(0, d3.shape[0]):
            if y - md * x - bd >= 0:
                d3[y, x] = 1
    D = d1 * d2 * d3

    # Ecords:
    E_bl = transform2CV([5.4, -6.9]).transform
    E_tl = transform2CV([15.6, -6.9]).transform
    E_tr = transform2CV([15.6, -14.3]).transform
    E_br = transform2CV([5.4, -17]).transform
    e1 = map.copy()
    for x in range(0, e1.shape[1]):
        for y in range(0, e1.shape[0]):
            if x >= E_bl[0] and y <= E_bl[1]:
                e1[y, x] = 1
    e2 = map.copy()
    for y in range(0, e2.shape[0]):
        if y >= E_tl[1]:
            e2[y, :] = 1
    e3 = map.copy()
    me, be = line_eq(E_tr, E_br)
    for x in range(0, e3.shape[1]):
        for y in range(0, e3.shape[0]):
            if y - me * x - be >= 0:
                e3[y, x] = 1
    E = e1 * e2 * e3

    # Fcords:
    F_tl = transform2CV([1.5, -6.9]).transform
    F_br = transform2CV([-8.5, -17.0]).transform
    F = draw_rect(F_tl, F_br)

    # Gcords:
    G_tl = transform2CV([1.5, -4.9]).transform
    G_br = transform2CV([-8.5, -5.1]).transform
    G = draw_rect(G_tl, G_br)

    # Hcords:
    H_tl=transform2CV([3.6, 7.0]).transform
    H_br=transform2CV([3.5, -3.0]).transform
    H = draw_rect(H_tl, H_br)

    # Icords:
    I_tl = transform2CV([3.6, -6.9]).transform
    I_br = transform2CV([3.5, -17.0]).transform
    I = draw_rect(I_tl, I_br)

    # Jcords:
    J_tl = transform2CV([15.5, -4.9]).transform
    J_br = transform2CV([5.5, -5.1]).transform
    J = draw_rect(J_tl, J_br)

    # Kcords:
    K_tl = transform2CV([27.5, -4.9]).transform
    K_br = transform2CV([17.5, -5.1]).transform
    K = draw_rect(K_tl, K_br)

    f = f - A - B - C - D - E - F - G - H - I - J - K

    for x in range(0, f.shape[1]):
        for y in range(0, f.shape[0]):
            if f[y,x] != 0:
                f[y, x] = 255

    f = -(f-255)

    def name_map(map):
        '''

        :param map:
        :return:
        '''
        name_map = np.empty((map.shape[0], map.shape[1], 3), dtype=object)

        # Aves
        FirstAveW = aw[0]
        FirstAveE = A_tl[0]
        name_map[:, FirstAveW:FirstAveE, 0] = '1ave'

        SecondAveSW = A_br[0]
        SecondAveSE = G_tl[0]
        name_map[:, SecondAveSW:SecondAveSE, 0] = '2aveS'

        SecondAveNW = G_br[0]
        SecondAveNE = F_tl[0]
        name_map[:, SecondAveNW:SecondAveNE, 0] = '2aveN'

        # Diagonal part of 3rd Ave
        for y in range(E_br[1]+1):
            x = int((y - bd)/md)
            name_map[y, x:, 0] = '3ave'

        # Straight part of 3rd Ave
        name_map[E_br[1]:, E_br[0]:, 0] = '3ave'

        # Streets
        FirstStrN = A_br[1]
        name_map[FirstStrN:, FirstAveW:, 1] = '1str'
        SecondStrEN = H_br[1]
        SecondStrES = A_tl[1]
        name_map[SecondStrEN:SecondStrES, FirstAveW:, 1] = '2strE'
        SecondStrWN = B_br[1]
        SecondStrWS = H_tl[1]
        name_map[SecondStrWN:SecondStrWS, FirstAveW:, 1] = '2strW'
        ThirdStrN = C_br[1]
        ThirdStrS = B_tl[1]
        name_map[ThirdStrN:ThirdStrS, FirstAveW:, 1] = '3str'
        FourthStrS = C_tl[1]
        name_map[:FourthStrS, FirstAveW:, 1] = '4str'

        # Blocks
        name_map[FirstStrN:, :FirstAveW, 2] = 'Garage'
        name_map[H_br[1]:, aw[0]:G_tl[0], 2] = 'A'
        name_map[H_br[1]:, G_br[0]:, 2] = 'F'
        name_map[C_br[1]:H_tl[1], aw[0]:G_tl[0] + 1, 2] = 'B'
        name_map[C_br[1]:H_tl[1], G_br[0]:, 2] = 'E'
        name_map[:C_br[1], aw[0]:G_tl[0] + 1, 2] = 'C'
        name_map[:C_br[1], G_br[0]:, 2] = 'D'

        return name_map

    check_table = {}
    check_table[(None, None, 'Garage')] = 0
    check_table[('1ave', None, 'A')] = 1
    check_table[('1ave', '2strE', 'A')] = 199
    check_table[('1ave', '2strW', 'B')] = 198
    check_table[('1ave', None, 'B')] = 2
    check_table[('1ave', '3str', 'B')] = 299
    check_table[('1ave', None, 'C')] = 3
    check_table[('1ave', '4str', 'C')] = 399
    check_table[(None, '4str', 'C')] = 4
    check_table[('2aveS', '4str', 'C')] = 499
    check_table[('2aveN', '4str', 'D')] = 498
    check_table[(None, '4str', 'D')] = 5
    check_table[('3ave', '4str', 'D')] = 599
    check_table[('2aveS', None, 'C')] = 6
    check_table[('2aveS', '3str', 'B')] = 699
    check_table[('2aveN', None, 'D')] = 7
    check_table[('3ave', None, 'D')] = 8
    check_table[('3ave', '3str', 'E')] = 899
    check_table[(None, '3str', 'B')] = 9
    check_table[(None, '3str', 'E')] = 10
    check_table[('2aveS', None, 'B')] = 11
    check_table[('2aveS', '2strW', 'B')] = 1199
    check_table[('2aveN', None, 'E')] = 12
    check_table[('2aveN', '3str', 'E')] = 1299
    check_table[('3ave', None, 'E')] = 13
    check_table[('3ave', '2strW', 'E')] = 1399
    check_table[('3ave', '2strE', 'F')] = 1398
    check_table[(None, '2strW', 'B')] = 14
    check_table[(None, '2strW', 'E')] = 15
    check_table[('2aveN', '2strW', 'E')] = 1599
    check_table[(None, '2strE', 'A')] = 16
    check_table[('2aveS', '2strE', 'A')] = 1699
    check_table[(None, '2strE', 'F')] = 17
    check_table[('2aveS', None, 'A')] = 18
    check_table[('2aveN', None, 'F')] = 19
    check_table[('2aveN', '2strE', 'F')] = 1999
    check_table[('3ave', None, 'F')] = 20
    check_table[('3ave', '1str', 'F')] = 2099
    check_table[(None, '1str', 'A')] = 21
    check_table[('1ave', '1str', 'A')] = 2199
    check_table[(None, '1str', 'F')] = 22
    check_table[('2aveN', '1str', 'F')] = 2299
    check_table[('2aveS', '1str', 'A')] = 2298


    def check_pt_inf(x, y, _f_min, _f_name):
        if sum(_f_min[y,x]) == 0:
            print("The point %s is at the street %s" % ((x ,y), _f_name[y, x]))
            print("The numerical name of the street is ", check_table[tuple(_f_name[y ,x])])
            return check_table[tuple(_f_name[y ,x])]
        else:
            print("the point is not avaliable for a car to drive on")
            return None

    f_min = f.copy()
    f_min = draw_Minkowski_sum(f_min)
    f_min = np.concatenate((f_min[..., None], f_min[..., None], f_min[..., None]), axis=2)
    f = np.concatenate((f[..., None], f[..., None], f[..., None]), axis=2)
    f_name = name_map(f)

    # color = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [0, 255, 255], [255, 255, 0], [255, 0, 255]]
    # clr_list = ['blue', 'green', 'red', 'yellow', 'light', 'purple']
    # for i in range(len(color)):
    #     print(clr_list[i])
    #     ptx = np.random.random_integers(0, f.shape[1])
    #     pty = np.random.random_integers(0, f.shape[0])
    #     pt = (ptx, pty)
    #     print("(x,y) = ", pt)
    #     # answer = True
    #     # while not answer:
    #     answer = check_pt_inf(pt[0], pt[1], f_min, f_name)
    #     print(answer)
    #     f = cv2.circle(f, pt, 2, color[i], -1)
    #     f_min = cv2.circle(f_min, pt, 2, color[i], -1)
    #     print("-----------------")

    # cv2.namedWindow("original", cv2.WINDOW_NORMAL)
    # cv2.namedWindow("Minkowski", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("original", f.shape[1]*2, f.shape[0] * 2)
    # cv2.resizeWindow("Minkowski", f.shape[1] * 2, f.shape[0] * 2)
    # cv2.imshow("original", f)
    # cv2.imshow("Minkowski", f_min)
    #
    # while True:
    #     # press Esc to stop the program
    #     k = cv2.waitKey(0)
    #     if k == 27:
    #         break
    # cv2.destroyAllWindows()

    return f, f_min, f_name, check_table


def coordinates_vrep2python(vpt):
    '''
    Convert coordinates in v-rep to coordinates being used in python planning
    :param vpt: float v-rep coordinates
    :return: integer coordinates
    '''
    vpt = np.array(vpt)
    print(vpt)
    T = [10.4, -11.4]  # Translation to set bottom left to be origin on map
    Rx_180 = np.array([[1, 0], [0, -1]])
    Rz_90 = np.array([[0, 1], [-1, 0]])
    R = np.dot(Rz_90, Rx_180)  # Rotation for cv2 show
    Tcv = np.array([0, 39.8 * 10])  # Translation for cv2 show
    _pt = (R @ (vpt + T) * 10 + Tcv).astype(int)
    # print("The point %s on V-Rep is transformed into %s in Python" % (vpt, _pt))
    return _pt


def check_pt_inf(_pt, _f_min, _f_name, _check_table):
    '''
    Function which checks
    1. if _pt is drivable
    2. the street information the _pt belongs to
    :param _pt: a tuple of (x,y) in planning coordinate system
    :param _f_min: Minkowski sum of planning map
    :param _f_name: Map with relative street information
    :param _check_table: table for convert relative information to absolute numerical information
    :return: the numerical street information the point belongs to if it is drivable; None otherwise
    '''
    x, y = _pt[0], _pt[1]
    if sum(_f_min[y, x]) == 0:
        print("The point %s is at the street %s" % ((x, y), _f_name[y, x]))
        key = tuple(_f_name[y, x])
        print("The numerical name of the street is ", _check_table[key])
        return _check_table[tuple(_f_name[y, x])]
    else:
        print("the point is not available for a car to drive on")
        return None


def creat_platform():
    '''
    Creat state space, map used to be shown the planning process, simplified map with relative and numerical
    street information
    :return:
    '''
    _state_space = np.zeros((8, 7, 2))
    # Garage
    _state_space[-1, 0, :] = [0, 1]
    # South-North direction
    # 1st Ave
    _state_space[1:, 1, 0] = -1
    # 2nd Ave South
    _state_space[:-1, 3, 0] = 1
    # 2nd Ave North
    _state_space[1:, 4, 0] = -1
    # 3rd Ave
    _state_space[:-1, 6, 0] = 1

    # East-West direction
    # 1st street
    _state_space[7, 1:, 1] = -1
    # 2nd street East
    _state_space[5, 1:-1, 1] = 1
    # 2nd street West
    _state_space[4, 2:, 1] = -1
    # 3rd street
    _state_space[2, 2:, 1] = -1
    # 4th street
    _state_space[0, 1:-1, 1] = 1

    # map used to be shown the planning process
    map_drawn = _state_space[..., 0] * _state_space[..., 1] + np.sum(_state_space, axis=2)
    map_drawn = 255 * np.where(map_drawn != 0, 1, 0)
    map_drawn = np.concatenate((map_drawn[..., None], map_drawn[..., None], map_drawn[..., None]), axis=2).astype(int)
    map_drawn = np.uint8(map_drawn)

    # Blocks name
    map1 = np.empty((8, 7), dtype=object)
    map1[-1, 0] = 'Garage'
    map1[5:, 1:4] = 'A'
    map1[2:5, 1:4] = 'B'
    map1[0:2, 1:4] = 'C'
    map1[5:, 4:] = 'F'
    map1[2:5, 4:] = 'E'
    map1[0:2, 4:] = 'D'

    # Ave name
    map2 = np.empty((8, 7), dtype=object)
    map2[:, 1] = '1ave'
    map2[:, 3] = '2aveS'
    map2[:, 4] = '2aveN'
    map2[:, 6] = '3ave'

    # Street name
    map3 = np.empty((8, 7), dtype=object)
    map3[7, 1:] = '1str'
    map3[5, 1:] = '2strE'
    map3[4, 1:] = '2strW'
    map3[2, 1:] = '3str'
    map3[0, 1:] = '4str'

    _name_map = np.concatenate((map2[..., None], map3[..., None], map1[..., None]), axis=2)

    numerical_mapping = np.array([[None, 399, 4, 499, 498, 5, 599],
                                  [None, 3, 'C', 6, 7, 'D', 8],
                                  [None, 299, 9, 699, 1299, 10, 899],
                                  [None, 2, 'B', 11, 12, 'E', 13],
                                  [None, 198, 14, 1199, 1599, 15, 1399],
                                  [None, 199, 16, 1699, 1999, 17, 1398],
                                  [None, 1, 'A', 18, 19, 'F', 20],
                                  [0, 2199, 21, 2298, 2299, 22, 2099]])


    return _state_space, map_drawn, _name_map, numerical_mapping


def manhattan_dist(_pt_current, _pt_goal):
    '''
    The function which calculates the Manhattan distance between 2 states
    :param _pt_current: a tuple of a node (y,x)
    :param _pt_goal: a tuple of a node (y,x)
    :return: the Manhattan distance between 2 states
    '''
    _x_diff = abs(_pt_goal[1] - _pt_current[1])
    _y_diff = abs(_pt_goal[0] - _pt_current[0])
    return _x_diff + _y_diff


def updateMap(_node, _map_drawn, _color, _win_nm):
    '''
    Update the map for visualizing planning porcess
    :param _node: a tuple of a node (y,x)
    :param _color: the color in [b, g, r]
    :param _map_drawn: the map to be drawn with the shape (x_map, y_map, 3)
    :param _win_nm: the name of the window to be shown
    :return: None
    '''
    _map_drawn[_node[0], _node[1]] = _color
    cv2.namedWindow(_win_nm, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_win_nm, _map_drawn.shape[1] * 100, _map_drawn.shape[0] * 606)
    cv2.imshow(_win_nm, _map_drawn)
    cv2.waitKey(1)


def A_star(_start, _goal, _state_space, _map_shown, _numerical_map, _traffic_inf=np.zeros(22)):
    '''
    A* algorithm
    :param _start: a tuple of a node (y,x)
    :param _goal: a tuple of a node (y,x)
    :param _state_space: the state space for actions
    :param _traffic_inf: the information of the traffic density, with size equal to the _state_space
    :return: the dictionary of exploration
    '''

    def extract_traffic_density(_node):
        street_num = _numerical_map[_node[0], _node[1]]
        if street_num < 100 and _traffic_inf[street_num - 1] != 1:
            _traffic_heuristic = _traffic_inf[street_num - 1]
        else:
            _traffic_heuristic = 1
        return _traffic_heuristic


    pq = []
    close = {}

    g = 0
    h = manhattan_dist(_start, _goal)
    cost = h + g
    hpq.heappush(pq, [cost, _start, g, None, None])
    while pq:
        _, node, g, parent, parent_action = hpq.heappop(pq)
        if node not in close:
            close[node] = [g, parent, parent_action]
            parent = node
            # parent_action = [None, None]
            for i in range(2):
                if i == 0:
                    parent_action = [_state_space[parent[0], parent[1], i].astype(int), 0]
                else:
                    parent_action = [0, _state_space[parent[0], parent[1], i].astype(int)]
                child = (node[0] + parent_action[0], node[1] + parent_action[1])
                if child == _goal:
                    close[child] = [g + 1, parent, parent_action]
                    return close, _map_shown
                # updateMap(child, _map_shown, [255, 0, 0], 'Exploration')
                h = manhattan_dist(child, _goal)
                traffic = extract_traffic_density(child)
                cost = h + g + traffic
                hpq.heappush(pq, [cost, child, g + 1, parent, parent_action])
        elif g < close[node][0]:
            close[node] = [g, parent]
            h = manhattan_dist(node, _goal)
            cost = g + h
            hpq.heappush(pq, [cost, node, g, parent, parent_action])


def direcion_generate(_path, _actions, _name_map, _check_table):
    '''
    1 = straight
    2 = right
    3 = left
    4 = turn 180

    degrees
    :param _path:
    :param _actions:
    :param _name_map:
    :param _check_table:
    :return:
    '''
    direction_list = [1]
    navigation = [_check_table[tuple(_name_map[_path[0][0], _path[0][1]])]]
    for i in range(1, len(_path)):
        if i < len(_path) - 1:
            direction = np.sum(np.cross(_actions[i-1], _actions[i]))
            if direction == 0:
                direction = 1
            elif direction < 0:
                direction = 2
            else:
                direction = 3
            direction_list.append(direction)
            navigation.append(_check_table[tuple(_name_map[_path[i][0], _path[i][1]])])
        else: navigation.append(_check_table[tuple(_name_map[_path[i][0], _path[i][1]])])

    return direction_list, navigation


def path_1(_dict, _start, _goal, _map_shown, _name_map, _check_table, mode = None, window_name = "Exploring", color=[0, 255, 255]):
    '''
    The function which draws the path from start to goal
    :param _dict: the dictionary of exploration
    :param _start: a tuple of a node (y,x)
    :param _goal: a tuple of a node (y,x)
    :return: None
    '''
    if _dict:
        _, parent, parent_action = _dict[_goal]
        # print(_dict[_goal])
        path = [_goal]
        actions = [parent_action]
        while parent != _start:
            child = parent
            _, parent, parent_action = _dict[child]
            if parent_action == [0, 0]:
                continue
            # print(child, _dict[child])
            path.append(child)
            actions.append(parent_action)
        path.append(_start)
        # path_show = path.copy()
        # actions_use = actions.copy()
        path = [ele for ele in reversed(path)]
        actions = [ele for ele in reversed(actions)]

        directions, navigation = direcion_generate(path, actions, _name_map, _check_table)
        return navigation, directions
    else:
        print("!!! Cannot find a feasible path !!!")
        return None


def path_2(_dict, _start, _goal, _map_shown, _name_map, _check_table, mode = None, window_name = "Exploring", color=[0, 255, 255]):
    '''
    The function which draws the path from start to goal
    :param _dict: the dictionary of exploration
    :param _start: a tuple of a node (y,x)
    :param _goal: a tuple of a node (y,x)
    :return: None
    '''
    if _dict:
        _, parent, parent_action = _dict[_goal]
        # print(_dict[_goal])
        path = [_goal]
        actions = [parent_action]
        while parent != _start:
            child = parent
            _, parent, parent_action = _dict[child]
            if parent_action == [0, 0]:
                continue
            # print(child, _dict[child])
            path.append(child)
            actions.append(parent_action)
        path.append(_start)
        # path_show = path.copy()
        # actions_use = actions.copy()
        path = [ele for ele in reversed(path)]
        actions = [ele for ele in reversed(actions)]

        directions, navigation = direcion_generate(path, actions, _name_map, _check_table)
        if mode == 'show':
            for node in path:
                updateMap(node, _map_shown, color, window_name)
            print("The numerical streets the car should go through are: ")
            print(navigation)
            print("The actions list: ")
            print(directions)
            print("presses Esc to exit")
            while True:
                # press Esc to stop the program
                k = cv2.waitKey(0)
                if k == 27:
                    break
            cv2.destroyAllWindows()
        return navigation, directions
    else:
        print("!!! Cannot find a feasible path !!!")
        return None


if __name__ == "__main__":
    '''
    This part is for testing. 
    
    After maps and platforms being created, the program will ask for the street the passenger at and the destination 
    he or she wants to go. The street that can be set as above parameters are from 1 to 22.
    
    There are three separated trips the agent will take: from garage to pick up the passenger; deliver the passenger 
    to the destination; and go back to the garage from destination.
    
    The first trip is considered to be usual traffic:
    traffic = [1, 1, 1, ...,1]
    traffic.shape = (22,) 
    
    For the second trip, the traffic has the bellow setting:
    Circle around block A and F: traffic are set to be a same number:
    traffic[16-1] = traffic[17-1] = traffic[20-1] = traffic[22-1] = traffic[21-1] = traffic[1-1] = a
    Circle around blcok D, E and F:
    traffic_1[19-1] = traffic_1[12-1] = traffic_1[7-1] = traffic_1[5-1] = traffic_1[8-1] = traffic_1[13-1] = b

    For the third trip, the traffic is the same as second trip.
    
    In the end, the program will show the planning process and return a list of street sections the car should pass and 
    the corresponding actions it should take.
    '''
    state_space, map_shown, name_map, numerical_map = creat_platform()
    original_map, minkowsky_map, street_name_map, check_table = map_generate()
    def gps2street(vpt):
        _pt = coordinates_vrep2python(vpt)
        street_num_inf = check_pt_inf(_pt, minkowsky_map, street_name_map, check_table)
        return street_num_inf


    def pickUp(_passanger_street, mode=None, _traffic_info=np.zeros(22)):
        a, b = np.where(numerical_map == 0)
        print(a, b)
        c, d = np.where(numerical_map == _passanger_street)
        pt_start = (int(a), int(b))
        pt_goal = (int(c), int(d))
        result, _map_shown = A_star(pt_start, pt_goal, state_space, map_shown, numerical_map, _traffic_info)
        _path, _actions = path_2(result, pt_start, pt_goal, _map_shown, name_map, check_table, mode, "Pick Up")

        return _path, _actions


    def deliver(_passanger_street, _destination, mode=None, _traffic_info=np.zeros(22)):
        a, b = np.where(numerical_map == _passanger_street)
        c, d = np.where(numerical_map == _destination)
        pt_start = (int(a), int(b))
        pt_goal = (int(c), int(d))
        result, _map_shown = A_star(pt_start, pt_goal, state_space, map_shown, numerical_map, _traffic_info)
        _path, _actions = path_2(result, pt_start, pt_goal, _map_shown, name_map, check_table, mode, "Deliver",
                               [255, 0, 0])
        return _path, _actions


    def back2garage(_destination, mode=None, _traffic_info=np.zeros(22)):
        a, b = np.where(numerical_map == _destination)
        c, d = np.where(numerical_map == 0)
        pt_start = (int(a), int(b))
        pt_goal = (int(c), int(d))
        result, _map_shown = A_star(pt_start, pt_goal, state_space, map_shown, numerical_map, _traffic_info)
        _path, _actions = path_2(result, pt_start, pt_goal, _map_shown, name_map, check_table, mode, "Back to Garage",
                               [0, 255, 0])
        return _path, _actions

    # gps_passanger = np.array(input("Enter the GPS loaction you are at (use 22.5,-3.975 as exmaple): ").split(",")).astype(float)
    # passanger_street = gps2street(gps_passanger)

    a = 4
    b = 4

    passanger_street = int(input("Enter in the street you are at: "))
    streetnumG = int(input("Enter in the street you would like to go to: "))

    usual_traffic = np.ones(22)
    pick_path, pick_actions = pickUp(passanger_street, mode="show", _traffic_info=usual_traffic)

    # The traffic density is setting to be multiple of the
    traffic = usual_traffic.copy()
    traffic[16-1] = traffic[17-1] = traffic[20-1] = traffic[22-1] = traffic[21-1] = traffic[1-1] = a
    # traffic += traffic_1
    traffic_1 = traffic.copy()
    traffic_1[19-1] = traffic_1[12-1] = traffic_1[7-1] = traffic_1[5-1] = traffic_1[8-1] = traffic_1[13-1] = b

    deliver_path, deliver_actions = deliver(passanger_street, streetnumG, mode="show", _traffic_info=traffic)

    back_path, back_actions = back2garage(streetnumG, mode="show", _traffic_info=traffic)
    print("Pick up path and corresponding actions")
    pick_move = np.vstack((pick_path[:-1], pick_actions)).T
    print(pick_move)
    print("Deliver path and corresponding actions")
    deliver_move = np.vstack((deliver_path[:-1], deliver_actions)).T
    print(deliver_move)
    print("Back to garage")
    back_move = np.vstack((back_path, np.hstack((back_actions, 4)))).T
    print(back_move)