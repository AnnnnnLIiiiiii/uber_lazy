import cv2
import numpy as np


def map_generate():
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
        orig_list = np.array([bottom_left, top_left, top_right, bottom_right])
        boundary_list = np.array([bottom_left.copy(), top_left.copy(), top_right.copy(), bottom_right.copy()])

        # Translation vector
        T = [10.4, -11.4]
        boundary_list += T

        # Scaling by 10 and Rotate
        Rx_180 = np.array([[1,0],[0,-1]])
        Rz_90 = np.array([[0,1],[-1,0]])
        R = np.dot(Rz_90,Rx_180)

        for i in range(len(boundary_list)):
            boundary_list[i] = np.dot(R, boundary_list[i])*10


        # Translate to cv2 show
        Tcv = np.array([0, 398])

        boundary_list += Tcv
        boudnary_dict = {"bottom_left": boundary_list[0], "top_left": boundary_list[1], "top_right": boundary_list[2],
                         "bottom_right": boundary_list[3]}
        i = 0
        for key in boudnary_dict:
            print("Transform %s point from %s to %s" %(key, orig_list[i], boudnary_dict[key]))
            i += 1

        return boudnary_dict

    class transform2CV:

        def __init__(self, original_coordinates):
            self.original_coordinates = original_coordinates

        @property
        def transform(self):
            T = [10.4, -11.4] # Translation to set bottom left to be origin on map
            Rx_180 = np.array([[1, 0], [0, -1]])
            Rz_90 = np.array([[0, 1], [-1, 0]])
            R = np.dot(Rz_90, Rx_180) # Rotation for cv2 show
            Tcv = np.array([0, 398]) # Translation for cv2 show
            return np.dot(R, (np.array(self.original_coordinates) + T)) * 10 + Tcv


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
    # def transform2CV(cords):
    #     T = [10.4, -11.4]  # Translation to set bottom left to be origin on map
    #     Rx_180 = np.array([[1, 0], [0, -1]])
    #     Rz_90 = np.array([[0, 1], [-1, 0]])
    #     R = np.dot(Rz_90, Rx_180)  # Rotation for cv2 show
    #     Tcv = np.array([0, 398])  # Translation for cv2 show
    #     return np.dot(R,(np.array(cords)+T))*10 + Tcv


    # aw = transform2CV([-8.5, 9.4])
    # print(aw)

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
    F_tl = transform2CV([1.5, -7.0]).transform
    print(F_tl)
    F_br = transform2CV([-8.5, -17.0]).transform
    print(F_br)
    F = draw_rect(F_tl, F_br)

    # Gcords:
    G_tl=transform2CV([-8.5,-4.95]).transform
    print(G_tl)
    G_br=transform2CV([1.5,-5.05]).transform
    print(G_br)
    G = draw_rect(G_tl, G_br)

    # Hcords:
    H_bl=transform2CV([3.45,7.0])
    H_tl=transform2CV([3.45,7.0])
    H_tr=transform2CV([3.55,-3.0])
    H_br=transform2CV([3.45,-3.0])

    # Icords:
    I_bl=transform2CV([3.45,-7.0])
    I_tl=transform2CV([3.45,-7.0])
    I_tr=transform2CV([3.55,-17.0])
    I_br=transform2CV([3.45,-17.0])

    # Jcords:
    J_bl=transform2CV([5.5,-4.95])
    J_tl=transform2CV([5.5,-4.95])
    J_tr=transform2CV([15.5,-5.05])
    J_br=transform2CV([5.5,-5.05])

    # Kcords:
    K_bl=transform2CV([17.5,-4.95])
    K_tl=transform2CV([17.5,-4.95])
    K_tr=transform2CV([27.5,-5.05])
    K_br=transform2CV([17.5,-5.05])

    f = f - A - B - C - D - E - F - G


    # Bcords: [5.5, 7.0] [15.5, 7.0] [15.5, -3.0] [5.5, -3.0]
    # Ccords: [17.5, 7.0] [27.5, 7.0] [27.5, -3.0] [17.5, -3.0]
    # Fcords: [-8.5, -7.0] [1.5, -7.0] [1.5, -17.0] [-8.5, -17.0]
    # Gcords: [-8.5, -4.95] [-8.5, -4.95] [1.5, -5.05] [-8.5, -5.05]
    # Hcords: [3.45, 7.0] [3.45, 7.0] [3.55, -3.0] [3.45, -3.0]
    # Icords: [3.45, -7.0] [3.45, -7.0] [3.55, -17.0] [3.45, -17.0]
    # Jcords: [5.5, -4.95] [5.5, -4.95] [15.5, -5.05] [5.5, -5.05]
    # Kcords: [17.5, -4.95] [17.5, -4.95] [27.5, -5.05] [17.5, -5.05]


    for x in range(0, f.shape[1]):
        for y in range(0, f.shape[0]):
            if f[y,x] != 0:
                f[y, x] = 255

    f = -(f-255)
    cv2.namedWindow("", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("", f.shape[1]*2, f.shape[0]*2)
    cv2.imshow("", f)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return f

