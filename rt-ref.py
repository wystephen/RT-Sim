# This Python file uses the following encoding: utf-8

# if__name__ == "__main__":
#     pass
import numpy as np
import scipy as sp
from matplotlib import pyplot as plt

from numba import jit, float64, njit, prange

from math import *


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def norm(self) -> float:
        return (self.x * self.x + self.y * self.y) ** 0.5

    def normalize(self):
        norm_value = self.norm()
        if norm_value > 1e-10:
            self.x /= norm_value
            self.y /= norm_value

    def __mul__(self, scaler):
        return Point(self.x * scaler, self.y * scaler)


class LineSeg:
    def __init__(self, start_point: Point, ori_vec: Point):
        self.start_point: Point = start_point
        self.ori_vec: Point = ori_vec
        self.line_id = -1
        self.eps = 1e-4

    def get_draw_unit(self) -> plt.Line2D:
        lin = [(self.start_point.x, self.start_point.y),
               (self.start_point.x + self.ori_vec.x, self.start_point.y + self.ori_vec.y)]
        (line_xs, line_ys) = zip(*lin)
        # print(line_xs, line_ys)
        return plt.Line2D(line_xs, line_ys, linewidth=6, color='red')

    def get_xy(self):  # -> [np.ndarray, np.ndarray]:
        x = np.zeros(2)
        y = np.zeros(2)
        x[0] = self.start_point.x
        x[1] = self.start_point.x + self.ori_vec.x
        y[0] = self.start_point.y
        y[1] = self.start_point.y + self.ori_vec.y
        return x, y

    def get_norm_vec(self) -> Point:
        # if abs(self.ori_vec.y) > self.eps and abs(self.ori_vec.x) > self.eps:
        if abs(self.ori_vec.x) > self.eps:
            nl = Point(-1.0 * self.ori_vec.y / self.ori_vec.x, 1.0)
        else:
            nl = Point(1.0, 0.0)

        # if abs(self.ori_vec.y) < self.eps and abs(self.ori_vec.x) > self.eps:
        #     nl = Point(-1, 0)

        # if abs(self.ori_vec.x) < self.eps and abs(self.ori_vec.y) > self.eps:
        #     nl = Point(0, -1)
        # print('cosvec nl:', vec_cos(nl, self.ori_vec))

        assert abs(vec_cos(nl, self.ori_vec)) < self.eps
        return nl


# @jit(nopython=True)
def vec_cos(vec1: Point,
            vec2: Point):
    cross = vec1.x * vec2.x + vec1.y * vec2.y
    norm1 = (vec1.x * vec1.x + vec1.y * vec1.y) ** 0.5
    norm2 = (vec2.x * vec2.x + vec2.y * vec2.y) ** 0.5
    if abs(norm1) < 1e-8 or abs(norm2) < 1e-8:
        return -1.0
    return cross / norm1 / norm2


class Ray:
    def __init__(self, start_point: Point,
                 start_vec: Point):
        self.start_point: Point = start_point
        self.cur_point: Point = self.start_point
        self.cur_vec: Point = start_vec
        self.line_list = list()

    def detect_intersection(self, l: LineSeg) -> [float, Point]:
        '''

        :param l:
        :return: distance of intersection, -1.0 if no intersection
        '''

        self.cur_vec.normalize()

        def solve_intersection(l1: Point,
                               p1: Point,
                               l2: Point,
                               p2: Point):
            b = np.zeros(2)
            A = np.zeros([2, 2])

            b[0] = l1.x - l2.x
            b[1] = l1.y - l2.y

            A[0, 0] = -1.0 * p1.x
            A[0, 1] = p2.x
            A[1, 0] = -1.0 * p1.y
            A[1, 1] = p2.y

            x = np.linalg.solve(A, b)
            return x[0], x[1]

        ray_fac, line_fac = solve_intersection(self.cur_point, self.cur_vec,
                                               l.start_point,
                                               l.ori_vec)
        # print(ray_fac,line_fac)

        if ray_fac > 0.0 and line_fac > 0 and line_fac < 1.0:
            return ray_fac * self.cur_vec.norm(), Point(self.cur_point.x + ray_fac * self.cur_vec.x,
                                                        self.cur_point.y + ray_fac * self.cur_vec.y)
        else:
            return -1.0, Point(0, 0)

    def reflection(self, l: LineSeg,
                   inter_p: Point):
        self.cur_vec.normalize()

        nl = l.get_norm_vec()

        if vec_cos(nl, self.cur_vec) < 0.0:
            nl = nl * -1.0

        idotn2 = (self.cur_vec.x * nl.x + self.cur_vec.y * nl.y)
        reflect_vec = Point(
            self.cur_vec.x - idotn2 * nl.x * 2.0,
            self.cur_vec.y - idotn2 * nl.y * 2.0
        )

        new_trace = LineSeg(Point(self.cur_point.x, self.cur_point.y),
                            Point(inter_p.x - self.cur_point.x,
                                  inter_p.y - self.cur_point.y))
        self.line_list.append(new_trace)

        before_cos = vec_cos(l.ori_vec, self.cur_vec)
        after_cos = vec_cos(l.ori_vec, reflect_vec)
        if abs(before_cos - after_cos) > 1e-8:
            print('l: start_point[', l.start_point.x, l.start_point.y, '],ori vec[',
                  l.ori_vec.x, l.ori_vec.y, ']')
            print('src_vec:', self.cur_vec.x, self.cur_vec.y)
        # print('before cos:', before_cos,
        #       'after cos:', after_cos)

        self.cur_vec = reflect_vec * (1.0 / reflect_vec.norm())
        self.cur_point = inter_p * 1.0

    def reached(self, p: Point):
        new_trace = LineSeg(Point(self.cur_point.x, self.cur_point.y),
                            Point(p.x - self.cur_point.x, p.y - self.cur_point.y))
        self.line_list.append(new_trace)

    def get_xys(self):
        # x = np.zeros(2 + len(self.line_list))
        x = np.zeros(1 + len(self.line_list))
        y = np.zeros_like(x)

        for i in range(len(self.line_list)):
            x[i] = self.line_list[i].start_point.x
            y[i] = self.line_list[i].start_point.y

        final_index = len(self.line_list)
        if final_index > 0:
            final_ls: LineSeg = self.line_list[final_index - 1]
            x[final_index] = final_ls.start_point.x + final_ls.ori_vec.x
            y[final_index] = final_ls.start_point.y + final_ls.ori_vec.y
        # x[final_index + 1] = x[final_index] + self.cur_vec.x
        # y[final_index + 1] = y[final_index] + self.cur_vec.y
        return x, y



class Scene:
    def __init__(self):
        self.obj_list = list()  # List[LineSeg]
        self.latest_id: int = 0
        self.ray_list = list()

    def addLineSeg(self, new_l: LineSeg):
        self.latest_id += 1
        new_l.line_id = self.latest_id * 1
        self.obj_list.append(new_l)
        return True

    def showScene(self, ax):
        # print(self.obj_list)
        # print(len(self.obj_list))

        for i in range(len(self.obj_list)):
            l: LineSeg = self.obj_list[i]
            x, y = l.get_xy()
            ax.plot(x, y, '-r', linewidth=2.0)

    def RT(self, start_point: Point,
           end_point: Point):

        ray_list = list()
        ray_list_valid = list()
        ray_reach_flag = list()
        # initial ray
        for i in np.linspace(-np.pi, np.pi, 18000):
            ray_t = Ray(start_point, Point(cos(i), sin(i)))
            ray_list.append(ray_t)
            ray_list_valid.append(True)
            ray_reach_flag.append(-1.0)

        # Ray tracking in whole scene
        tracing_depth = 0
        while tracing_depth < 5:
            tracing_depth += 1

            for index in range(len(ray_list)):
                ray: Ray = ray_list[index]
                if ray_list_valid[index] == True:

                    # if abs(vec_cos(pv_vec,ray.cur_vec)-1.0) < 0.1/180.0*np.pi:
                    #     ray_reach_flag[index] = True
                    #     ray_list_valid[index] = False
                    # continue

                    dis_list = list()
                    l_seg_list = list()
                    inter_p_list = list()
                    # search each ray
                    for l in self.obj_list:
                        dis, inter_p = ray.detect_intersection(l)
                        if dis > 0:
                            dis_list.append(dis)
                            inter_p_list.append(inter_p)
                            l_seg_list.append(l)
                    if len(dis_list) > 0:
                        min_dis = 100000000
                        min_l = LineSeg(Point(0, 0), Point(0, 0))
                        min_inter_p = Point(0, 0)
                        for i in range(len(dis_list)):
                            if dis_list[i] < min_dis:
                                min_dis = dis_list[i]
                                min_inter_p = inter_p_list[i]
                                min_l = l_seg_list[i]
                        # check if in this point
                        pv_vec = Point(end_point.x - ray.cur_point.x,
                                       end_point.y - ray.cur_point.y)
                        cos_theta = vec_cos(pv_vec, ray.cur_vec)

                        # if cos_theta > cos(0.1 / 180. * np.pi) and pv_vec.norm() < min_dis:
                        if (1.0 - cos_theta ** 2.0) ** 0.5 * pv_vec.norm() < 0.05 and pv_vec.norm() < min_dis:
                            ray_reach_flag[index] = acos(cos_theta)
                            ray_list_valid[index] = False
                            ray.reached(end_point)

                        else:
                            ray.reflection(min_l, min_inter_p)
                    else:
                        ray_list_valid[index] = False

        self.ray_list = ray_list
        self.reached_list = ray_reach_flag

    def draw_RT(self, ax):

        for i in range(len(self.ray_list)):
            if abs(self.reached_list[i]) < 0.1 / 180.0 * np.pi:  # or True:
                ri = i * 1
                xs, ys = self.ray_list[ri].get_xys()
                ax.plot(xs, ys, '-+', label=str(ri))

    # def save_RT(self, dir_name):
    #     for i in range(len(self.ray_list)):
    #         if abs(self.reached_list[i]) < 0.1 / 180.0 * np.pi:
    def get_shortest_length(self) -> float:
        min_length = 10000.0
        for i in range(len(self.ray_list)):
            if abs(self.reached_list[i]) < 0.1 / 180 * np.pi:
                xs, ys = self.ray_list[i].get_xys()
                length = 0.0
                for k in range(xs.shape[0] - 1):
                    length += ((xs[i + 1] - xs[i]) ** 2.0 + (ys[i + 1] - ys[i]) ** 2.0) ** 0.5
                if length < min_length:
                    min_length = length

        if min_length > 1000:
            return -1.0
        else:
            return min_length


if __name__ == '__main__':
    # test_intersection_all()
    s = Scene()
    s.addLineSeg(LineSeg(Point(0, 0), Point(10, 0)))
    s.addLineSeg(LineSeg(Point(0, 0), Point(0, 10)))
    s.addLineSeg(LineSeg(Point(10, 0), Point(0, 10)))
    s.addLineSeg(LineSeg(Point(0, 10), Point(10, 0)))
    s.addLineSeg(LineSeg(Point(5, 2), Point(0, 6)))

    s.addLineSeg(LineSeg(Point(3, 6), Point(0, -3)))
    s.addLineSeg(LineSeg(Point(5, 7), Point(-3, 0)))
    s.addLineSeg(LineSeg(Point(5, 3), Point(3, 0)))
    s.addLineSeg(LineSeg(Point(7, 4), Point(0, 3)))

    fig = plt.figure()
    ax = fig.add_subplot(111)
    s.showScene(ax)

    # fig = plt.figure()
    # ax = fig.add_subplot(111)

    # s.RT(Point(1, 1), Point(6, 8))
    s.RT(Point(1, 1), Point(8, 8))
    # s.RT(Point(1, 1), Point(6, 5))

    s.draw_RT(ax)

    fig = plt.figure()
    ax = fig.add_subplot(111)

    s.showScene(ax)
    s.RT(Point(1, 1), Point(4, 6))
    s.draw_RT(ax)

    try:
        import cPickle as pickle

    except ImportError:
        import pickle

    f = open('tmp_s.file', 'wb')
    pickle.dump(s, f)
    f.close()

    f = open('tmp_s.file', 'rb')
    ts: Scene = pickle.load(f)
    f.close()

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)

    ts.showScene(ax2)
    ts.draw_RT(ax2)

    plt.show()
#
