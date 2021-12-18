from math import pi

from matplotlib import pyplot as plt

import labirynt
import ciesnina
import losoweprzeszkody
from Ustawienia import N, plansza


def plot_legend(label):
    plt.legend(label, loc='center left', bbox_to_anchor=(1, 0.5), fancybox=True, shadow=True)


def plot_path(path, style, l, h):
    if path is not None:
        plt.axis([l, h, l, h])
        matrix = []
        for line in path.split("\n"):
            tmp = []
            for item in line.split():
                tmp.append(float(item))
            if len(tmp) != 0:
                matrix.append(list(tmp))
        x = []
        y = []
        for item in matrix:
            x.append(item[0])
            y.append(item[1])
        plt.plot(x, y, color=style)


def print_path_txt(path):
    plt.axis([0, N, 0, N])
    matrix = []
    for line in path.split("\n"):
        tmp = []
        for item in line.split():
            tmp.append(float(item))
        if len(tmp) != 0:
            matrix.append(list(tmp))
    for item in matrix:
        item[2] = item[2] * 180 / pi
    return matrix


def plot_path_to_png(path, style, LB, HB, fignum, legend, pathtofile, s, g):
    plt.figure(fignum)
    if path:
        plot_path(path, style, LB, HB)
        plt.plot(s[0], s[1], 'k*')
        plt.plot(g[0], g[1], 'y*')
        paintobs(plansza)
        plot_legend(list((legend, 'start', 'goal')))
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig(pathtofile)


def paintobs(obs):
    if obs == 1:
        labirynt.paint_obs(0, N)
    elif obs == 2 or obs == 3:
        losoweprzeszkody.paint_obs(0, N)
    elif obs == 4:
        ciesnina.paint_obs(0, N)