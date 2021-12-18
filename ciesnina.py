import matplotlib.pyplot as plt
import random

N = 200
t = 20
hg = 10
yg = 45# random.randrange(10, 90)
h = 100
pom = 0
pom2 = 40

def isStateValid(state):
    x = state.getX()
    y = state.getY()
    if (yg+pom >= y >= pom or h+pom >= y >= pom+hg+yg) and pom2+t >= x >= pom2:
        return False
    else:
        return True

def isStateValid2(state):
    x = state[0]
    y = state[1]
    if (yg+pom >= y >= pom or h+pom >= y >= pom+hg+yg) and pom2+t >= x >= pom2:
        return False
    else:
        return True

def paint_obs(LowB, HighB):
    plt.axis([LowB, HighB, LowB, HighB])
    rec1 = plt.Rectangle([pom2+1, pom+1], t-1, yg-1, color='k')
    rec2 = plt.Rectangle([pom2+1, pom+hg+yg+1], t-1, h-yg-hg-1, color='k')
    plt.gcf().gca().add_artist(rec1)
    plt.gcf().gca().add_artist(rec2)


if __name__ == '__main__':
    paint_obs(0, 100)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
