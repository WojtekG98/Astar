import matplotlib.pyplot as plt

points = [[[0, 10], 27, 2],
          [[25, 20], 2, 15],
          [[25, 35], 20, 2],
          [[50, 35], 2, 25],
          [[0, 75], 25, 2],
          [[30, 75], 20, 2],
          [[50, 25], 25, 2],
          [[50, 20], 2, 30],
          [[80, 20], 25, 2],
          [[0, 50], 15, 2],
          [[20, 50], 5, 2],
          [[32, 50], 20, 2],
          [[25, 35], 2, 17],
          [[32, 20], 18, 2],
          [[50, 65], 45, 2],
          [[93, 30], 2, 35],
          [[60, 45], 25, 2],
          [[60, 35], 2, 25],
          [[80, 30], 15, 2],
          [[59, 0], 2, 18],
          [[59, 80], 2, 20],
          [[5, 30], 20, 2],
          [[19, 60], 2, 15],
          [[25, 65], 20, 2],
          [[34, 57], 2, 18],
          [[24, 75], 2, 20],
          [[14, 85], 2, 15],
          [[39, 82], 2, 18],
          [[49, 75], 2, 18],
          [[39, 5], 2, 25],
          [[85, 74], 2, 20],
          [[74, 85], 20, 2],
          [[69, 65], 2, 18],
          [[84, 5], 2, 15],
          [[69, 10], 2, 25],
          [[77, 0], 2, 15],
          [[76, 52], 2, 13]
          ]

def isStateValid(state):
    x = state.getX()
    y = state.getY()
    if x >= 100 or x <= 0 or y <= 0 or y >= 100:
        return False
    for point in points:
        if point[0][0] + point[1] + 1 >= x >= point[0][0] - 1 and point[0][1] + point[2] + 1 >= y >= point[0][1] - 1:
            return False
    return True

def isStateValid2(state):
    x = state[0]
    y = state[1]
    for point in points:
        if point[0][0] + point[1] + 1 >= x >= point[0][0] - 1 and point[0][1] + point[2] + 1 >= y >= point[0][1] - 1:
            return False
    return True

def paint_obs(LowB, HighB):
    plt.axis([LowB, HighB, LowB, HighB])
    for point in points:
        rec = plt.Rectangle(point[0], point[1], point[2], color='k')
        plt.gcf().gca().add_artist(rec)

if __name__ == '__main__':
    paint_obs(0, 100)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
