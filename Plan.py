from ompl import base as ob
from ompl import geometric as og
import Astar
import matplotlib.pyplot as plt
import sys
import labirynt
import losoweprzeszkody
import ciesnina
from Pathfun import plot_path, plot_path_to_png, paintobs, plot_legend
from Ustawienia import N, dgoal, L, des_time, plansza, plot_paths


def plan(space, planner, time, start, goal, eps=0):
    ss = og.SimpleSetup(space)
    if plansza == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(labirynt.isStateValid))
    elif plansza == 2 or plansza == 3:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(losoweprzeszkody.isStateValid))
    elif plansza == 4:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(ciesnina.isStateValid))
    ss.setStartAndGoalStates(start, goal)
    if planner.lower() == 'rrt':
        ss.setPlanner(og.RRT(ss.getSpaceInformation()))
    elif planner.lower() == 'astar':
        ss.setPlanner(Astar.Astar(ss.getSpaceInformation()))
    elif planner.lower() == 'lbtrrt':
        planner_handle = og.LBTRRT(ss.getSpaceInformation())
        planner_handle.setApproximationFactor(eps)
        ss.setPlanner(planner_handle)
    elif planner.lower() == "rrtconnect":
        ss.setPlanner(og.RRTConnect(ss.getSpaceInformation()))
    elif planner.lower() == "est":
        ss.setPlanner(og.EST(ss.getSpaceInformation()))
    elif planner.lower() == "rrtstar":
        ss.setPlanner(og.RRTstar(ss.getSpaceInformation()))
    elif planner.lower() == "informedrrtstar":
        ss.setPlanner(og.InformedRRTstar(ss.getSpaceInformation()))
    elif planner.lower() == "sst":
        ss.setPlanner(og.SST(ss.getSpaceInformation()))
    else:
        print('Bad planner')
        return 0, None
    OptObj = ob.PathLengthOptimizationObjective(ss.getSpaceInformation())
    ss.setOptimizationObjective(OptObj)
    solved = ss.solve(time)
    if solved:
        # ss.simplifySolution()
        path = ss.getSolutionPath()
        if ss.haveExactSolutionPath():
            found = 1
        else:
            found = 0
        path.interpolate(1000)
        return found, path.printAsMatrix(), path.length()
    else:
        print("No solution found.")
        return 0, None, 0


def set_start_and_goal(start, goal):
    if plansza == 1:
        start[0], start[1] = 10, 5
        goal[0], goal[1] = 80, 75
    elif plansza == 2:
        start[0], start[1] = 80, 15
        goal[0], goal[1] = 50, 75
    elif plansza == 3:
        start[0], start[1] = 90, 10
        goal[0], goal[1] = 5, 80
    elif plansza == 4:
        start[0], start[1] = 35, 25
        goal[0], goal[1] = 65, 25

    else:
        start[0], start[1] = 1, 1
        goal[0], goal[1] = 99, 99


def badanie(time, space, start, goal):
    rrt_path = plan(space, 'rrt', time, start, goal)
    rrtstar_path = plan(space, 'rrtstar', time, start, goal)
    sst_path = plan(space, 'sst', time, start, goal)
    informedrrtstar_path = plan(space, 'informedrrtstar', time, start, goal)
    lbtrrt_12_path = plan(space, 'lbtrrt', time, start, goal, 0.2)
    astar_path = plan(space, 'Astar', time, start, goal)
    if plot_paths == 1:
        plot_path_to_png(rrt_path[1], 'dimgrey', 0, N, 1, 'RRT',
                         'figures/path_RRT.png', start, goal)
        plot_path_to_png(rrtstar_path[1], 'blue', 0, N, 1, 'RRT*',
                         'figures/path_RRTStar.png', start, goal)
        plot_path_to_png(sst_path[1], 'green', 0, N, 2, 'SST',
                         'figures/path_SST.png', start, goal)
        plot_path_to_png(informedrrtstar_path[1], 'red', 0, N, 3, 'InformedRRT*',
                         'figures/path_InformedRRTStar.png', start, goal)
        plot_path_to_png(lbtrrt_12_path[1], 'magenta', 0, N, 4, '1.2',
                         'figures/path_LBTRRT12.png', start, goal)
        plot_path_to_png(astar_path[1], 'darkmagenta', 0, N, 4, 'A*',
                         'figures/path_Astar.png', start, goal)

        plt.figure(5)
        plot_path(rrt_path[1], 'dimgrey', 0, N)
        plot_path(rrtstar_path[1], 'blue', 0, N)
        plot_path(sst_path[1], 'green', 0, N)
        plot_path(informedrrtstar_path[1], 'red', 0, N)
        plot_path(lbtrrt_12_path[1], 'magenta', 0, N)
        plot_path(astar_path[1], 'darkmagenta', 0, N)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.plot(start[0], start[1], 'k*')
        plt.plot(goal[0], goal[1], 'y*')
        paintobs(plansza)
        plot_legend(['RRT', 'RRT*', 'SST', 'I-RRT*', '1.2', 'A*'])
        plt.savefig('figures/paths.png')
    return (rrt_path[0], rrt_path[2]), (rrtstar_path[0], rrtstar_path[2]), (sst_path[0], sst_path[2]),\
           (informedrrtstar_path[0], informedrrtstar_path[2]), (lbtrrt_12_path[0], lbtrrt_12_path[2]), \
           (astar_path[0], astar_path[2])


if __name__ == '__main__':
    s = ob.ReedsSheppStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(N)
    s.setBounds(bounds)
    # Set our robot's starting and goal states to be random
    start_, goal_ = ob.State(s), ob.State(s)
    set_start_and_goal(start_, goal_)
    print("start: ", start_[0], start_[1])
    print("goal: ", goal_[0], goal_[1])
    wynik = list([(0, 0),(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0)])
    wynik[:] = [list(x) for x in wynik]
    nazwa_pliku = "badania/plansza_" + str(plansza) + "_it_" + str(L) + "_" + str(des_time) + "s.txt"
    f = open(nazwa_pliku, 'w')
    sys.stdout = f
    print("time:", des_time)
    for n in range(0, L):
        tmp = badanie(des_time, s, start_, goal_)
        wynik_rrt = tmp[0]
        wynik_rrtstar = tmp[1]
        wynik_sst = tmp[2]
        wynik_informedrrtstar = tmp[3]
        wynik_12 = tmp[4]
        wynik_astar = tmp[5]
        wynik[0][0] = wynik[0][0] + wynik_rrt[0]
        wynik[0][1] = wynik[0][1] + wynik_rrt[1]
        wynik[1][0] = wynik[1][0] + wynik_rrtstar[0]
        wynik[1][1] = wynik[1][1] + wynik_rrtstar[1]
        wynik[2][0] = wynik[2][0] + wynik_sst[0]
        wynik[2][1] = wynik[2][1] + wynik_sst[1]
        wynik[3][0] = wynik[3][0] + wynik_informedrrtstar[0]
        wynik[3][1] = wynik[3][1] + wynik_informedrrtstar[1]
        wynik[4][0] = wynik[4][0] + wynik_12[0]
        wynik[4][1] = wynik[4][1] + wynik_12[1]
        wynik[5][0] = wynik[5][0] + wynik_astar[0]
        wynik[5][1] = wynik[5][1] + wynik_astar[1]
    times = [x[0] / L for x in wynik]
    length = [x[1] / L for x in wynik]
    names = ('rrt', 'rrtstar', 'sst', 'informedrrtstar', '1.2', 'A*')
    print(list(names))
    print(times)
    print(length)
    f.close()
    sys.stdout = sys.__stdout__
    print(list(names))
    print(times)
    print(length)
