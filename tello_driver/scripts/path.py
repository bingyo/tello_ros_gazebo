#!/usr/bin/env python
import rospy

import matplotlib.pyplot as plt
import random
import math
import copy
import time

show_animation = False 

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, searchList, goalFlag,
                 randArea, expandDis=0.4, goalSampleRate=5, maxIter=500, margin=0.5):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.searchList = searchList
        self.margin = margin
        self.goalFlag = goalFlag

    def Planning(self, animation=True):
        """
        Pathplanning
        """

        self.nodeList = [self.start]
        while True:

            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacleList, self.searchList, self.margin):
                continue



            self.nodeList.append(newNode)
            #print("nNodelist:", len(self.nodeList))

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis + self.margin:
                print("Goal!!")
                if not self.goalFlag:
                    self.end.x = newNode.x
                    self.end.y = newNode.y
                break

            #if animation:
            #    self.DrawGraph(rnd)

        if self.goalFlag:
            print(self.end.x)
            print(self.end.y)
            path = [[self.end.x, self.end.y]]
        else:
            path = []
            self.end.x = self.nodeList[len(self.nodeList) - 1].x
            self.end.y = self.nodeList[len(self.nodeList) - 1].y
            path = [[self.end.x, self.end.y]]

        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self, rnd=None):  # pragma: no cover
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-k")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "og", ms=30 * size)

        for (ox, oy, size) in self.searchList:
            plt.plot(ox, oy, "or", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList, searchList, margin):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy) - margin/2
            if d <= size:
                return False  # collision

        for (ox, oy, size) in searchList[1:]:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy) # margin/2# + margin/2
            if d <= size:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main(gx=10.0, gy=10.0):
    global plan_path
    #print("start " + __file__)
    start = time.time()

    # ====Search Path with RRT====
    obstacleList = [
       # (5, 5, 0.7),
       # (2, 7, 0.5),
       # (3, 3, 0.5),
       # (7, 6, 0.5)
    ]  # [x,y,size]
    searchList = [
        (0, 0, 0.0),
        (2, 4, 0.5),
        (7, 5, 0.5),
        #(8, 4, 0.4),
        #(5, 9, 0.5)
    ]  # [x,y,size]
    # Set Initial parameters


    rrtList = [RRT(start=[0, 0], goal=[searchList[0][0], searchList[0][1]],
              randArea=[-2, 15], obstacleList=obstacleList, searchList=searchList, goalFlag=False)]# for search in searchList]

    path = [rrt.Planning(animation=show_animation) for rrt in rrtList]

    obstacleList = [searchList.pop(0)]#[searchList[0]]#[(5, 5, 0.7)]
    searchCount = len(searchList)

    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.seq = 0
    pose.pose.position.z = 1
    for x in path:
            for y in reversed(x):
                pose.header.seq = pose.header.seq + 1
                pose.pose.position.x = y[0]
                pose.pose.position.y = y[1]
                plan_path.poses.append(pose)
                #print(y)
                print(plan_path)

    print("first")

    for count in range(searchCount):
        rrtList = [RRT(start=[path[-1][0][0], path[-1][0][1]], goal=[searchList[0][0], searchList[0][1]],
                  randArea=[-2, 15], obstacleList=obstacleList, searchList=searchList, goalFlag=False)]# for search in searchList]
        path_tmp = [rrt.Planning(animation=show_animation) for rrt in rrtList]
        path = path + path_tmp

        obstacleList.append(searchList.pop(0))

        for x in path_tmp:
            for y in reversed(x):
                pose.header.seq = pose.header.seq + 1
                pose.pose.position.x = y[0]
                pose.pose.position.y = y[1]
                plan_path.poses.append(pose)
                #print(y)
        

        # Draw final path
        #if show_animation:  # pragma: no cover
        #    rrtList[0].DrawGraph()
        #    plt.plot([x for (x, y) in path[count+1]], [y for (x, y) in path[count+1]], '-r')
        #    plt.grid(True)
        #    plt.show()

    rrtList2 = [RRT(start=[path[-1][0][0], path[-1][0][1]], goal=[gx, gy],
              randArea=[-2, 15], obstacleList=obstacleList, searchList=searchList, goalFlag=True)] # for search in searchList]
    path2 = [rrtList2[0].Planning(animation=show_animation)]# for rrt in rrtList2]
    elapsed_time = time.time() - start
    #print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")

    # Draw final path
    #if show_animation:  # pragma: no cover
    #    rrtList2[0].DrawGraph()
    #    plt.plot([x for (x, y) in path2[0]], [y for (x, y) in path2[0]], '-r')
    #    plt.grid(True)
    #    plt.show()
    for x in path2:
            for y in reversed(x):
                pose.header.seq = pose.header.seq + 1
                pose.pose.position.x = y[0]
                pose.pose.position.y = y[1]
                ###plan_path.poses.append(pose)
                #print(y)



tello_path = Path()
plan_path = Path()

def odom_cb(data):
    global tello_path
    global plan_path
    #print(plan_path)
    tello_path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose
    tello_path.poses.append(pose)
    path_pub.publish(tello_path)
    #path_pub.publish(plan_path)

rospy.init_node('path_node')

#odom_sub = rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, odom_cb)
odom_sub = rospy.Subscriber('/tello/orb_slam2_mono/pose', PoseStamped, odom_cb)
path_pub = rospy.Publisher('/tello_path', Path, queue_size=1)
#plan_path_pub = rospy.Publisher('/plan_path', Path, queue_size=1)

if __name__ == '__main__':
    main()
    rospy.spin()
