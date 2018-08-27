#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import math
import numpy as np
import cv2 as cv
from graph_tool.all import *



obstacles = 0

didRun = False
filename = "/home/parallels/catkin_ws/src/graph/scripts/Ledger2.txt"
def run():
    global obstacles
    file  = open(filename, "r")
    ledgerEntries = file.readlines()
    c = []
    obs = []
    for i in ledgerEntries[1:]:
        i = i.rstrip()
        Q = i[i.find('R')+1:i.find('F')]
        R = i[i.find('F')+1:i.find('@')]
        d = i[i.find('@')+1:i.find('&')]
        a = i[i.find('&')+1:]
        if R != "999":
            x = [int(Q),int(R),int(d),int(a)]
            c.append(x)
        else:
            x = [int(Q),int(R),int(d),int(a)]
            obs.append(x)




    initx = 0
    inity = 0
    pos = 0
    clusters = {}
    neighbors = {}
    connections = {}
    chain = {}


    #update number of neighbors
    for x in c:
        qr  = x[0]
        fe = x[1]
        if qr not in neighbors.keys():
            neighbors[qr] = [fe]
        else:
            neighbors[qr].append(fe)

    for x in c:
        qr  = x[0]
        fe = x[1]
        connections[qr,fe] = [x[2],x[3]]


    for x in c:
        qr  = x[0]
        fe = x[1]
        qr_loc = 0
        fe_loc = 0
        for g in clusters.keys():
            k = clusters[g]
            if qr in k:
                qr_loc = g
            if fe in k:
                fe_loc = g
        if qr_loc == 0 and fe_loc == 0:
            pos +=1
            clusters[pos] = [qr,fe]
        else:
            if qr_loc != 0 and fe_loc == 0:
                clusters[qr_loc].append(fe)
            elif qr_loc == 0 and fe_loc != 0:
                clusters[fe_loc].append(qr)
            else:
                if qr_loc < fe_loc:
                    clusters[qr_loc].extend(clusters.pop(fe_loc))
                    pos -=1
                elif qr_loc > fe_loc:
                    clusters[fe_loc].extend(clusters.pop(qr_loc))
                    pos -=1


    # Create a black image
    # img = np.zeros((1000,1000,3), np.uint8)
    localized = {}


    for g in clusters.keys():
        k = clusters[g]

        while len(k) != len(chain):
            for robot in k:
                if robot in neighbors.keys():
                    neighborsOfRobot = neighbors[robot]
                    

                    if chain.keys():
                        if not robot in chain.keys():
                            for x in neighborsOfRobot:
                                if x in chain.keys():
                                    nx = chain[x][0]
                                    ny = chain[x][1]
                                    nh = chain[x][2] #theta0
                                    theta0 = nh

                                    #extract relative information
                                    #only if the connection exixts
                                    if (x,robot) in connections.keys():
                                        [dist,theta1] = connections[x,robot]
                                        nrInfo = connections[robot,x]
                                        localized[robot] = True
                                        
                                    else:
                                        [dist,theta1] = connections[robot,x]
                                        nrInfo = connections[robot,x]
                                        localized[robot] = False
                                        
                                    theta2 = nrInfo[1]

                                    phi = theta0 + theta1 - theta2 + 180
                                    rh = phi
                                

                                    absX = dist*math.cos(math.radians(theta1))
                                    absY = dist*math.sin(math.radians(theta1))


                                    relX = absX*math.cos(math.radians(theta0)) - absY*math.sin(math.radians(theta0)) + nx
                                    relY = absX*math.sin(math.radians(theta0)) + absY*math.cos(math.radians(theta0)) + ny
                                    # rospy.loginfo("Robot - %d, Neighbor - %d, absx - %f, absY - %f, relx - %f, rely - %f, theta1 - %f",robot,x,absX,absY,relX,relY,theta1)

                                    chain[robot] = [relX,relY,rh]

                    else:
                        chain[robot] = [initx,inity,0]
                        localized[robot] = True

                else:
                    #extract neighbors of isolated robot
                    # RsThatSeeRobot = []
                    for l in neighbors.keys():
                        if robot in neighbors[l]:
                            [dist,theta1] = connections[l,robot]

                            if l in chain.keys():
                                nx = chain[l][0]
                                ny = chain[l][1]
                                nh = chain[l][2] #theta0
                                theta0 = nh
                                absX = dist*math.cos(math.radians(theta1))
                                absY = dist*math.sin(math.radians(theta1))


                                relX = absX*math.cos(math.radians(theta0)) - absY*math.sin(math.radians(theta0)) + nx
                                relY = absX*math.sin(math.radians(theta0)) + absY*math.cos(math.radians(theta0)) + ny

                                chain[robot] = [relX,relY,0]
                                localized[robot] = False





    # Draw a diagonal blue line with thickness of 5 px
    # cv.line(img,(0,0),(511,511),(255,0,0),5)

    #draw circles for detected robots

    for k in chain.keys():
        x = int(chain[k][0])
        y = int(chain[k][1])
        ang = int(chain[k][2])
        #add arrows indicating direction of robots
        pt1 = (x,y)
        pt2 = (int(x+15*math.cos(math.radians(ang))),int(y+15*math.sin(math.radians(ang))))



    g = Graph()
    oid = g.new_vertex_property("string")
    shape = g.new_vertex_property("int")
    # halo = g.new_vertex_property("bool")
    # haloSize = g.new_vertex_property("double")

    v_robot = {}
    e_robot = {}

    #create dynamic graph

    for v in clusters.keys():
        k = clusters[v]
        for robot in k:
            v_robot[robot] = g.add_vertex()
            oid[v_robot[robot]] = str(robot)

        
    for e in connections.keys():
        [v1,v2] = e
        e_robot[e] = g.add_edge(v_robot[v1],v_robot[v2])

    #update position of vertices

    vpos = g.new_vertex_property("vector<double>")
    v_ang = g.new_vertex_property("double")
    v_indices = (g.get_vertices())

    for r in chain.keys():
        x = int(chain[r][0])
        y = int(chain[r][1])
        a = math.radians(chain[r][2])
        vpos[v_robot[r]] = [x,y]
        v_ang[v_robot[r]] =  a




    for k in localized.keys():
        if localized[k] == True:
            shape[v_robot[k]] = 1
        else:
            shape[v_robot[k]] = 0
            # if k in neighbors.keys():
            #   for l in neighbors[k]:
            #       halo[v_robot[l]] =  True
            #       haloSize[v_robot[l]] =  (15.3/40)*connections[k,l][0]


    #analyze obstacles
    refined_obstacles = {}
    obstacle_positions_pre = {}
    obs_connections = []
    for o in obs:
        obs = 'o' + str(obstacles)
        refined_obstacles[obs] = o
        obstacles+=1
        obs_connections.append((obs,o[0]))

    #calculate relative positions of obstacles
    for k in refined_obstacles.keys():
        a = refined_obstacles[k]
        #extract position of detecting robot
        [rx,ry,theta0] = chain[a[0]]
        dist = a[2]
        theta1 = a[3]

        absX = dist*math.cos(math.radians(theta1))
        absY = dist*math.sin(math.radians(theta1))


        relX = absX*math.cos(math.radians(theta0)) - absY*math.sin(math.radians(theta0)) + rx
        relY = absX*math.sin(math.radians(theta0)) + absY*math.cos(math.radians(theta0)) + ry

        obstacle_positions_pre[k] = [relX,relY]
        # rospy.loginfo("Robot - %d, Obs - %s, absx - %f, absY - %f, relx - %f, rely - %f, theta1 - %f, theta0 - %f",a[0],k,absX,absY,relX,relY,theta1,theta0)

    #calculate distances and filter 
    obstacleFilter = []
    replacementIndex = {}
    distThreshold = 3
    for k in obstacle_positions_pre.keys():
        [x1,y1] = obstacle_positions_pre[k]
        for l in obstacle_positions_pre.keys():
            if l != k and l < k:
                [x2,y2] = obstacle_positions_pre[l]
                ObsDist = math.sqrt(math.pow((x2-x1),2)+math.pow((y2-y1),2))
                # rospy.loginfo("%s to %s is %f units",l,k,ObsDist)
                if ObsDist < distThreshold:
                    obstacleFilter.append([l,k])
                    replacementIndex[k] = l

    #remove duplicate obstacles
    final_obs = obstacle_positions_pre
    for i in obstacleFilter:
        [obs1,obs2] = i
        if obs1 in final_obs and obs2 in final_obs:
            del final_obs[obs2]

    #update final obstacles in graph
    v_obs = {}
    for i in final_obs.keys():
        v_obs[i] = g.add_vertex()
        vpos[v_obs[i]] = [final_obs[i][0],final_obs[i][1]]
        oid[v_obs[i]] = i

    #add edges to obstacle links based on obstacle filter data
    final_obs_connections = []
    for e in obs_connections:
        [obs,linkRobot] = e
        if obs not in final_obs.keys():
            r = replacementIndex[obs]
            final_obs_connections.append((r,linkRobot))
        else:
            final_obs_connections.append(e)



    #add edges to graph
    e_obs = {}
    for e in final_obs_connections:
        [obs,linkRobot] = e
        e_obs = g.add_edge(v_robot[linkRobot],v_obs[obs])






    # rospy.loginfo(v_obs)
    # rospy.loginfo(localized)
    # rospy.loginfo(haloSize.get_array())
    rospy.loginfo("Replacement Index Data:")
    rospy.loginfo(replacementIndex) 
    rospy.loginfo("Obs connection Data:")
    rospy.loginfo(final_obs_connections) 
    rospy.loginfo("Raw Obstacle Data:")
    rospy.loginfo(refined_obstacles) 
    rospy.loginfo("Obstacle connection Data:")
    rospy.loginfo(obs_connections)    
    rospy.loginfo("Pre filter Obstacle Data:")
    rospy.loginfo(obstacle_positions_pre) 
    rospy.loginfo("Post filter Obstacle Data:")
    rospy.loginfo(obstacleFilter) 
    rospy.loginfo("final Obstacle Data:")
    rospy.loginfo(final_obs) 
    rospy.loginfo("Cluster Data:")                
    rospy.loginfo(clusters)
    rospy.loginfo("Neighbor Data")
    rospy.loginfo(neighbors)
    rospy.loginfo("Chain Data")
    rospy.loginfo(chain)
    rospy.loginfo("Connections Data")
    rospy.loginfo(connections)
    graph_draw(g, vertex_text=oid,vertex_size = 25,vertex_rotation = v_ang,
    pos = vpos,edge_marker_size = 10,edge_pen_width = 1 , vertex_shape=shape ,
     vertex_font_size=10, output_size=(700, 700), output="RTgraph.pdf")


    rospy.loginfo("Finished running")

    #create opencv canvas

def talker():
    global didRun
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if not didRun:
            run()
            didRun = True
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
