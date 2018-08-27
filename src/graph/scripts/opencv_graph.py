#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import math
import numpy as np
import cv2 as cv

didRun = False
filename = "/home/parallels/catkin_ws/src/graph/scripts/Ledger.txt"
def run():
    file  = open(filename, "r")
    ledgerEntries = file.readlines()
    c = []
    for i in ledgerEntries[1:]:
        i = i.rstrip()
        Q = i[i.find('R')+1:i.find('F')]
        R = i[i.find('F')+1:i.find('@')]
        d = i[i.find('@')+1:i.find('&')]
        a = i[i.find('&')+1:]
        x = [int(Q),int(R),int(d),int(a)]
        c.append(x)



    initx = 500
    inity = 500
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
    img = np.zeros((1000,1000,3), np.uint8)


    for g in clusters.keys():
    	k = clusters[g]

    	while len(k) != len(chain):
	    	for robot in k:
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
		    					[dist,theta1] = connections[x,robot]
		    					nrInfo = connections[robot,x]
		    					theta2 = nrInfo[1]

		    					phi = theta0 + theta1 - theta2 + 180
		    					rh = phi
		    					
                                absX = dist*math.cos(math.radians(theta1))
                                absY = dist*math.sin(math.radians(theta1))


                                relX = absX*math.cos(math.radians(theta0)) - absY*math.sin(math.radians(theta0)) + nx
                                relY = absX*math.sin(math.radians(theta0)) + absY*math.cos(math.radians(theta0)) + ny

                                chain[robot] = [relX,relY,rh]
                                p1 = (int(nx),int(ny))
                                p2 = (int(relX),int(relY))
                                cv.line(img,p1,p2,(255,255,255),1,4,0)
                                # cv.arrowedLine(img,pt1,pt2,(0,255,255),2,8,0,0.2)



	    		else:
	    			chain[robot] = [initx,inity,0]
  			


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
    	cv.arrowedLine(img,pt1,pt2,(0,0,255),2,8,0,0.2)
    	cv.circle(img,(x,y), 5, (0,255,0),-1)
    	cv.putText(img,str(k),(x-5,y+20),cv.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)


    rospy.loginfo("WTF")
    rospy.loginfo("Cluster Data:")                
    rospy.loginfo(clusters)
    rospy.loginfo("Neighbor Data")
    rospy.loginfo(neighbors)
    rospy.loginfo("Chain Data")
    rospy.loginfo(chain)
    rospy.loginfo("Connections Data")
    rospy.loginfo(connections)



    cv.imwrite( "/home/parallels/catkin_ws/src/graph/scripts/graph2.jpg", img );


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
