import numpy as np
from numpy.random import uniform
from numpy.random import randn
from numpy.random import normal
import matplotlib.pyplot as plt
import scipy.stats
from time import sleep
import math
import cv2

def main():
    # Main configuration
    fieldLength = 900
    fieldWidth = 600
    totalParticles = 100
    totalLandmarks = 2

    # Robot position 1D array
    robotPosition = np.empty([2], dtype=int)

    # Landmarks position 2D array
    landmarksPosition = np.empty((totalLandmarks, 2), dtype=int)
    particlesPosition = np.empty((totalParticles, 2), dtype=int)
    distanceRobotToLandmarks = np.empty((totalLandmarks), dtype=int)
    distanceParticlesToLandmarks = np.empty((totalParticles, totalLandmarks), dtype=int)
    particlesWeight= np.empty((totalParticles), dtype=float)

    # Estimate position
    estimatePosition = np.empty([2], dtype=int)

    # Initialize robot position
    robotPosition[0] = uniform(0, fieldLength)
    robotPosition[1] = uniform(0, fieldWidth)
    robotPosition[0] = 50
    robotPosition[1] = 300

    # Initialize landmark position
    landmarksPosition[:, 0] = uniform(0, fieldLength, size=totalLandmarks)
    landmarksPosition[:, 1] = uniform(0, fieldWidth, size=totalLandmarks)

    landmarksPosition[0,0] = 0
    landmarksPosition[0,1] = 170
    landmarksPosition[1,0] = 0
    landmarksPosition[1,1] = 430
    
    # print 'BarelangFC - Particle Filter Robot Localization'
    # print 'Landmark position :'
    # print landmarksPosition
    # print 'Initial robot position :'
    # print robotPosition

    # Create random position of particles
    # print 'Process ==> Create random particles'
    particlesPosition[:, 0] = uniform(0, fieldLength, size=totalParticles)
    particlesPosition[:, 1] = uniform(0, fieldWidth, size=totalParticles)
    # print 'Particles position :'
    # print particlesPosition

    gambarLapangan = np.zeros((800,1100,3), np.uint8)
    


    # Simualte velocity value
    # X Y alpha velocity command from robot
    simVelocity = np.empty([3], dtype=int)
    # Loop every 1 second
    deltaTime = 1 

    # plt.show()    
    # axes = plt.gca()
    # axes.set_xlim(0, 900)
    # axes.set_ylim(0, 600)
    # plt.title('Barelang FC - Particle Filter Simulation')
    # particlesData, = axes.plot(particlesPosition[:,0], particlesPosition[:,1], 'ro')
    # robotPosData, = axes.plot(robotPosition[0], robotPosition[1], 'ro', color='g')
    # estimatePosData, = axes.plot(estimatePosition[0], estimatePosition[1], 'ro', color='b')
    # landmarksPosData, = axes.plot(landmarksPosition[:,0], landmarksPosition[:,1], 'ro', color='y')

    loop = 0

    while True:
        print 'Loop : %d'%loop
        gambarLapangan[:] = (0, 255, 0)
        cv2.rectangle(gambarLapangan,(100,100),(1000,700),(255,255,255),3) # GARIS LUAR
        cv2.rectangle(gambarLapangan,(40,530),(100,270),(255,255,255),3) #garis LUAR gawang kiri
        cv2.rectangle(gambarLapangan,(1000,530),(1060,270),(255,255,255),3) #garis LUAR gawang kiri
        cv2.rectangle(gambarLapangan,(100,650),(200,150),(255,255,255),3) #garis LUAR gawang kiri
        cv2.rectangle(gambarLapangan,(900,650),(1000,150),(255,255,255),3) #garis LUAR gawang kiri
        cv2.line(gambarLapangan,(550,100),(550,700),(255,255,255),3) # garis tengah
        cv2.circle(gambarLapangan,(550,400), 75, (255,255,255), 3) # LINGKARAN TENGAH
        cv2.circle(gambarLapangan,(310,400), 3, (255,255,255), 5)
        cv2.circle(gambarLapangan,(790,400), 3, (255,255,255), 5)
        
        # X = 5, Y = 0, alpha = 0
        simVelocity[0] = 10 
        simVelocity[1] = 5 
        simVelocity[2] = 0 
        # Simulate robot movement
        robotPosition[0] += simVelocity[0]
        robotPosition[1] += simVelocity[1]
        print 'Robot position : (%d,%d)'%(robotPosition[0],robotPosition[1])

        # Predict movement of particles
        # print 'Process ==> Predict movement of particles'
        particlesPosition[:,0] += simVelocity[0]
        particlesPosition[:,1] += simVelocity[1]
        print 'Particles position'
        print particlesPosition
        # Update measurement
        # Measurement distance between robot and landmarks
        for i in range (0,totalLandmarks):
            distanceRobotToLandmarks[i] = math.hypot(robotPosition[0] - landmarksPosition[i,0], robotPosition[1] - landmarksPosition[i,1])
        # print 'Distance Robot to Landmarks :'
        # print distanceRobotToLandmarks
        # Measurement distance between particles and landmarks
        for i in range (0, totalParticles):
            for j in range (0, totalLandmarks):
                distanceParticlesToLandmarks[i,j] = math.hypot(particlesPosition[i,0] - landmarksPosition[j,0], particlesPosition[i,1] - landmarksPosition[j,1])
        # print 'Distance Particles to Landmarks :'
        # print distanceParticlesToLandmarks
        # Calculating weight
        # Initialize particles weight with 1.00
        particlesWeight.fill(1.0)
        for i in range (0, totalParticles):
            for j in range (0, totalLandmarks):
                # mean = jarak robot ke landmark
                # stddev = 5
                particlesWeight[i] *= scipy.stats.norm.pdf(distanceParticlesToLandmarks[i,j],distanceRobotToLandmarks[j],5)
        # Normalize weight
        totalWeight = sum(particlesWeight)
        for i in range (0, totalParticles):
            particlesWeight[i] = particlesWeight[i] / totalWeight
        # print 'Particles Weight'
        # np.set_printoptions(precision=2, suppress=True)
        # print particlesWeight

        # Calculate estimate position
        pos = particlesPosition[:, 0:2]
        mean = np.average(pos, weights=particlesWeight, axis=0)    

        estimatePosition = mean
        print 'Estimate Position : (%d,%d)'%(estimatePosition[0],estimatePosition[1])


        cv2.circle(gambarLapangan,(robotPosition[0]+100, 800 - (robotPosition[1]+100) ), 7, (0,255,255), -1)
        for i in range (0, totalParticles):
            cv2.circle(gambarLapangan,(particlesPosition[i,0]+100, 800 - (particlesPosition[i,1]+100) ), 7, (0,0,255), -1)
        cv2.circle(gambarLapangan,(int(estimatePosition[0])+100, 800 - (int(estimatePosition[1])+100)), 7, (255,0,0), -1)
        cv2.imshow("Barelang FC - Localization ", gambarLapangan)

        # particlesData.set_xdata(particlesPosition[:,0])
        # particlesData.set_ydata(particlesPosition[:,1])
        # robotPosData.set_xdata(robotPosition[0])
        # robotPosData.set_ydata(robotPosition[1])
        # estimatePosData.set_xdata(estimatePosition[0])
        # estimatePosData.set_ydata(estimatePosition[1])
        # landmarksPosData.set_xdata(landmarksPosition[:,0])
        # landmarksPosData.set_ydata(landmarksPosition[:,1])
        # plt.draw()
        # plt.pause(1)
                
        # Resample
        indexHighestWeight = np.argmax(particlesWeight)
        xHighest = particlesPosition[indexHighestWeight,0]
        yHighest = particlesPosition[indexHighestWeight,1]

        _10PercentParticle = int(totalParticles * 0.1)

        for i in range (0, _10PercentParticle):
            particlesPosition[i,0] = uniform(0, fieldLength)
            particlesPosition[i,1] = uniform(0, fieldWidth)

        _90PercentParticle = totalParticles - _10PercentParticle

        for i in range (_10PercentParticle + 1, _90PercentParticle):
            particlesPosition[i,0] = normal(xHighest, 50)
            particlesPosition[i,1] = normal(yHighest, 50) 

        loop += 1     
        k = cv2.waitKey(1000)
        if k == ord('x'):         # X
            break
        
if __name__ == "__main__":
    main()