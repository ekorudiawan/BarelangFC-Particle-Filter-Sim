from flask import Flask, render_template, Response, request
import socket
import sys
import numpy as np
from numpy.random import uniform
from numpy.random import randn
from numpy.random import normal
import matplotlib.pyplot as plt
import scipy.stats
from time import sleep
import math
import cv2
import webbrowser
from time import sleep
import time
from scipy.spatial import distance

# Main configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# Satuan dalam CM
fieldLength = 900
fieldWidth = 600
totalParticles = 300
totalLandmarks = 2

# Global Variable
# Robot position 1D array
robotGlobalPosition = np.zeros((3))
robotLocalPosition = np.zeros((3))

# Estimate position
estimatePosition = np.zeros((3))

# Landmarks position 2D array
landmarksPosition = np.zeros((totalLandmarks, 2))
particlesGlobalPosition = np.zeros((totalParticles, 3))
particlesLocalPosition = np.zeros((totalParticles, 3))
distanceRobotToLandmarks = np.zeros((totalLandmarks))
distanceParticlesToLandmarks = np.zeros((totalParticles, totalLandmarks))
particlesWeight= np.zeros((totalParticles))

# Flask Webserver
##############################################################################
# Definisi ID robot

robotID = 1

app = Flask(__name__)

def shutdown_server():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()

@app.route('/shutdown', methods=['POST'])
def shutdown():
    shutdown_server()
    return 'Server shutting down...'

# http://mattrichardson.com/Raspberry-Pi-Flask/
@app.route('/')
def index():
    """Video streaming home page."""
    templateData = {
        'robotid' : str(robotID),
        }
    return render_template('index.html', **templateData)

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(main(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
#############################################################################

# Convert robot velocity to cm/s
# Loaded from calibrated value
def convertVel(robotId, inputVel):
    outputVel = np.zeros((3))
    if robotId == 1:
        if inputVel[0] == 0:
            outputVel[0] = 0
        else:
            outputVel[0] = 355.27 * inputVel[0] - 0.4811

        if inputVel[1] == 0:   
            outputVel[1] = 0
        else:
            outputVel[1] = 389.51 * inputVel[1] + 0.1839

        if inputVel[2] == 0:
            outputVel[2] = 0
        else:
            outputVel[2] = 124.78 * inputVel[2] + 1.366

    return outputVel

def worldCoorToImageCoor(x, y):
    x = x + 100
    y = 800 - (y + 100)
    return x, y

def main():
    # Initialize robot position
    robotGlobalPosition[0] = uniform(0, fieldLength)
    robotGlobalPosition[1] = uniform(0, fieldWidth)
    robotGlobalPosition[2] = uniform(0, 360)
    # Set initial location of robot
    robotGlobalPosition[0] = 900
    robotGlobalPosition[1] = 0
    robotGlobalPosition[2] = 180 # Heading

    robotLocalPosition[0] = 0
    robotLocalPosition[1] = 0
    robotLocalPosition[2] = robotGlobalPosition[2] # Heading

    # Initialize landmark position
    landmarksPosition[:, 0] = uniform(0, fieldLength, size=totalLandmarks)
    landmarksPosition[:, 1] = uniform(0, fieldWidth, size=totalLandmarks)

    # Gawang lawan
    landmarksPosition[0,0] = 900
    landmarksPosition[0,1] = 170
    landmarksPosition[1,0] = 900
    landmarksPosition[1,1] = 430
    
    # print 'BarelangFC - Particle Filter Robot Localization'
    # print 'Landmark position :'
    # print landmarksPosition
    # print 'Initial robot position :'
    # print robotPosition

    # Create random position of particles
    # print 'Process ==> Create random particles'
    particlesGlobalPosition[:, 0] = uniform(0, fieldLength, size=totalParticles)
    particlesGlobalPosition[:, 1] = uniform(0, fieldWidth, size=totalParticles)
    particlesGlobalPosition[:, 2] = robotGlobalPosition[2]

    particlesLocalPosition[:,0] = 0
    particlesLocalPosition[:,1] = 0
    particlesLocalPosition[:,2] = particlesGlobalPosition[:, 2]
    # print 'Particles position :'
    # print particlesGlobalPosition

    mapImage = np.zeros((800,1100,3), np.uint8)
    
    # Simualte velocity value
    # X Y alpha velocity command from robot
    realVelocity = np.empty([3], dtype=int)
    # Loop every 1 second
    deltaTime = 1 

    # plt.show()    
    # axes = plt.gca()
    # axes.set_xlim(0, 900)
    # axes.set_ylim(0, 600)
    # plt.title('Barelang FC - Particle Filter Simulation')
    # particlesData, = axes.plot(particlesGlobalPosition[:,0], particlesGlobalPosition[:,1], 'ro')
    # robotPosData, = axes.plot(robotPosition[0], robotPosition[1], 'ro', color='g')
    # estimatePosData, = axes.plot(estimatePosition[0], estimatePosition[1], 'ro', color='b')
    # landmarksPosData, = axes.plot(landmarksPosition[:,0], landmarksPosition[:,1], 'ro', color='y')

    # Create UDP client to receive data from kinematic
    # try:
    #     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
    #     sock.bind((UDP_IP, UDP_PORT))
    # except socket.error:
    #     print 'Failed to create socket'
    #     sys.exit()

    # Nilai timing
    nowTime = 0
    lastTime = 0
    loop = 0
    while True:
        # linux
        nowTime = time.clock()
        timer = nowTime - lastTime
        # print 'Timer : ', timer
        if (timer > deltaTime):
            loop += 1
            print 'Runtime : {} s'.format(deltaTime*loop) 
            lastTime = nowTime
            mapImage[:] = (0, 255, 0)
            cv2.rectangle(mapImage,(100,100),(1000,700),(255,255,255),3) # GARIS LUAR
            cv2.rectangle(mapImage,(40,530),(100,270),(255,255,255),3) #garis LUAR gawang kiri
            cv2.rectangle(mapImage,(1000,530),(1060,270),(255,255,255),3) #garis LUAR gawang kiri
            cv2.rectangle(mapImage,(100,650),(200,150),(255,255,255),3) #garis LUAR gawang kiri
            cv2.rectangle(mapImage,(900,650),(1000,150),(255,255,255),3) #garis LUAR gawang kiri
            cv2.line(mapImage,(550,100),(550,700),(255,255,255),3) # garis tengah
            cv2.circle(mapImage,(550,400), 75, (255,255,255), 3) # LINGKARAN TENGAH
            cv2.circle(mapImage,(310,400), 3, (255,255,255), 5)
            cv2.circle(mapImage,(790,400), 3, (255,255,255), 5)

            # Gambar titik 0,0
            textLine = "(0,0)"
            x, y = worldCoorToImageCoor(0,0)
            cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "(0,600)"
            x, y = worldCoorToImageCoor(0,600)
            cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "(900,600)"
            x, y = worldCoorToImageCoor(900,600)
            cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "(900,0)"
            x, y = worldCoorToImageCoor(900,0)
            cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            drawLandmark = True
            if drawLandmark == True:
                for i in range(totalLandmarks):
                    x, y = worldCoorToImageCoor(int(landmarksPosition[i,0]), int(landmarksPosition[i,1]))
                    cv2.circle(mapImage,(x, y), 15, (127,0,127), -1)

            
            # cv2.imwrite("mapImage.jpg", mapImage)
            # break

            # # Get data from kinematic
            # data, _ = sock.recvfrom(1024) # buffer size is 1024 bytes
            # # print "Vel Input : ", data
            velFromKinematic = np.zeros((3))
            # strVelFromKinematic = data.split(",")
            # velFromKinematic[0] = float(strVelFromKinematic[0])
            # velFromKinematic[1] = float(strVelFromKinematic[1])
            # velFromKinematic[2] = float(strVelFromKinematic[2])

            velFromKinematic[0] = 0.02
            velFromKinematic[1] = 0.00
            velFromKinematic[2] = 0.00

            # print "Input Vel : ", velFromKinematic

            # X = 5, Y = 0, alpha = 0
            # realVelocity[0] = 10 
            # realVelocity[1] = 0 
            # realVelocity[2] = 0 

            # Nanti diganti nama jadi real velocity
            realVelocity = convertVel(robotID, velFromKinematic)
            print "Real Velocity : ", realVelocity
            # Simulate robot movement
            # if robotLocalPosition[0] >= 0 and robotLocalPosition[0] < fieldLength and robotLocalPosition[1] >= 0 and robotLocalPosition[1] < fieldWidth:
            robotLocalPosition[0] += realVelocity[0] * deltaTime
            robotLocalPosition[1] += realVelocity[1] * deltaTime
            robotLocalPosition[2] += realVelocity[2] * deltaTime
            # motion model heading
            if robotLocalPosition[2] >= 360:
                robotLocalPosition[2] = 0
            if robotLocalPosition[2] < 0:
                robotLocalPosition[2] = 359
            print 'R Local Pos : ', robotLocalPosition

            # Create matrix rotation
            theta = np.radians(robotLocalPosition[2])
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c,-s), (s, c)))
            # Multiply rotation matrix with robot local position x dan y
            npOutMatMul = np.matmul(R, robotLocalPosition[:2]) 
            # print npOutMatMul

            # if robotGlobalPosition[0] >= 0 and robotGlobalPosition[0] < fieldLength and robotGlobalPosition[1] >= 0 and robotGlobalPosition[1] < fieldWidth:
            robotGlobalPosition[0] += npOutMatMul[0]
            robotGlobalPosition[1] += npOutMatMul[1]
            robotGlobalPosition[2] = robotLocalPosition[2]

            print 'R Global Pos : ', robotGlobalPosition

            # Predict movement of particles
            # print 'Process ==> Predict movement of particles'
            particlesLocalPosition[:,0] += realVelocity[0] * deltaTime
            particlesLocalPosition[:,1] += realVelocity[1] * deltaTime
            particlesLocalPosition[:,2] += realVelocity[2] * deltaTime
            
            # Simulate noise movement of robot with error stddev = 10
            # particlesLocalPosition[:,0] = normal(particlesLocalPosition[:,0], 10)
            # particlesLocalPosition[:,1] = normal(particlesLocalPosition[:,1], 10)
            # particlesLocalPosition[:,2] = normal(particlesLocalPosition[:,1], 3)

            for i in range (0,totalParticles):
                if particlesLocalPosition[i, 2] >= 360:
                    particlesLocalPosition[i, 2] = 0
                if particlesLocalPosition[i, 2] < 0:
                    particlesLocalPosition[i, 2] = 359
                # Create matrix rotation
                theta = np.radians(particlesLocalPosition[i,2])
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c,-s), (s, c)))
                npOutMatMul = np.matmul(R, particlesLocalPosition[i,:2]) 
                particlesGlobalPosition[i,0] += npOutMatMul[0]
                particlesGlobalPosition[i,1] += npOutMatMul[1]
                particlesGlobalPosition[i,2] = particlesLocalPosition[i,2]
                # Jika keluar lapangan random partikel yang baru
                if particlesGlobalPosition[i,0] < 0 or particlesGlobalPosition[i,1] < 0 or particlesGlobalPosition[i,0] > fieldLength or particlesGlobalPosition[i,1] > fieldWidth:
                    particlesGlobalPosition[i, 0] = normal(estimatePosition[0], 20)
                    particlesGlobalPosition[i, 1] = normal(estimatePosition[1], 20)
                    particlesGlobalPosition[i, 2] = robotGlobalPosition[2]
                    particlesLocalPosition[i, 0] = 0
                    particlesLocalPosition[i, 1] = 0
                    particlesLocalPosition[i, 2] = particlesGlobalPosition[i, 2]
            

            # print 'Particles position'
            # print particlesGlobalPosition
            # Update measurement
            # Measurement distance between robot and landmarks
            for i in range (0,totalLandmarks):
                distanceRobotToLandmarks[i] = distance.euclidean([robotGlobalPosition[:2]], [landmarksPosition[i]])
            # print 'distance robot to l', distanceRobotToLandmarks
                # distanceRobotToLandmarks[i] = math.hypot(robotGlobalPosition[0] - landmarksPosition[i,0], robotGlobalPosition[1] - landmarksPosition[i,1])
                # Simulate noise with random gaussian from measurment
                # distanceRobotToLandmarks[i] = normal(distanceRobotToLandmarks[i], 10)
            # print 'Distance Robot to Landmarks :'
            # print distanceRobotToLandmarks
            # Measurement distance between particles and landmarks
            for i in range (0, totalParticles):
                for j in range (0, totalLandmarks):
                    distanceParticlesToLandmarks[i,j] = distance.euclidean([particlesGlobalPosition[i,:2]], [landmarksPosition[j]])
                    # distanceParticlesToLandmarks[i,j] = math.hypot(particlesGlobalPosition[i,0] - landmarksPosition[j,0], particlesGlobalPosition[i,1] - landmarksPosition[j,1])
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
            # pos = particlesGlobalPosition[:, 0:2]

            estimatePosition[:] = np.average(particlesGlobalPosition, weights=particlesWeight, axis=0)

            if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]):
                estimatePosition[0] = -888
                estimatePosition[1] = -888
                estimatePosition[2] = -888
            print estimatePosition
            
            drawParticles = True
            if drawParticles == True:
                for i in range (0, totalParticles):
                    x, y = worldCoorToImageCoor(int(particlesGlobalPosition[i,0]), int(particlesGlobalPosition[i,1]))
                    cv2.circle(mapImage,(x, y), 7, (0,0,255), -1)

            drawSimRobot = True
            if drawSimRobot == True:
                x, y = worldCoorToImageCoor(int(robotGlobalPosition[0]), int(robotGlobalPosition[1]))
                cv2.circle(mapImage,(x, y), 7, (0,255,255), -1)

            drawEstimatePosition = True
            if drawEstimatePosition == True:
                try:
                    x, y = worldCoorToImageCoor(int(estimatePosition[0]), int(estimatePosition[1]))
                    cv2.circle(mapImage,(x, y), 7, (255,0,0), -1)
                except:
                    pass

            textLine = "R{} Global Position (X, Y, Theta) : ({}, {}, {})".format(robotID, int(estimatePosition[0]), int(estimatePosition[1]), int(estimatePosition[2]))
            cv2.putText(mapImage, textLine, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            # Enable GUI Streaming
            showGUI = False
            if showGUI:
                cv2.imshow("Barelang FC - Localization ", mapImage)
            # Enable URL Streaming
            streamUrl = True
            if streamUrl == True:
                cv2.imwrite('stream.jpg', mapImage)
                yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + open('stream.jpg', 'rb').read() + b'\r\n')
            # particlesData.set_xdata(particlesGlobalPosition[:,0])
            # particlesData.set_ydata(particlesGlobalPosition[:,1])
            # robotPosData.set_xdata(robotPosition[0])
            # robotPosData.set_ydata(robotPosition[1])
            # estimatePosData.set_xdata(estimatePosition[0])
            # estimatePosData.set_ydata(estimatePosition[1])
            # landmarksPosData.set_xdata(landmarksPosition[:,0])
            # landmarksPosData.set_ydata(landmarksPosition[:,1])
            # plt.draw()
            # plt.pause(1)
                    
            # Resample
            resample = True
            if resample == True:
                indexHighestWeight = np.argmax(particlesWeight)
                xHighest = particlesGlobalPosition[indexHighestWeight,0]
                yHighest = particlesGlobalPosition[indexHighestWeight,1]
                thetaHighest = particlesGlobalPosition[indexHighestWeight,2]

                _10PercentParticle = int(totalParticles * 0.1)

                for i in range (0, _10PercentParticle):
                    particlesGlobalPosition[i,0] = uniform(0, fieldLength)
                    particlesGlobalPosition[i,1] = uniform(0, fieldWidth)
                    particlesGlobalPosition[i,2] = robotGlobalPosition[2]
                    particlesLocalPosition[i, 0] = 0
                    particlesLocalPosition[i, 1] = 0
                    particlesLocalPosition[i, 2] = particlesGlobalPosition[i, 2]

                _90PercentParticle = totalParticles - _10PercentParticle

                # _90PercentParticle = totalParticles - 0

                for i in range (_10PercentParticle + 1, _90PercentParticle):
                    particlesGlobalPosition[i,0] = normal(estimatePosition[0], 10)
                    particlesGlobalPosition[i,1] = normal(estimatePosition[1], 10) 
                    particlesGlobalPosition[i,2] = robotGlobalPosition[2]
                    particlesLocalPosition[i, 0] = 0
                    particlesLocalPosition[i, 1] = 0
                    particlesLocalPosition[i, 2] = particlesGlobalPosition[i, 2]

if __name__ == "__main__":
    print 'Running BarelangFC - MCL Localization'
    url = "http://127.0.0.1:8888"
    webbrowser.get(using='chromium-browser').open(url)
    app.run(host='127.0.0.1', port=8888, debug=False, threaded=True)
    