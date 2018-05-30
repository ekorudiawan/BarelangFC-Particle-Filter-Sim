from flask import Flask, render_template, Response, request
from scipy.spatial import distance
from numpy.random import uniform, normal
import time
import socket
import sys
import numpy as np
import scipy.stats
import math
import cv2
import webbrowser
import os

# Definisi ID robot
robotID = 1

# Set False for real localization
simulationMode = True
headingFromIMU = True

# Main configuration to receive data from kinematic
MAIN_UDP_IP = "127.0.0.1"
MAIN_UDP_IN_PORT = 5005
MAIN_UDP_OUT_PORT = 5006

# Configuration in Cm
fieldLength = 900
fieldWidth = 600

# Particles and landmarks
totalParticles = 50
totalLandmarks = 2
deltaTime = 1

mapImage = np.zeros((800,1100,3), np.uint8)

# Global Variable
robotGlobalPosition = np.zeros((3))
robotInitialPosition = np.zeros((3))
robotLocalPosition = np.zeros((3))

# Landmarks position 2D array
landmarksPosition = np.zeros((totalLandmarks, 2))

# Particles position
particlesGlobalPosition = np.zeros((totalParticles, 3))
particlesLocalPosition = np.zeros((totalParticles, 3))
particlesInitialPosition = np.zeros((totalParticles, 3))

# Estimate position
estimatePosition = np.zeros((3))
estimateLocalPosition = np.zeros((3))
ballEstimatePosition = np.zeros((2))

distanceRobotToLandmarks = np.zeros((totalLandmarks))
distanceParticlesToLandmarks = np.zeros((totalParticles, totalLandmarks))
particlesWeight= np.zeros((totalParticles))

velFromKinematic = np.zeros((3))

realVelocity = np.zeros([3])

app = Flask(__name__)

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
    elif robotId == 5:
        if inputVel[0] == 0:
            outputVel[0] = 0
        else:
            outputVel[0] = 315.95 * inputVel[0] - 0.5579

        if inputVel[1] == 0:
            outputVel[1] = 0
        else:
            outputVel[1] = 338.71 * inputVel[1] + 0.9102

        if inputVel[2] == 0:
            outputVel[2] = 0
        else:
            outputVel[2] = 131.66 * inputVel[2] + 0.9137
    return outputVel

def worldCoorToImageCoor(x, y):
    x = x + 100
    y = 800 - (y + 100)
    return x, y

def main():
    # Set initial location of robot, Just for simulation
    robotInitialPosition[0] = 450
    robotInitialPosition[1] = 300
    robotInitialPosition[2] = 0 # Heading
    robotGlobalPosition[:] = 0
    robotLocalPosition[:] = 0

    # Initialize landmark position (left and right goal pole)
    landmarksPosition[0,0] = 900
    landmarksPosition[0,1] = 170
    landmarksPosition[1,0] = 900
    landmarksPosition[1,1] = 430
  
    velFromKinematic[0] = 0.03
    velFromKinematic[1] = 0.00
    velFromKinematic[2] = 0.00

    imuInitHeading = 0
    imuCurrentHeading = 0

    ballDistance = 20
    panAngle = -45

    defineInitialPosition = False
    if defineInitialPosition == True:
        # Create 90 percent random particles from defined initial position and 10 percent from random uniform
        estimatePosition[0] = 450
        estimatePosition[1] = 300
        estimatePosition[2] = 0 # Zero = Heading to goal opponent

        _10PercentParticle = int(totalParticles * 0.1)

        for i in range (0, _10PercentParticle):
            particlesInitialPosition[i,0] = uniform(0, fieldLength)
            particlesInitialPosition[i,1] = uniform(0, fieldWidth)
            particlesInitialPosition[i,2] = uniform(0, 360)
            particlesGlobalPosition[i,:] = 0
            particlesLocalPosition[i,:] = 0

        _90PercentParticle = totalParticles - _10PercentParticle

        for i in range (_10PercentParticle + 1, _90PercentParticle):
            particlesInitialPosition[i,0] = normal(estimatePosition[0], 50)
            particlesInitialPosition[i,1] = normal(estimatePosition[1], 50)
            particlesInitialPosition[i,2] = normal(estimatePosition[2], 10)
            particlesGlobalPosition[i,:] = 0
            particlesLocalPosition[i,:] = 0
    else:
        # Create random uniform position of particles
        particlesInitialPosition[:,0] = uniform(0, fieldLength, size=totalParticles)
        particlesInitialPosition[:,1] = uniform(0, fieldWidth, size=totalParticles)
        particlesInitialPosition[:,2] = uniform(0, 360, size=totalParticles) 

    # Zero all global and local position of particles
    particlesGlobalPosition[:,:] = 0
    particlesLocalPosition[:,:] = 0

    # Create UDP client to receive data from kinematic
    if simulationMode == False:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
            sock.bind((MAIN_UDP_IP, MAIN_UDP_IN_PORT))
        except socket.error:
            print 'Failed to create socket'
            sys.exit()

    # Timing value
    nowTime = 0
    lastTime = 0
    loop = 0

    while True:
        nowTime = time.clock()
        timer = nowTime - lastTime
        halfDeltaTime = deltaTime / 2.00
        # Update every 0.5 * deltatime
        if timer > halfDeltaTime:
            lastTime = nowTime
            loop += 1
            print 'Runtime : {} s'.format(deltaTime*loop) 

            mapFromFile = True
            if mapFromFile == True:
                # image tidak clear
                mapImage[:] = cv2.imread('mapImage.jpg')
            else:
                mapImage[:] = (0, 255, 0)
                cv2.rectangle(mapImage,(100,100),(1000,700),(255,255,255),3) # Garis Luar
                cv2.rectangle(mapImage,(40,530),(100,270),(255,255,255),3) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage,(1000,530),(1060,270),(255,255,255),3) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage,(100,650),(200,150),(255,255,255),3) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage,(900,650),(1000,150),(255,255,255),3) # Garis Luar Gawang Kiri
                cv2.line(mapImage,(550,100),(550,700),(255,255,255),3) # Garis Tengah
                cv2.circle(mapImage,(550,400), 75, (255,255,255), 3) # Lingkaran Tengah
                cv2.circle(mapImage,(310,400), 3, (255,255,255), 5)
                cv2.circle(mapImage,(790,400), 3, (255,255,255), 5)

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
            if simulationMode == False:
                data, _ = sock.recvfrom(1024) # buffer size is 1024 bytes
                strDataFromKinematic = data.split(",")
                velFromKinematic[0] = float(strDataFromKinematic[0]) # Vx
                velFromKinematic[1] = float(strDataFromKinematic[1]) # Vy
                velFromKinematic[2] = float(strDataFromKinematic[2]) # Va

            realVelocity[:] = convertVel(robotID, velFromKinematic)

            # Kalau keluar lapangan random posisi robot yg baru
            if robotGlobalPosition[0] < 0 or robotGlobalPosition[0] >= fieldLength or robotGlobalPosition[1] < 0 or robotGlobalPosition[1] >= fieldWidth:
                robotInitialPosition[0] = uniform(0, fieldLength)
                robotInitialPosition[1] = uniform(0, fieldWidth)
                robotInitialPosition[2] = uniform(0, 360)
                robotGlobalPosition[:] = 0
                robotLocalPosition[:] = 0

            # Simulate robot movement
            robotLocalPosition[0] += realVelocity[0] * deltaTime
            robotLocalPosition[1] += realVelocity[1] * deltaTime
            if headingFromIMU:
                if simulationMode == False:
                    imuInitHeading = float(strDataFromKinematic[5])
                    imuCurrentHeading = float(strDataFromKinematic[6])
                robotLocalPosition[2] = imuCurrentHeading - imuInitHeading
            else:
                robotLocalPosition[2] += realVelocity[2] * deltaTime

            # Motion model heading
            if robotLocalPosition[2] >= 360:
                robotLocalPosition[2] = robotLocalPosition[2] - 360
            if robotLocalPosition[2] < 0:
                robotLocalPosition[2] = 360 + robotLocalPosition[2]

            if headingFromIMU:
                angle = robotLocalPosition[2]
            else:
                angle = robotInitialPosition[2] + robotLocalPosition[2]

            if angle >= 360:
                angle = angle - 360
            if angle < 0:
                angle = 360 + angle

            # Create matrix rotation
            theta = np.radians(angle)
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c,-s), (s, c)))
            npOutMatMul = np.matmul(R, robotLocalPosition[:2]) 
            robotGlobalPosition[0] = npOutMatMul[0] + robotInitialPosition[0]
            robotGlobalPosition[1] = npOutMatMul[1] + robotInitialPosition[1]
            robotGlobalPosition[2] = angle
 
            # Predict movement of particles
            particlesLocalPosition[:,0] += realVelocity[0] * deltaTime
            particlesLocalPosition[:,1] += realVelocity[1] * deltaTime
            if headingFromIMU:
                if simulationMode == False:
                    imuInitHeading = float(strDataFromKinematic[5])
                    imuCurrentHeading = float(strDataFromKinematic[6])
                particlesLocalPosition[:,2] = imuCurrentHeading - imuInitHeading
            else:
                particlesLocalPosition[:,2] += realVelocity[2] * deltaTime
            
            # Simulate noise movement of robot with error stddev = 10
            simulateNoiseMovement = False
            if simulateNoiseMovement == True:
                particlesLocalPosition[:,0] = normal(particlesLocalPosition[:,0], 10)
                particlesLocalPosition[:,1] = normal(particlesLocalPosition[:,1], 10)
                particlesLocalPosition[:,2] = normal(particlesLocalPosition[:,2], 3)

            # Calculate position of particles in global coordinat
            updateParticlesMovement = True
            if updateParticlesMovement == True:
                for i in range (0,totalParticles):
                    if particlesLocalPosition[i,2] >= 360:
                        particlesLocalPosition[i,2] = particlesLocalPosition[i,2] - 360
                    if particlesLocalPosition[i,2] < 0:
                        particlesLocalPosition[i,2] = 360 + particlesLocalPosition[i,2]
                    # Kalau pakai data IMU dianggap tidak ada rotasi 
                    if headingFromIMU:
                        angle = particlesLocalPosition[i,2]
                    # Kalau pakai yaw rate ditambahkan dulu dengan initial position
                    else:
                        angle = particlesInitialPosition[i,2] + particlesLocalPosition[i,2]
                    # Check limit bearing
                    if angle >= 360:
                        angle = angle - 360
                    if angle < 0:
                        angle = 360 + angle
                    theta = np.radians(angle)
                    c, s = np.cos(theta), np.sin(theta)
                    R = np.array(((c,-s), (s, c)))
                    npOutMatMul = np.matmul(R, particlesLocalPosition[i,:2]) 
                    particlesGlobalPosition[i,0] = npOutMatMul[0] + particlesInitialPosition[i,0]
                    particlesGlobalPosition[i,1] = npOutMatMul[1] + particlesInitialPosition[i,1]
                    particlesGlobalPosition[i,2] = angle

                    # Jika keluar lapangan random partikel yang baru di sekitar estimate position terakhir
                    if particlesGlobalPosition[i,0] < 0 or particlesGlobalPosition[i,1] < 0 or particlesGlobalPosition[i,0] >= fieldLength or particlesGlobalPosition[i,1] >= fieldWidth:
                        # Cek kalau estimatenya tdk nan atau inf
                        if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                            particlesInitialPosition[i,0] = uniform(0, fieldLength)
                            particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                            particlesInitialPosition[i,2] = uniform(0, 3650)
                            particlesGlobalPosition[i,:] = 0
                            particlesLocalPosition[i,:] = 0
                        else:
                            particlesInitialPosition[i,0] = normal(estimatePosition[0], 50)
                            particlesInitialPosition[i,1] = normal(estimatePosition[1], 50)
                            particlesInitialPosition[i,2] = normal(estimatePosition[2], 10)
                            particlesGlobalPosition[i,:] = 0
                            particlesLocalPosition[i,:] = 0

            # Measurement distance between robot and landmarks
            if simulationMode == True:
                for i in range (0,totalLandmarks):
                    distanceRobotToLandmarks[i] = distance.euclidean([robotGlobalPosition[:2]], [landmarksPosition[i]])
            else:
                distanceRobotToLandmarks[0] = float(strDataFromKinematic[3])
                distanceRobotToLandmarks[1] = float(strDataFromKinematic[4])

            # Resample only when get valid distance data 
            if distanceRobotToLandmarks[0] > 0 and distanceRobotToLandmarks[1] > 0:
                resample = True
            else:
                resample = False

            # Measurement distance between particles and landmarks
            for i in range (0, totalParticles):
                for j in range (0, totalLandmarks):
                    distanceParticlesToLandmarks[i,j] = distance.euclidean([particlesGlobalPosition[i,:2]], [landmarksPosition[j]])

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

            # Calculate estimate position
            # Jika ada perintah resample
            if resample == True:
                estimatePosition[:] = np.average(particlesGlobalPosition, weights=particlesWeight, axis=0)
                estimateLocalPosition[:] = 0
            # Jka tidak update estimate position dengan data dari kinematik
            else:
                estimateLocalPosition[0] += realVelocity[0] * deltaTime
                estimateLocalPosition[1] += realVelocity[1] * deltaTime
                if headingFromIMU:
                    if simulationMode == False:
                        imuInitHeading = float(strDataFromKinematic[5])
                        imuCurrentHeading = float(strDataFromKinematic[6])
                    estimateLocalPosition[2] = imuCurrentHeading - imuInitHeading
                else:
                    estimateLocalPosition[2] += realVelocity[2] * deltaTime

                if estimateLocalPosition[2] >= 360:
                    estimateLocalPosition[2] = estimateLocalPosition[2] - 360
                if estimateLocalPosition[2] < 0:
                    estimateLocalPosition[2] = 360 + estimateLocalPosition[2]

                if headingFromIMU:
                    angle = estimateLocalPosition[2]
                else:
                    angle = estimatePosition[2] + estimateLocalPosition[2]

                if angle >= 360:
                    angle = angle - 360
                if angle < 0:
                    angle = 360 + angle
                theta = np.radians(angle)
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c,-s), (s, c)))
                npOutMatMul = np.matmul(R, estimateLocalPosition[:2]) 
                estimatePosition[0] = npOutMatMul[0] + estimatePosition[0]
                estimatePosition[1] = npOutMatMul[1] + estimatePosition[1]
                estimatePosition[2] = angle
            
            # Mark as -888 if result infinity or nan
            if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                estimatePosition[:] = -888
                ballEstimatePosition[:] = -888
                # random uniform lagi
            else:
                if simulationMode == False:
                    ballDistance = float(strDataFromKinematic[7]) # Jarak bola dari main
                    panAngle = float(strDataFromKinematic[8])
                # ini masih dalam koordinat lokal robot
                ballEstimatePosition[0] = ballDistance
                ballEstimatePosition[1] = 0
                # ini nanti ditambahkan sama posisi servo pan
                headHeading = estimatePosition[2] + panAngle
                if headHeading  >= 360:
                    headHeading = headHeading  - 360
                if headHeading  < 0:
                    headHeading  = 360 + headHeading 
                theta = np.radians(headHeading)
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c,-s), (s, c)))
                npOutMatMul = np.matmul(R, ballEstimatePosition[:2])
                ballEstimatePosition[0] = npOutMatMul[0] + estimatePosition[0]
                ballEstimatePosition[1] = npOutMatMul[1] + estimatePosition[1]

            print "Robot Global Position : ", robotGlobalPosition
            print "Robot Estimate Position : ", estimatePosition
            print "Ball Estimate Position : ", ballEstimatePosition
            # Kirim X, Y, Theta Robot, Ball X, Ball Y
            if simulationMode == False:
                msgToMainProgram = "{},{},{},{},{}".format(int(estimatePosition[0]), int(estimatePosition[1]), int(estimatePosition[2]), int(ballEstimatePosition[0]), int(ballEstimatePosition[1])) 
                sock.sendto(msgToMainProgram, (MAIN_UDP_IP, MAIN_UDP_OUT_PORT))
            
            drawParticles = True
            if drawParticles == True:
                for i in range (0, totalParticles):
                    x, y = worldCoorToImageCoor(int(particlesGlobalPosition[i,0]), int(particlesGlobalPosition[i,1]))
                    cv2.circle(mapImage,(x, y), 7, (0,0,255), -1)

            drawSimRobot = True
            if drawSimRobot == True:
                x, y = worldCoorToImageCoor(int(robotGlobalPosition[0]), int(robotGlobalPosition[1]))
                cv2.circle(mapImage,(x, y), 12, (0,255,255), -1)

            drawEstimatePosition = True
            if drawEstimatePosition == True:
                try:
                    x, y = worldCoorToImageCoor(int(estimatePosition[0]), int(estimatePosition[1]))
                    cv2.circle(mapImage,(x, y), 12, (255,0,0), -1)
                except:
                    pass
            
            drawBallEstimatePosition = True
            if drawBallEstimatePosition == True:
                try:
                    x, y = worldCoorToImageCoor(int(ballEstimatePosition[0]), int(ballEstimatePosition[1]))
                    cv2.circle(mapImage,(x, y), 10, (255,255,0), -1)
                except:
                    pass

            textLine = "R%d Velocity : (%.2f, %.2f, %.2f)"%(robotID, realVelocity[0], realVelocity[1], realVelocity[2])
            cv2.putText(mapImage, textLine, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "R{} Local Position : ({}, {}, {})".format(robotID, int(robotLocalPosition[0]), int(robotLocalPosition[1]), int(robotLocalPosition[2]))
            cv2.putText(mapImage, textLine, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "R{} Global Position : ({}, {}, {})".format(robotID, int(robotGlobalPosition[0]), int(robotGlobalPosition[1]), int(robotGlobalPosition[2]))
            cv2.putText(mapImage, textLine, (300,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            textLine = "R{} Estimate Position : ({}, {}, {})".format(robotID, int(estimatePosition[0]), int(estimatePosition[1]), int(estimatePosition[2]))
            cv2.putText(mapImage, textLine, (300,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 127), 1, cv2.LINE_AA)

            # Enable GUI Streaming
            showGUI = False
            if showGUI:
                cv2.imshow("Barelang FC - Localization ", mapImage)
            # Enable URL Streaming
            streamUrl = True
            if streamUrl == True:
                smallMapImage = cv2.resize(mapImage, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
                cv2.imwrite('stream.jpg', smallMapImage)
                yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + open('stream.jpg', 'rb').read() + b'\r\n')
                    
            # Resample
            if resample == True:
                indexHighestWeight = np.argmax(particlesWeight)
                xHighest = particlesGlobalPosition[indexHighestWeight,0]
                yHighest = particlesGlobalPosition[indexHighestWeight,1]
                thetaHighest = particlesGlobalPosition[indexHighestWeight,2]

                _10PercentParticle = int(totalParticles * 0.1)

                for i in range (0, _10PercentParticle):
                    particlesInitialPosition[i,0] = uniform(0, fieldLength)
                    particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                    particlesInitialPosition[i,2] = uniform(0, 360)
                    particlesGlobalPosition[i,:] = 0
                    particlesLocalPosition[i,:] = 0

                _90PercentParticle = totalParticles - _10PercentParticle

                for i in range (_10PercentParticle + 1, _90PercentParticle):
                    # particlesInitialPosition[i,0] = normal(xHighest, 50)
                    # particlesInitialPosition[i,1] = normal(yHighest, 50)
                    # particlesInitialPosition[i,2] = normal(thetaHighest, 10)
                    if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                        particlesInitialPosition[i,0] = uniform(0, fieldLength)
                        particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                        particlesInitialPosition[i,2] = uniform(0, 360)
                        particlesGlobalPosition[i,:] = 0
                        particlesLocalPosition[i,:] = 0
                    else:
                        particlesInitialPosition[i,0] = normal(estimatePosition[0], 50)
                        particlesInitialPosition[i,1] = normal(estimatePosition[1], 50)
                        particlesInitialPosition[i,2] = normal(estimatePosition[2], 10)
                        particlesGlobalPosition[i,:] = 0
                        particlesLocalPosition[i,:] = 0

                if showGUI:
                    cv2.waitKey(1)


if __name__ == "__main__":
    print 'Running BarelangFC - MCL Localization'
    url = "http://0.0.0.0:8888"
    if (os.name == "nt"):
        chromedir= 'C:/Program Files (x86)/Google/Chrome/Application/chrome.exe %s'
        webbrowser.get(chromedir).open(url)
    else:
        webbrowser.get(using='firefox').open_new_tab(url)
    app.run(host='0.0.0.0', port=8888, debug=False, threaded=True)
    