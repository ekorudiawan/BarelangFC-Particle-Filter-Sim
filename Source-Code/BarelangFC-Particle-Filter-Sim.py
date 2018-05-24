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

# Main configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

fieldLength = 900
fieldWidth = 600
totalParticles = 100
totalLandmarks = 2

# Global Variable
# Robot position 1D array
robotPosition = np.zeros((3), dtype=int)

# Estimate position
estimatePosition = np.zeros((3), dtype=int)

# Landmarks position 2D array
landmarksPosition = np.zeros((totalLandmarks, 2), dtype=int)
particlesPosition = np.zeros((totalParticles, 3), dtype=int)
distanceRobotToLandmarks = np.zeros((totalLandmarks), dtype=int)
distanceParticlesToLandmarks = np.zeros((totalParticles, totalLandmarks), dtype=int)
particlesWeight= np.zeros((totalParticles), dtype=float)

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

def main():
    # Initialize robot position
    robotPosition[0] = uniform(0, fieldLength)
    robotPosition[1] = uniform(0, fieldWidth)
    # Set initial location of robot
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

    mapImage = np.zeros((800,1100,3), np.uint8)
    
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

    # Create UDP client to receive data from kinematic
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        sock.bind((UDP_IP, UDP_PORT))
    except socket.error:
        print 'Failed to create socket'
        sys.exit()

    loop = 0

    while True:
        # print 'Loop : %d'%loop
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
        # cv2.imwrite("mapImage.jpg", mapImage)
        # break

        # Get data from kinematic
        data, _ = sock.recvfrom(1024) # buffer size is 1024 bytes
        # print "Vel Input : ", data
        velFromKinematic = np.zeros((3))
        strVelFromKinematic = data.split(",")
        velFromKinematic[0] = float(strVelFromKinematic[0])
        velFromKinematic[1] = float(strVelFromKinematic[1])
        velFromKinematic[2] = float(strVelFromKinematic[2])

        # print "Input Vel : ", velFromKinematic

        # X = 5, Y = 0, alpha = 0
        # simVelocity[0] = 10 
        # simVelocity[1] = 0 
        # simVelocity[2] = 0 

        simVelocity = convertVel(robotID, velFromKinematic)
        # print "Output Vel : ", simVelocity

        # Simulate robot movement
        vectVelocity  = math.sqrt(pow(simVelocity[0], 2) + pow(simVelocity[1], 2))

        if simVelocity[2] != 0:
            robotPosition[0] += (vectVelocity / simVelocity[2]) * ( math.sin(math.radians(robotPosition[2])) + (simVelocity[2] * deltaTime) ) - math.sin(math.radians(robotPosition[2]))
            robotPosition[1] += (vectVelocity / simVelocity[2]) * ( math.cos(math.radians(robotPosition[2])) - math.cos(math.radians(robotPosition[2])) + (simVelocity[2] * deltaTime))
            robotPosition[2] += simVelocity[2] * deltaTime
            # // Add measurements to particles
            # particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
            # particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
            # particles[i].theta += yaw_rate * delta_t;
        else:
            robotPosition[0] += simVelocity[0] * deltaTime 
            robotPosition[1] += simVelocity[1] * deltaTime 

        # } else {
        # // Add measurements to particles
        # particles[i].x += velocity * delta_t * cos(particles[i].theta);
        # particles[i].y += velocity * delta_t * sin(particles[i].theta);
        # // Theta will stay the same due to no yaw_rate
        
        # }
        # print "Vel : ", vectVelocity
        # robotPosition[0] += simVelocity[0]
        # robotPosition[1] += simVelocity[1]
        print 'Robot position : (%d, %d, %d)'%(robotPosition[0], robotPosition[1], robotPosition[2])

        # Predict movement of particles
        # print 'Process ==> Predict movement of particles'
        particlesPosition[:,0] += int(simVelocity[0])
        particlesPosition[:,1] += int(simVelocity[1])
        # Simulate noise movement of robot with error stddev = 10
        particlesPosition[:,0] = normal(particlesPosition[:,0], 10)
        particlesPosition[:,1] = normal(particlesPosition[:,1], 10)

        # print 'Particles position'
        # print particlesPosition
        # Update measurement
        # Measurement distance between robot and landmarks
        for i in range (0,totalLandmarks):
            distanceRobotToLandmarks[i] = math.hypot(robotPosition[0] - landmarksPosition[i,0], robotPosition[1] - landmarksPosition[i,1])
            # Simulate noise with random gaussian from measurment
            distanceRobotToLandmarks[i] = normal(distanceRobotToLandmarks[i], 10)
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
        # print 'Estimate Position : (%d,%d)'%(estimatePosition[0],estimatePosition[1])
        
        drawParticles = False
        if drawParticles == True:
            for i in range (0, totalParticles):
                cv2.circle(mapImage,(particlesPosition[i,0]+100, 800 - (particlesPosition[i,1]+100) ), 7, (0,0,255), -1)

        
        cv2.circle(mapImage,(robotPosition[0]+100, 800 - (robotPosition[1]+100) ), 7, (0,255,255), -1)
        # cv2.circle(mapImage,(int(estimatePosition[0])+100, 800 - (int(estimatePosition[1])+100)), 7, (255,0,0), -1)

        textLine = "R{} Position (X, Y, Theta) : ({}, {}, {})".format(robotID, int(estimatePosition[0]), int(estimatePosition[1]), int(estimatePosition[1]))
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
            particlesPosition[i,0] = normal(xHighest, 20)
            particlesPosition[i,1] = normal(yHighest, 20) 

        loop += 1     
        k = cv2.waitKey(1000)
        if k == ord('x'):         # X
            break

if __name__ == "__main__":
	print 'Running BarelangFC - MCL Localization'
	app.run(host='127.0.0.1', port=8888, debug=False, threaded=True)
