# BarelangFC-Particle-Filter-Sim V1.0

Barelang FC Particle Filter Localization Simulator

Opensource contribution of Barelang FC Team for RoboCup Competition

Created by : Eko Rudiawan Jamzuri

## Dependencies

* Python 2.7
* OpenCV 3.2.0
* Numpy
* Scipy

## How To Run

* Running program

    ```bash
    ek91@BarelangFC:~$ git clone https://github.com/ekorudiawan/BarelangFC-Particle-Filter-Sim.git
    ek91@BarelangFC:~$ cd BarelangFC-Particle-Filter-Sim/Source-Code/
    ek91@BarelangFC:~$ python Barelang-FC-Particle-Filter-Sim.py
    ```

* Exit from program

    Press X

## Example Visualization

![Barelang FC - Particle Filter Simulator](/Images/Demo.png)

* **Red** : Random Particles
* **Yellow** : Robot Position
* **Blue** : Estimate Position
* **Default time sampling** : 1s
* **Default random particles** : 100
* **Default landmark** : 2 (Left opposite goal & right opposite goal)
* This program still estimate robot position (X and Y position), heading position not yet calculated

## References

* [Kalman and Bayesian Filters in Python by Roger R. Labbe](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
* [Adiprawita, Widyawardana, et al. "New resampling algorithm for particle filter localization for mobile robot with 3 ultrasonic sonar sensor." Electrical Engineering and Informatics (ICEEI), 2011 International Conference on. IEEE, 2011.](http://ieeexplore.ieee.org/abstract/document/6021733/)