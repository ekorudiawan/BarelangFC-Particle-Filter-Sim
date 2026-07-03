# BarelangFC-Particle-Filter-Sim V2.0

Barelang FC Particle Filter (MCL) Localization Simulator

Opensource contribution of Barelang FC Team for RoboCup Competition

Created by : Eko Rudiawan Jamzuri

---

## Changelog

### V2.0 (2025)
- **Python 3** — migrated from Python 2.7 (`print`, `time.clock`, UDP socket encoding)
- **Motion noise model** — Gaussian noise injected on particle position & heading each predict step (prevents particle collapse)
- **Systematic resampling** — low-variance sampler replaces ad-hoc loop; better particle diversity, avoids degeneracy
- **normalize_angle()** helper — replaces 4 duplicated if/elif angle-wrapping blocks
- **Weight degenerate guard** — reset to uniform when all weights collapse to zero

### V1.0 (2017)
- Initial release: MCL localization with 2 landmarks, Flask web stream

---

## Algorithm Overview

```
[Odometry / IMU]
       │
       ▼
  Predict Particles        ← Motion model + Gaussian noise (Upgrade V2)
       │
       ▼
  Measure Landmarks        ← Euclidean distance robot ↔ 2 goal poles
       │
       ▼
  Weight Particles         ← Gaussian PDF per landmark
       │
       ▼
  Normalize Weights
       │
       ▼
  Weighted Average         ← Estimate position (X, Y, θ)
       │
       ▼
  Systematic Resample      ← Low-variance sampler (Upgrade V2)
       │                      10% random explorers + 90% clustered
       └──────────────────────────────────┐
                                          ▼
                                   Next iteration
```

---

## Dependencies

```
Python >= 3.8
OpenCV  >= 4.x
NumPy
SciPy
Flask
```

Install:

```bash
pip install opencv-python numpy scipy flask
```

---

## How To Run

```bash
git clone https://github.com/ekorudiawan/BarelangFC-Particle-Filter-Sim.git
cd BarelangFC-Particle-Filter-Sim/Source-Code/
python3 BarelangFC-Particle-Filter-Sim.py
```

Browser opens automatically at `http://0.0.0.0:8888`

Exit from program: close the browser / Ctrl+C terminal

---

## Configuration

Edit the top of `BarelangFC-Particle-Filter-Sim.py`:

| Variable | Default | Description |
|----------|---------|-------------|
| `simulationMode` | `True` | `False` = read odometry via UDP |
| `headingFromIMU` | `True` | `False` = integrate angular velocity |
| `totalParticles` | `50` | Number of particles |
| `deltaTime` | `1` | Update interval (seconds) |
| `NOISE_POS_STDDEV` | `10.0` | Motion noise — position (cm) |
| `NOISE_HDG_STDDEV` | `3.0` | Motion noise — heading (degrees) |

### Real robot mode (simulationMode = False)

UDP input on port `5005` — comma-separated string:

```
Vx, Vy, Va, Dist_Landmark0, Dist_Landmark1, IMU_Init, IMU_Current, BallDist, PanAngle
```

UDP output on port `5006`:

```
RobotX, RobotY, RobotTheta, BallX, BallY
```

---

## Example Visualization

![Barelang FC - Particle Filter Simulator](/Images/Demo.png)

| Color | Meaning |
|-------|---------|
| 🔴 Red | Particles |
| 🟡 Yellow | Ground truth robot position (simulation only) |
| 🔵 Blue | Estimated robot position |
| 🟢 Cyan-Yellow | Estimated ball position |
| 🟣 Purple | Landmarks (goal poles) |

- **Default sampling** : 1 s
- **Default particles** : 50
- **Landmarks** : 2 — left & right opponent goal pole

---

## Tuning Tips

- **Particle collapse** (all particles converge to wrong spot): increase `NOISE_POS_STDDEV`
- **Slow convergence**: reduce `NOISE_POS_STDDEV`, increase `totalParticles`
- **Landmark ambiguity** (symmetric field): add more landmarks (e.g. 4 corner goal poles)
- **Heading drift**: enable `headingFromIMU = True` and connect real IMU data

---

## References

* [Kalman and Bayesian Filters in Python — Roger R. Labbe](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
* [Probabilistic Robotics — Thrun, Burgard, Fox (systematic resampling: Table 4.4)](http://www.probabilistic-robotics.org/)
* [Adiprawita et al. "New resampling algorithm for particle filter localization for mobile robot with 3 ultrasonic sonar sensor." ICEEI 2011. IEEE.](http://ieeexplore.ieee.org/abstract/document/6021733/)
