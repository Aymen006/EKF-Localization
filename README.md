

# 📌  EKF Robot Localisation — Project README  
**Extended Kalman Filter (EKF) for Mobile Robot Localisation**  
Author: **Eymene Ouasime Khiat**.  

---

## Overview
This repository contains a complete implementation and experimental study of an **Extended Kalman Filter (EKF)** for estimating a mobile robot’s pose and velocity by fusing measurements from a speed sensor, a gyro, and a GNSS receiver. The project includes:

- formal motion & observation models,
- analytic Jacobians used for linearisation,
- EKF prediction & update loops,
- parameter tuning experiments on process and observation covariance (Q and R),
- visualisations of trajectories and covariance ellipses,
- analysis of filter behaviour and limitations.

---

## 🎯  Goals & Contributions
- Implement a nonlinear motion model for a differential/mobile robot and derive the Jacobian (F) required for EKF covariance propagation. 
- Implement a GNSS-based observation model that measures the robot’s x-y position and derive its Jacobian (H).
- Implement the EKF predict–update algorithm, including Kalman gain computation and covariance update.  
- Run systematic experiments to tune **Q** (process noise) and **R** (observation noise) and evaluate their effect on estimation accuracy and uncertainty. Key tuned values and observations are reported. 

---

## 🧠  Mathematical Models & Implementation Details

### State and Control
The implemented state vector is:
x_t = [x, y, θ, v]^T
- `x, y` — position in 2D  
- `θ` — orientation (heading)  
- `v` — forward velocity  
Control input:
u_t = [v_t, ω_t]^T
- `v_t` — commanded/measured linear velocity (from speed sensor)  
- `ω_t` — angular velocity (from gyro). 

### Motion Model (f)
The motion function `f(x_t, u_t)` implements the nonlinear kinematic update (discrete-time) used to predict the robot next state from current state and control inputs. Typical form implemented:
- `v_t` — commanded/measured linear velocity (from speed sensor)  
- `ω_t` — angular velocity (from gyro).

### Motion Model (f)
The motion function `f(x_t, u_t)` implements the nonlinear kinematic update (discrete-time) used to predict the robot next state from current state and control inputs. Typical form implemented:
x_{t+1} = x_t + v * cos(θ) * Δt
y_{t+1} = y_t + v * sin(θ) * Δt
θ_{t+1} = θ_t + ω * Δt
Process noise `w_x` is assumed zero-mean Gaussian with covariance `Q`. In code `Q` is a diagonal matrix tuned experimentally.

### Motion Jacobian (F)
The Jacobian `F = ∂f/∂x` is computed analytically and used to propagate the state covariance in the prediction step:
P̄ = F P F^T + Q
Details of the partial derivatives (with respect to x,y,θ,v) are implemented in `motion_model.py` (or equivalent) and used at every predict step. 

### Observation Model (h)
The GNSS observation provides position only:
z_t = [x, y]^T = h(x_t) + w_z

Observation Jacobian `H = ∂h/∂x` is therefore a matrix that extracts the x and y coordinates from the state. Observation noise `w_z` ~ N(0, R) where `R` is the GNSS covariance. 

### EKF Steps (implemented)
**Prediction**
- `x̄ = f(x, u)`  
- `P̄ = F P F^T + Q`  

**Update**
- `z̄ = h(x̄)`  
- `S = H P̄ H^T + R`  
- `K = P̄ H^T S^{-1}`  
- `x = x̄ + K (z - z̄)`  
- `P = (I - K H) P̄`  
All matrix operations are implemented with numerical stability checks (e.g., symmetric enforcement of P, small-epsilon for matrix inversions).

---

## 🧮  Experiments & Tuning (results from this project)
The project includes a set of experiments that systematically tuned `Q` and `R` and visualised effects on the state estimates and covariance ellipses.

- **Initial tuning ranges**: Q started near `[1.5, 1.5]` (per-dimension motion variance) and R near `[1, 1]`.
- **Best-observed configuration in experiments**:  
  - `Q = diag([2.23, 2.23, q_theta, q_v])` — increased motion uncertainty to reflect model mismatch.  
  - `R = diag([0.6, 0.6])` — tightened GNSS observation covariance to reflect higher GNSS confidence.  
  These changes produced **smaller covariance ellipses** and more stable pose/velocity estimates vs. baseline. 

**Observations**
- Lowering `R` (more trust in GNSS) reduces long-term pose uncertainty but increases sensitivity to GNSS outliers.  
- Increasing `Q` reduces overconfidence in the motion model and prevents filter divergence when motion model is inaccurate.  
- Achieving balance required >30 iterations of tuning in our experiments — EKF sensitivity to Q/R was evident. 
---

## 📊 Visualisations produced
- True trajectory vs estimated trajectory plot (with GNSS scatter).  
- Covariance ellipses plotted at regular time intervals (showing P).  
- Time-series plots for estimated velocity and orientation vs ground truth.  
- Tuning comparison plots showing how different Q/R settings affect error and covariance.

---

![Image Alt](https://github.com/Aymen006/EKF-Localization/blob/master/Screenshot%202025-12-06%20at%2021.39.20.png?raw=true)
![Image Alt](https://github.com/Aymen006/EKF-Localization/blob/master/Screenshot%202025-12-06%20at%2021.39.10.png?raw=true)











