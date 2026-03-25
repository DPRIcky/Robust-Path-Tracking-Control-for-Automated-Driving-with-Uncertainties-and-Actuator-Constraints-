# Robust Path Tracking Control for Automated Driving

## Midterm Presentation

**Course:** EGR 560 - Vehicle Dynamics and Control  
**Team:** Prajjwal Dutta, Vamsi Sai Korlapati  
**Date:** March 24, 2026

### Midterm focus

- Build a control-oriented path-tracking model in MATLAB/Simulink.
- Implement a baseline LQR controller with feedforward steering.
- Evaluate actuator limits, model uncertainty, crosswind disturbance, and residual-based novelty estimation.
- Use this baseline as the foundation for the robust controller phase.

---

# 1. Motivation and Problem Statement

- Automated path tracking must keep lateral error `e_y` and heading error `e_psi` small during maneuvers.
- Real vehicles face modeling error, external disturbances, and steering actuator limits.
- A controller that looks strong in nominal simulation can degrade once rate limits or plant mismatch are introduced.
- Our midterm goal is to establish a reliable baseline implementation before moving to a fully robust design.

### Questions addressed at midterm

- Does a discrete LQR baseline track lane-change and constant-turn references accurately?
- How sensitive is the closed-loop response to actuator limits and plant uncertainty?
- Can a simple residual estimator detect mismatch and disturbance online?

---

# 2. Literature Review and Design Positioning

### Litmaps-guided literature story

- Our Litmaps review starts with **classical lateral guidance and trajectory tracking**, then moves through **feedforward-feedback and LQR implementations**, and ends in **robust H-infinity and tube / robust MPC path tracking**.
- **Peng (1991), _Optimal Preview Control For Vehicle Lateral Guidance_** is the earliest control-oriented anchor in our map and motivates the idea that path tracking quality improves when curvature preview is built into steering control.
- **Hoffmann (2007), _Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing_** and **Kammel (2008), _Team AnnieWAY's autonomous system for the 2007 DARPA Urban Challenge_** show that trajectory tracking must work in realistic autonomous driving systems, not only in simplified simulation.
- **Phillips (2011), _Model-Based Feedforward-Feedback Tracking Control for Real-Time Hybrid Simulation_** strengthens the case for combining feedforward path information with feedback stabilization, which is exactly the structure we use in `delta_cmd = (l_f + l_r) kappa_ref - Kx`.
- **Jing (2015), _Robust H-infinity output-feedback control for path following of autonomous ground vehicles_** marks the transition from nominal tracking to robustness against uncertainty and disturbance.
- **Peng (2020), _Path Tracking and Direct Yaw Moment Coordinated Control Based on Robust MPC With the Finite Time Horizon for Autonomous Independent-Drive Vehicles_** and **Park (2021), _Experimental Verification of a Drift Controller for Autonomous Vehicle Tracking: a Circular Trajectory Using LQR Method_** show two complementary directions: advanced robust MPC on one side and experimentally validated LQR-based tracking on the other.
- **Zhou (2021), _Popov-H-infinity Robust Path-Tracking Control of Autonomous Ground Vehicles with Consideration of Sector Bounded Kinematic Nonlinearity_**, **Zheng (2022), _Varying Zonotopic tube RMPC with switching logic for lateral path tracking of autonomous vehicle_**, **Dai (2023), _A Tube Model Predictive Control Method for Autonomous Lateral Vehicle Control Based on Sliding Mode Control_**, and **Yao (2025), _Path Tracking Robust Control Strategy for Intelligent Vehicle Based on Force-Driven with MPC and H-infinity_** define the current frontier in our review: path tracking with explicit uncertainty sets, robust guarantees, and constraint handling.

### What we took from this review

- Start with a control-oriented dynamic bicycle model and a transparent baseline controller.
- Include feedforward curvature action, because the literature repeatedly shows that pure feedback is not enough for strong path-following performance.
- Add actuator constraints, uncertainty tests, and disturbance monitoring now, because that is the bridge from classical tracking papers to modern robust MPC and H-infinity designs.

---

# 3. Implementation Architecture

### Control-oriented model

- State vector: `x = [e_y, e_psi, v_y, r]^T`
- Inputs: steering `delta` and reference curvature `kappa_ref`
- Continuous bicycle model is discretized with zero-order hold at `Ts = 0.02 s`

### Baseline controller

- Steering law: `delta_cmd = (l_f + l_r) kappa_ref - Kx`
- Feedforward term handles nominal path curvature.
- LQR state feedback rejects lateral and heading error.

### Midterm implementation blocks

- MATLAB script for model generation, simulation, metrics, and plots
- Simulink model that mirrors the same closed-loop architecture
- Actuator block for steering angle/rate limits
- Residual block for `w_hat`, `w_inf`, and rolling `w_max`

**Backup architecture figure:** [Simulink model overview](../plots/simulink_model.pdf)

---

# 4. Experimental Setup

### Nominal vehicle and controller settings

- `m = 1573 kg`, `Iz = 2031.4 kg m^2`
- `l_f = 1.04 m`, `l_r = 1.56 m`
- `C_f = 52618 N/rad`, `C_r = 110185 N/rad`
- `V_x = 10 m/s`, `Ts = 0.02 s`
- `Q = diag([1000, 300, 5, 5])`, `R = 1`

### Scenarios

- **Scenario A:** smooth lane-change curvature burst
- **Scenario B:** constant-radius turn

### Stress tests

- Steering limits: `delta_max = 0.5 rad`, `delta_rate_max = 0.5 rad/s`
- Parameter uncertainty: `C_f` and `C_r` reduced to `0.7 x` nominal
- Crosswind disturbance: `500 N` pulse from `t = 2 s` to `t = 3 s`
- Novelty metric: compare measured next state with nominal one-step prediction

---

# 5. Open-Loop Response Motivates Feedback Control

- With `delta = 0`, the plant follows road curvature only through the disturbance-like curvature channel.
- Both scenarios generate nonzero lateral and heading error.
- This confirms that feedback control is required even for moderate maneuvers.

| Scenario A: Lane change | Scenario B: Constant turn |
| --- | --- |
| ![Open-loop lane change](../plots/scenario_A_lane_change.png) | ![Open-loop constant turn](../plots/scenario_B_constant_turn.png) |

---

# 6. Baseline LQR Tracking Results

- Computed gain: `K = [12.6139, 13.2589, 0.8954, 0.5511]`
- Closed-loop poles move inside the unit circle and the tracking response becomes well damped.
- The feedforward plus LQR structure tracks both references with millimeter-level lateral error in simulation.

### Key nominal metrics

- **Lane change:** `RMS(e_y) = 0.0017 m`, `max |e_y| = 0.0039 m`, `max |delta| = 0.0766 rad`
- **Constant turn:** `RMS(e_y) = 0.0014 m`, `max |e_y| = 0.0020 m`, `max |delta| = 0.0394 rad`

| Scenario A: LQR lane change | Scenario B: LQR constant turn |
| --- | --- |
| ![LQR lane change](../plots/scenario_A_lane_change_LQR.png) | ![LQR constant turn](../plots/scenario_B_constant_turn_LQR.png) |

---

# 7. Actuator-Constrained Performance

- We added realistic steering magnitude and rate limits to test implementability.
- The lane-change case remains essentially unchanged because commanded steering stays within limits.
- The constant-turn case activates the steering rate limit, but tracking degradation remains very small.

### Constraint observations

- **Lane change:** `0.0%` rate-limited, `0.0%` angle-saturated
- **Constant turn:** `2.8%` rate-limited, `0.0%` angle-saturated
- This indicates the baseline controller is feasible under the current steering limits.

| Scenario A: constrained vs unconstrained | Scenario B: constrained vs unconstrained |
| --- | --- |
| ![Lane change constraints](../plots/scenario_A_lane_change_LQR_constraints.png) | ![Constant turn constraints](../plots/scenario_B_constant_turn_LQR_constraints.png) |

---

# 8. Robustness Study: Parameter Uncertainty and Crosswind

- To emulate plant mismatch, front and rear cornering stiffness were both reduced by `30%`.
- To emulate disturbance, a `500 N` lateral wind pulse was injected for `1 s`.
- The controller remains stable in all tested cases, but uncertainty increases tracking error and steering demand.

### Representative quantitative trends

- **Lane change uncertainty:** `RMS(e_y)` grows from `0.0017 m` to `0.0028 m`
- **Constant turn uncertainty:** `RMS(e_y)` grows from `0.0014 m` to `0.0023 m`
- **Constant turn uncertainty:** rate-limited fraction rises from `2.8%` to `4.4%`
- **Wind pulse:** produces smaller tracking degradation than parameter uncertainty in the tested setup

| Constant turn with uncertainty | Constant turn with wind |
| --- | --- |
| ![Constant turn uncertainty](../plots/scenario_B_constant_turn_uncertainty.png) | ![Constant turn wind](../plots/scenario_B_constant_turn_wind.png) |

---

# 9. Novelty Estimation Separates Nominal, Uncertainty, and Wind Cases

- We compute `w_hat(k) = x(k+1) - (A_d x_meas(k) + B_d delta_applied(k) + E_d kappa_ref(k))`
- `w_inf` is the instantaneous infinity norm of the residual.
- `w_max` is a rolling maximum over a 1-second window.
- This gives an online indicator of model mismatch or disturbance strength.

### Scenario B results

- **Nominal:** `max(w_inf) = 1.70e-4`
- **Uncertainty:** `max(w_inf) = 1.29e-2`
- **Wind:** `max(w_inf) = 6.45e-3`
- The residual monitor clearly separates nominal behavior from non-nominal behavior.

| Instantaneous residual `w_inf` | Rolling bound `w_max` |
| --- | --- |
| ![Residual overlay](../plots/scenario_B_what_overlay.png) | ![Rolling bound overlay](../plots/scenario_B_wmax_overlay.png) |

---

# 10. Conclusions and Next Steps

### Midterm conclusions

- The MATLAB and Simulink baseline implementation is complete and reproducible.
- The LQR controller achieves accurate path tracking on both benchmark maneuvers.
- Steering constraints are manageable in the current test envelope.
- Parameter uncertainty is a stronger challenge than the tested wind pulse.
- Residual-based novelty estimation provides a useful signal for detecting non-nominal operation.

### Next phase

- Implement a robust controller that explicitly accounts for uncertainty and constraints.
- Expand the test matrix to higher-speed, lower-friction, and more aggressive maneuvers.
- Use the current residual signals to inform disturbance bounds or adaptive robust logic.

### Core references

- Peng, 1991, _Optimal Preview Control For Vehicle Lateral Guidance_.  
- Hoffmann, 2007, _Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing_.  
- Kammel, 2008, _Team AnnieWAY's autonomous system for the 2007 DARPA Urban Challenge_.  
- Phillips, 2011, _Model-Based Feedforward-Feedback Tracking Control for Real-Time Hybrid Simulation_.  
- Jing, 2015, _Robust H-infinity output-feedback control for path following of autonomous ground vehicles_.  
- Peng, 2020, _Path Tracking and Direct Yaw Moment Coordinated Control Based on Robust MPC With the Finite Time Horizon for Autonomous Independent-Drive Vehicles_.  
- Park, 2021, _Experimental Verification of a Drift Controller for Autonomous Vehicle Tracking: a Circular Trajectory Using LQR Method_.  
- Zhou, 2021, _Popov-H-infinity Robust Path-Tracking Control of Autonomous Ground Vehicles with Consideration of Sector Bounded Kinematic Nonlinearity_.  
- Zheng, 2022, _Varying Zonotopic tube RMPC with switching logic for lateral path tracking of autonomous vehicle_.  
- Dai, 2023, _A Tube Model Predictive Control Method for Autonomous Lateral Vehicle Control Based on Sliding Mode Control_.  
- Yao, 2025, _Path Tracking Robust Control Strategy for Intelligent Vehicle Based on Force-Driven with MPC and H-infinity_.
