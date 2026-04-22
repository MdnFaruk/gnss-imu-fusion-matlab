# GNSS/IMU Sensor Fusion for Outdoor Mobile Robot Positioning

**Course:** Intelligent Robots — University of Vaasa  
**Task:** Accurate positioning for a highway driving robot  
**Author:** Md Faruk  
**Language:** MATLAB (R2019b or later)

---

## What this project does

GPS alone gives about 35 metres of error on highways because signals bounce off bridges and barriers, this is called multipath. GPS also stops working completely inside tunnels.

This project combines GPS with an IMU (accelerometer) using a **Kalman Filter** and a **Rauch-Tung-Striebel (RTS) Smoother** to get better position estimates. The IMU keeps tracking when GPS is lost. The RTS smoother goes backwards through all the data after the Kalman Filter finishes, which makes the path cleaner.

---

## Results

| Metric | GPS Only | Kalman Filter | RTS Smoother |
|---|---|---|---|
| Mean Error | 35.09 m | 35.32 m | 35.09 m |
| RMS Error | 36.67 m | 36.85 m | 36.66 m |
| Path Jitter | 1.038 m | 1.052 m | **0.979 m** |

**GPS outage test (30 seconds, simulated tunnel at 130 km/h):**

| Method | Error |
|---|---|
| Frozen GPS (last known fix) | 680 m |
| Kalman Filter + IMU | **54 m (92% improvement)** |

The path smoothness improved by **5.7%**. The 35 m GPS bias stays because it is a systematic offset that filtering alone cannot fix — that needs PPK/RTK differential corrections.

---

## Dataset

File: `df_total.csv`  
1200 time steps · 1 second per step · ~43 km highway drive · ~130 km/h

| Column | Description |
|---|---|
| `true_x`, `true_y` | Ground truth position from survey-grade GPS |
| `gnss_x`, `gnss_y` | Raw smartphone GPS (noisy) |
| `ax`, `ay` | IMU body-frame accelerations (ay includes gravity ~9.81 m/s²) |

---

## File structure

```
gnss_fusion_matlab/
├── main.m              ← run this file to start everything
├── df_total.csv        ← input dataset (place here)
├── load_data.m         ← reads CSV, removes gravity from IMU ay
├── kf_matrices.m       ← builds F, B, Q, H matrices
├── kalman_filter.m     ← forward Kalman Filter
├── rts_smoother.m      ← backward RTS Smoother
├── evaluate.m          ← computes errors, prints results
├── visualize.m         ← generates all figures
└── navigation_demo.m   ← optional: obstacle avoidance demo
```

---

## How to run

1. Clone or download this repository
2. Place `df_total.csv` in the same folder as the `.m` files
3. Open MATLAB and navigate to that folder
4. Type `main` in the Command Window and press Enter

All figures will open automatically. Nothing is saved to disk.

---

## Algorithm overview

```
Load df_total.csv
       |
       v
IMU Preprocessing
  ay_corr = ay - 9.81   (remove gravity)
       |
       v
Forward Kalman Filter
  State: x = [px, vx, py, vy]
  Predict:  x = F*x + B*u   (u = IMU acceleration)
  Update:   x = x + K*(z - H*x)   (z = GPS position)
  If GPS lost → skip update, use IMU only (dead-reckoning)
       |
       v
Backward RTS Smoother
  Goes from step N back to step 1
  Improves every estimate using future data
       |
       v
Evaluate + Visualize
```

**Key equations:**

State transition:
```
x_{k+1} = F * x_k  +  B * u_k
F = [1 dt 0  0 ]    B = [0.5*dt^2    0       ]
    [0  1 0  0 ]        [dt          0       ]
    [0  0 1 dt ]        [0           0.5*dt^2]
    [0  0 0  1 ]        [0           dt      ]
```

RTS smoother backward pass:
```
C_k     = P_{k|k} * F' * inv(P_{k+1|k})
x_{k|N} = x_{k|k} + C_k * (x_{k+1|N} - x_{k+1|k})
```

---

## Figures produced

| Figure | Description |
|---|---|
| Figure 1 | Full trajectory + zoomed view (GPS vs KF vs RTS vs Ground Truth) |
| Figure 2 | Horizontal error over time for all three methods |
| Figure 3 | Accuracy and smoothness summary bar charts |
| Figure 4 | GPS outage bridging demo (tunnel simulation) |
| Figure 5 | Noise parameter sensitivity (sigma_gnss and sigma_acc) |
| Figure 6 | Navigation demo with obstacle avoidance (optional) |

---

## Parameters

Edit these values at the top of `main.m`:

| Parameter | Default | Meaning |
|---|---|---|
| `sigma_gnss` | 10.0 m | How noisy the GPS is |
| `sigma_acc` | 5.0 m/s² | How much to trust IMU prediction |
| `dt` | 1.0 s | Time between samples |
| `outage_start` | 400 | Step where GPS outage begins |
| `outage_len` | 30 | Length of GPS outage in steps |

---

## Requirements

- MATLAB R2019b or later
- No extra toolboxes needed
- All functions used (`readtable`, `rms`, `prctile`, `diff`) are built into MATLAB

---

## References

1. Siemuri, A., Elsanhoury, M., Välisuo, P., Kuusniemi, H., & Elmusrati, M. S. (2022). Application of machine learning to GNSS/IMU integration for high precision positioning on smartphones. *ION GNSS+ 2022*, pp. 2256–2264.
2. Rauch, H. E., Tung, F., & Striebel, C. T. (1965). Maximum likelihood estimates of linear dynamic systems. *AIAA Journal*, 3(8), 1445–1450.
3. Kalman, R. E. (1960). A new approach to linear filtering and prediction problems. *Journal of Basic Engineering*, 82(1), 35–45.
