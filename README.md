# AI Analysis of Regenerative Braking in Urban Electric Vehicles
### An Energy Recovery Gap Assessment Approach

MSc Dissertation project for CST7000 — Robotics and Artificial Intelligence
Cardiff School of Technologies · Cardiff Metropolitan University · August, 2026

Author: **Md Amjad Hossain Khan** (st20341331)
Supervisor: Dr Paul Jenkins

---

## Overview

Regenerative braking is one of the main efficiency advantages electric vehicles hold over conventional cars, yet a large share of the kinetic energy available during braking is never returned to the battery. This project quantifies that gap between **recoverable** and **actually recovered** braking energy in a representative urban battery electric vehicle, across the two most widely adopted standardised driving cycles: the **EPA UDDS** and the **WLTP Class 3b**.

A physics-based energy model was built in Python, independently cross-validated in MATLAB, and paired with a machine learning analysis that identifies which factors actually drive the recovery gap.

---

## Key Findings

- **The gap is large and consistent.** Across 99 braking events, roughly **37% of recoverable braking energy is never recovered** — 37.55% under UDDS and 36.32% under WLTP.
- **The gap is cycle-agnostic.** Two very different driving cycles produced a recovery gap within ~1 percentage point of each other, indicating the gap is a systemic drivetrain property rather than a cycle-specific effect.
- **Stop-and-go beats smooth driving.** Stop-and-go segments recovered significantly more energy than smooth segments (63.93% vs 41.74%, p = 0.0324), explained by motor operating-point behaviour.
- **Constraints dominate.** A Gradient Boosting model (test R² = 0.972) identified the combined **constraint factor** as the dominant driver of the gap, while the driving cycle itself had essentially zero predictive importance.

**Takeaway:** constraint management, not cycle-specific tuning, offers the greatest opportunity for improving urban regenerative braking efficiency.

---

## Repository Contents

| File | Description |
|------|-------------|
| `Regen_Braking_Analysis_st20341331.ipynb` | Main analysis notebook — full pipeline from data loading to ML |
| `udds_validation.m` | MATLAB script that independently cross-validates the Python energy model |
| `Dissertation_Results_st20341331.xlsx` | Processed results: per-event data, segment analysis, feature matrix, loss breakdown |
| `/figures` | All generated figures used in the dissertation |

---

## Methodology in Brief

1. **Data** — Official EPA UDDS speed trace and the official EU JRC `wltp` package (WLTP Class 3b, UNECE Reg. 154).
2. **Energy model** — Per-event recoverable energy from kinetic energy change, with recovered energy applying deceleration-dependent motor efficiency, inverter and battery losses, SOC limits, and friction-braking constraints.
3. **Analysis** — Statistical cycle comparison (Mann-Whitney U), stop-and-go vs smooth segmentation, and a five-model machine learning comparison with permutation importance.
4. **Validation** — Entire energy model rebuilt independently in MATLAB; core energy metrics agreed within 5%.

---

## How to Run

The notebook is designed to run top-to-bottom in **Google Colab**.

1. Open `Regen_Braking_Analysis_st20341331.ipynb` in Colab.
2. In Section 2, upload the EPA `udds.txt` file when prompted.
3. Section 3 installs the official `wltp` package automatically if it is not present.
4. Run all remaining cells in order.

**Data sources**
- UDDS: [EPA Dynamometer Drive Schedules](https://www.epa.gov/vehicle-and-fuel-emissions-testing/dynamometer-drive-schedules)
- WLTP Class 3b: [`wltp` Python package](https://pypi.org/project/wltp/) (EU Joint Research Centre)

---

## Tools and Libraries

Python 3 · NumPy · Pandas · SciPy · scikit-learn · Matplotlib · Seaborn · `wltp` · MATLAB R2026a

---

## Citation

If you refer to this work, please cite:

> Khan, M.A.H. (2026) *AI Analysis of Regenerative Braking in Urban Electric Vehicles: An Energy Recovery Gap Assessment Approach.* MSc Dissertation, Cardiff Metropolitan University.

---

## Contact

**Md Amjad Hossain Khan**
Amjadhkhan.aust@gmail.com
[LinkedIn](https://linkedin.com/in/amjad-hossain-khan-522685179)

---

*This repository accompanies an MSc dissertation submitted to Cardiff School of Technologies, Cardiff Metropolitan University.*
