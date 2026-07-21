# Regenerative Braking Analysis — MSc Dissertation

This is the code and data for my dissertation at Cardiff Metropolitan University, studying the gap between recoverable and actually recovered braking energy in urban electric vehicles.

## What is in this repo

| Folder | What it contains |
|---|---|
| `data/` | UDDS driving cycle file from the EPA, plus WLTP data loaded via the official JRC package |
| `notebooks/` | The main Python analysis notebook with all energy calculations, ML models, and figures |
| `matlab/` | MATLAB validation script that replicates the Python energy model independently |
| `outputs/` | All generated figures and the Excel results file |

## Main results

- 37.55% energy recovery gap under UDDS, 36.32% under WLTP
- The gap is nearly identical across two very different cycles, which means it is a systemic drivetrain property
- Stop-and-go driving recovers significantly more energy than smooth driving (63.93% vs 41.74%, p = 0.0324)
- Gradient Boosting model achieved R² = 0.972 predicting event-level recovery rates
- The constraint factor (SOC limits, friction threshold, speed effects) is the dominant driver of the gap

## How to run

**Python notebook:**

```bash
pip install -r requirements.txt
jupyter notebook
