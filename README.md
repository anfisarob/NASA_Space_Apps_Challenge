#NASA Space Apps Challenge – Safe Path Planner on Mars

An astronaut picks a position on a Mars globe; the app draws a safe route to
Curiosity (default, white dot) using A* over MOLA topography, avoiding steep slopes.
The **Alert** button re-routes to the nearest safe station (blue dots).
Layer buttons: **Topography**, **Thermal inertia**, **Roughness**, **Weather** (card), and **Back** (default texture).

## Data sources (NASA missions)
| Layer / use | Mission & instrument | Source |
|---|---|---|
| Topography, slopes, path planning | Mars Global Surveyor, MOLA (MEGDR, 0.0625°/px) | https://pds-geosciences.wustl.edu/mgs/urn-nasa-pds-mgs_mola_topography_derived/meg016/ |
| Thermal inertia layer | Mars Global Surveyor, TES (Christensen et al., 2001) | https://www.mars.asu.edu/data/tes_ti/ (ASU Mars Space Flight Facility) |
| Roughness layer | Mars Global Surveyor, MOLA vertical roughness (Kreslavsky & Head; see Kreslavsky & Head, *Geophys. Res. Lett.*, 29, 2002) | http://www.mars.asu.edu/data , http://www.planetary.brown.edu/planetary/rough |
| Weather card (Gale Crater) | Mars Science Laboratory (Curiosity), REMS. Static snapshot of Sols 4989-4995 (Aug 19-25, 2026), not live. Credit: NASA/JPL-Caltech/CAB (CSIC-INTA) | https://mars.nasa.gov/layout/embed/image/mslweather/ |

The roughness layer is display-only (an RGB composite of roughness at 0.6, 2.4
and 9.2 km baselines); it is not used in the path cost.

Thermal inertia is also used in the path cost: the layer is a colored picture, so its
color (hue) is converted to an approximate value and low inertia (fine dust, where a rover
could get stuck) is penalized. These are **approximate values, not measured ones**.

## How it works
- **Path planning:** A* (`pyastar2d`) over a cost grid built from MOLA elevation. Cells with a slope above 30° are obstacles; the rest cost more with height changes and with low thermal inertia.
- **Topography layer:** `src/mars.png` is generated from the MOLA data by `topography_generation.py`.
- **Safe stations (blue dots):** Arcadia and Ellada, candidate base locations chosen by the team. **Alert** runs A* to each station and follows the cheapest path.

## Setup
The elevation file is too large for GitHub (>25 MB), so it is **not** in the repo.
Download `megt90n000eb.img` from the PDS link above and put it next to
`megt90n000eb.lbl`.

```
pip install -r requirements.txt
python/python3 astar_algorithm.py
```
Then open http://127.0.0.1:5000
