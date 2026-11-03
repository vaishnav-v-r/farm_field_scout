#!/usr/bin/env python3
"""
Regenerates plant_map.yaml with plant IDs assigned strictly by visit order
(the order the robot actually encounters them on its zigzag route), so that
terminal output always reads Plant #1 through #16 in sequence.

Visit order:
  Pass 1 - Aisle 1 northbound, left side (col 0): IDs 1-4  (south to north)
  Pass 2 - Aisle 2 southbound, left side (col 1): IDs 5-8  (north to south)
  Pass 3 - Aisle 2 northbound, right side (col 2): IDs 9-12 (south to north)
  Pass 4 - Aisle 3 southbound, right side (col 3): IDs 13-16 (north to south)
"""
import random
import yaml

random.seed(42)

COL_XS  = [0.6, 2.2, 3.8, 5.4]
ROW_YS  = [0.6, 2.2, 3.8, 5.4]
AISLES  = [1.4, 3.0, 4.6]
Y_START = -0.3
Y_END   = 6.3
JITTER  = 0.08

categories = (
    ["healthy_unripe"] * 5 +
    ["healthy_ripe"]   * 6 +
    ["unhealthy"]      * 5
)
random.shuffle(categories)

color_map = {
    "healthy_unripe": "Green",
    "healthy_ripe":   "Red",
    "unhealthy":      "Yellow",
}

# Visit order: list of (col_index, row_index) tuples in the exact order
# the robot encounters them on the zigzag route
visit_order = (
    [(0, r) for r in range(4)] +   # Pass 1: col 0, south→north (rows 0,1,2,3)
    [(1, r) for r in range(3,-1,-1)] +  # Pass 2: col 1, north→south (rows 3,2,1,0)
    [(2, r) for r in range(4)] +   # Pass 3: col 2, south→north (rows 0,1,2,3)
    [(3, r) for r in range(3,-1,-1)]    # Pass 4: col 3, north→south (rows 3,2,1,0)
)

plants = []
for visit_id, (col, row) in enumerate(visit_order, start=1):
    category = categories[visit_id - 1]
    plants.append({
        "id":       visit_id,
        "x":        round(COL_XS[col] + random.uniform(-JITTER, JITTER), 3),
        "y":        round(ROW_YS[row] + random.uniform(-JITTER, JITTER), 3),
        "col":      col,
        "row":      row,
        "category": category,
        "color":    color_map[category],
    })

output = {
    "plants": plants,
    "navigation": {
        "aisles":  AISLES,
        "y_start": Y_START,
        "y_end":   Y_END,
    },
}

with open("config/plant_map.yaml", "w") as f:
    yaml.dump(output, f, default_flow_style=False, sort_keys=False)

print(f"Generated {len(plants)} plants (visit-order IDs) into config/plant_map.yaml")
for p in plants:
    print(f"  #{p['id']:2d}  col={p['col']} row={p['row']}  "
          f"x={p['x']:.3f} y={p['y']:.3f}  {p['category']}")
