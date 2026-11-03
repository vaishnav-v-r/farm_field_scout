# 🚜 Autonomous Farm Field Scout Robot (TurtleBot3 + ROS 1 Noetic + Gazebo)

An autonomous precision agriculture scouting system built with **ROS 1 Noetic** and **Gazebo** simulation. A **TurtleBot3 Waffle** robot navigates a 4×4 crop field (16 plants) in an aisle-by-aisle zigzag pattern, senses plants using synchronized diagonal LIDAR detection bands, captures RGB camera frames, extracts normalized HSV color features, and runs a trained scikit-learn classifier to monitor plant health and ripeness in real-time.

---

## 🌟 Key Features

- **Autonomous Zigzag Aisle Navigation:** 4-pass route covering all 16 plants in strict numerical visit order (**#1 to #16**).
- **Closed-Loop P Heading-Hold Controller:** Maintains dead-reckoning heading along straight aisle sweeps, eliminating wheel slip and drift.
- **Synchronized LIDAR Detection Band:** Uses forward-diagonal LIDAR sectors matched to the camera's $77^\circ$ field-of-view (FOV) to detect approaching plants without false triggers.
- **Turn & Transition Isolation:** State-gated inspection (`/scan_active`) prevents premature triggers when turning at aisle ends.
- **Active Column Disambiguation:** Guarantees zero cross-aisle misassociation by filtering plant candidates to the active pass column.
- **Machine Learning Classification:**
  - **Healthy & Ripe:** Red (`Gazebo/Red`)
  - **Healthy & Unripe:** Green (`Gazebo/Green`)
  - **Unhealthy / Diseased:** Yellow (`Gazebo/Yellow`)
  - Normalizes hue ($0^\circ/180^\circ$ boundary) to achieve clean class separation.
- **Live RViz Visualization & CSV Logging:** Real-time 3D plant ID markers above foliage and full run logs saved to `data/scout_log.csv`.

---

## 📐 Field & Route Architecture

```
           [North (+y)]
  Col 0       Aisle 1      Col 1       Aisle 2      Col 2       Aisle 3      Col 3
 (x=0.6)      (x=1.4)     (x=2.2)      (x=3.0)     (x=3.8)      (x=4.6)     (x=5.4)
   #4  <--- [Pass 1 ▲]      #5   ===> [Pass 2 ▼] ---> #12  ===> [Pass 3 ▲] ---> #13   [Row 3: y=5.4]
   #3       [       ▲]      #6        [       ▼]      #11       [       ▲]      #14   [Row 2: y=3.8]
   #2       [       ▲]      #7        [       ▼]      #10       [       ▲]      #15   [Row 1: y=2.2]
   #1       [       ▲]      #8        [       ▼]      #9        [       ▲]      #16   [Row 0: y=0.6]
                ▲                          ▼                    ▲                 ▼
            Start/Home                Shift to A2          Shift to A3      Return Home
           [South (-y)]
```

### Route Table
| Pass | Aisle ($x$) | Direction | Heading | Target Column | Plants Visited | Scan Side (Robot Body) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Pass 1** | Aisle 1 ($1.4$) | North ($+y$) | $+\pi/2$ | Col 0 | **#1, #2, #3, #4** | `left` |
| **Pass 2** | Aisle 2 ($3.0$) | South ($-y$) | $-\pi/2$ | Col 1 | **#5, #6, #7, #8** | `right` |
| **Pass 3** | Aisle 2 ($3.0$) | North ($+y$) | $+\pi/2$ | Col 2 | **#9, #10, #11, #12** | `right` |
| **Pass 4** | Aisle 3 ($4.6$) | South ($-y$) | $-\pi/2$ | Col 3 | **#13, #14, #15, #16** | `left` |

---

## 📂 Package Structure

```
farm_field_scout/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── farm_field.launch           # Full launch file (supports gui:=true/false)
├── config/
│   ├── plant_map.yaml              # Plant positions, categories, colors, aisles
│   └── plant_classifier.pkl        # Trained LogisticRegression model + StandardScaler
├── scripts/
│   ├── generate_plant_map.py       # Utility: generates plant_map.yaml (#1-16 visit order)
│   ├── generate_world.py           # Utility: generates farm_field.world SDF models
│   ├── train_classifier.py         # Utility: trains HSV classifier
│   ├── field_navigator.py          # Node: 4-pass zigzag navigator with P heading hold
│   ├── plant_inspector.py          # Node: synchronized LIDAR trigger + vision classifier
│   ├── plant_label_publisher.py    # Node: publishes 3D floating markers (#1-16) to RViz
│   └── scout_logger.py             # Node: logs reports to CSV
├── worlds/
│   └── farm_field.world            # 4x4 crop field Gazebo simulation world
└── data/
    └── scout_log.csv               # Historical run data and accuracy statistics
```

---

## 🚀 Getting Started

### 1. Prerequisites
- **ROS 1 Noetic** (Ubuntu 20.04 / WSL2)
- **TurtleBot3 Packages:**
  ```bash
  sudo apt-get update
  sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-description ros-noetic-turtlebot3-gazebo
  ```
- **Python Dependencies:**
  ```bash
  pip3 install opencv-python scikit-learn pyyaml
  ```

### 2. Build the Workspace
Clone into your catkin workspace `src/` directory and build:
```bash
cd ~/catkin_ws/src
git clone https://github.com/<your-username>/farm_field_scout.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Running the Simulation

#### Fast Headless Mode (Recommended for testing, ~2 minutes):
```bash
roslaunch farm_field_scout farm_field.launch gui:=false
```

#### Visual Mode with Gazebo 3D GUI:
```bash
roslaunch farm_field_scout farm_field.launch gui:=true
```

#### Live Monitoring (in a second terminal):
```bash
source /opt/ros/noetic/setup.bash
rostopic echo /plant_inspection_report
```

---

## 📊 Results & Output

During the run, inspection results are published in real time:
```text
Plant #1  ->  Health: HEALTHY, Ripeness: RIPE  [Actual: HEALTHY, RIPE]  (CORRECT)
Plant #2  ->  Health: HEALTHY, Ripeness: RIPE  [Actual: HEALTHY, RIPE]  (CORRECT)
...
Plant #16 ->  Health: HEALTHY, Ripeness: NOT RIPE  [Actual: HEALTHY, NOT RIPE]  (CORRECT)
```

Inspection reports are automatically logged to `data/scout_log.csv` with timestamps, plant IDs, predictions, actual values, and correctness.
