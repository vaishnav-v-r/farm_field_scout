#!/usr/bin/env python3
"""
Utility script (NOT a ROS node - run once by hand) to generate the Gazebo
.world file from config/plant_map.yaml. Re-run this any time plant_map.yaml
changes, then relaunch Gazebo to see the update.
"""
import yaml

with open("config/plant_map.yaml") as f:
    data = yaml.safe_load(f)

plants = data["plants"]

plant_template = """
    <model name="plant_{id}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <visual name="stem_visual">
          <pose>0 0 0.1 0 0 0</pose>
          <geometry><cylinder><radius>0.02</radius><length>0.2</length></cylinder></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Brown</name>
            </script>
          </material>
        </visual>
        <collision name="stem_collision">
          <pose>0 0 0.1 0 0 0</pose>
          <geometry><cylinder><radius>0.02</radius><length>0.2</length></cylinder></geometry>
        </collision>
        <visual name="foliage_visual">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><sphere><radius>0.12</radius></sphere></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/{color}</name>
            </script>
          </material>
        </visual>
        <collision name="foliage_collision">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><sphere><radius>0.12</radius></sphere></geometry>
        </collision>
      </link>
    </model>
"""

plant_blocks = "".join(
    plant_template.format(id=p["id"], x=p["x"], y=p["y"], color=p["color"])
    for p in plants
)

world_content = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="farm_field">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <gui>
      <camera name="user_camera">
        <pose>-2 -2 4 0 0.6 0.8</pose>
      </camera>
    </gui>

{plant_blocks}
  </world>
</sdf>
"""

with open("worlds/farm_field.world", "w") as f:
    f.write(world_content)

print(f"Generated worlds/farm_field.world with {len(plants)} plants")
