# SOI Drone Dash 2023

## Instructions

1. Clone the repository such that `demo_node.py` file has the same path as provided
in the original workspace setup [GitHub](https://github.com/shashankp28/DroneDash-23).

```
git clone https://github.com/shashankp28/drone-dash-2023.git <correct_path>
```

**Note**: *The final path of `demo_node.py` must be `~/catkin_ws/src/DroneDash-23/scripts/demo_node.py`*

3. Install requirements:
```
pip install -r requirements.txt
```

4. In the workspace directory run
```
roslaunch dronedash demo.launch rviz:=true world:=0
```