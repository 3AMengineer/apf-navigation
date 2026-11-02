# apf-navigation

**Reactive autonomous navigation with Artificial Potential Fields (APF) in ROS 2**, written from scratch. The goal pulls the rover in and every obstacle pushes it away, recomputed 10 times a second from live sensor data. It is implemented twice, for two very different sensors:

- **360° 2D LiDAR:** full-coverage obstacle perception
- **Intel RealSense D455 depth camera:** depth images turned into a virtual laser scan, with only an 87° field of view

Runs on the [four_wheel_diff_drive](https://github.com/3AMengineer/four_wheel_diff_drive) rover in Gazebo.

<!-- Add GIFs here: docs/apf_lidar.gif (Gazebo + RViz force arrows) and docs/apf_depth.gif -->

---

## Pipeline

```
/scan or depth image
      │
      ▼
 polar → Cartesian points (body frame)
      │
      ▼
 obstacle clustering        consecutive points < 0.3 m apart = one obstacle, ≥ 2 points
      │
      ▼
 closest point per cluster  one repulsive source per obstacle, not per beam
      │
      ▼
 forces in the world frame  F_att = k_a · (goal − p)
                            F_rep = k_r · (1/d − 1/d₀) / d²   for d < d₀
      │
      ▼
 heading + speed            yaw_des = atan2(F_y, F_x)
                            v = min(v_max, |F|),  ω = k_θ · wrap(yaw_des − yaw)
      │
      ▼
 /cmd_vel  (+ force arrows on /visual_debug in RViz)
```

Clustering before computing repulsion is the key design choice. Without it, a wall made of 50 LiDAR beams pushes 50 times harder than a pole made of 2 beams.

---

## Nodes

| Entry point | Sensor | Source |
|---|---|---|
| `apfa_final` | 2D LiDAR (`/scan`) + `/odom` | `odom_subscriber4.py` |
| `apfa_depth_final2` | Depth image (`/camera/depth/image_raw`) + `/odom` | `apfa_depth_final2.py` |
| `apfa_1` … `apfa_3`, `apfa_depth*` | Earlier iterations, kept to show how the controller evolved | |

### Depth camera version

A depth image has no "ranges" array, so the node builds one:

1. take a 60-pixel horizontal strip through the image centre (the obstacle band at rover height),
2. take the minimum depth in every column to get a **virtual laser scan**,
3. assign every column a bearing across the 87° FOV,
4. feed the result into the same clustering and APF pipeline as the LiDAR version.

The repulsive forces and their resultant are published as RViz arrows (red = repulsion, green = attraction), which made tuning possible.

---

## Run it

```bash
cd ~/ws_diffdrive/src
git clone https://github.com/3AMengineer/four_wheel_diff_drive.git
git clone https://github.com/3AMengineer/apf-navigation.git odom_publisher
cd .. && colcon build && source install/setup.bash

ros2 launch four_wheel_diff_drive gazebo_model.launch.py      # LiDAR rover
ros2 run odom_publisher apfa_final                            # APF on LiDAR

ros2 launch four_wheel_diff_drive depth_gazebo_model.launch.py  # depth rover
ros2 run odom_publisher apfa_depth_final2                       # APF on depth
```

The goal is set in the node (`goal_position`), and the gains are `ka`, `kr` and the influence radius `Frep_radius`.

---

## What broke, and what I learned

- **Local minima.** When attraction and repulsion cancel (for example a goal directly behind a wall), the rover stops making progress. That is the fundamental limit of purely reactive planning, and it's why the real rover moved to Nav2 with a global planner.
- **Oscillation in narrow passages.** Repulsion from both walls fights the heading controller. Gains, influence radius and the speed cap all had to be tuned together.
- **LiDAR vs depth camera.** With 360° coverage the rover "remembers" an obstacle beside it. With an 87° camera, that obstacle disappears the moment it leaves the FOV, and the rover can turn straight back into it. Changing the sensor changed the behaviour, not just the input.

## Next

This work fed directly into the UAS-DTU autonomous rover for the **DARPA Triage Challenge**: Nav2, a Blickfeld 3D LiDAR, LiDAR-inertial odometry and ArduRover on real hardware.

## Author

**Shourya Sharma**, Mathematics & Computing @ DTU, UAS-DTU
[LinkedIn](https://www.linkedin.com/in/shourya-sharma-6a365a341)
