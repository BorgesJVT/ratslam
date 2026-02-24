# RatSLAM Scripts

This folder contains utility scripts for post processing data in the RatSLAM pipeline.

## Suggested Folder Structure
```
ratslam/
├── 📁 scripts/                  # Main script location
│   ├── 📄 extract_gps_data.py   # Extract GPS to csv
│   ├── 📄 extract_topics.py     # Extract topological messages
│   ├── 📄 extract_nodes_edges_map.py
│   ├── 📄 show_deadereckoning.m # Matlab odometry plots
│   ├── 📄 lat_long_to_cartesian.m
│   ├── 📄 show_em.py            # Experience map evolution
│   ├── 📄 show_id.py            # Template visualization
│   ├── 📁 exported_data/        # CSV files
│   └── 📁 output_bags/          # Processed ROS bags
│       └── 📁 bag_dataset1/
│           ├── 📄 metadata.yaml
│           └── 📄 bag_dataset1.db3
│   └──📁 Figures/                  # Visualization outputs
        ├── 🖼️ Final_Exp_map.png
        ├── 🖼️ Final_Exp_map.eps
        ├── 🎞️ experience_map_evolution.avi
        └── 📁 map_evolution/        # Partial evolution frames
```                        

### Core Requirements
- Python 3.8+
- pip package manager

### ROS 2 Dependencies
- ROS 2 Humble or Rolling (recommended)
- `rosbag2` package:
  ```bash
  sudo apt-get install ros-$ROS_DISTRO-rosbag2

Before you run these script you **must store a bag with the topics**: /surveyor/ExperienceMap/Map /surveyor/ExperienceMap/RobotPose /surveyor/LocalView/Template /surveyor/PoseCell/TopologicalAction and /surveyor/gps_fix with your ground truth is provided by GPS data. For other bags, verify the topics you want to store.

**Example: Surveyor**
```bash
ros2 bag record -o surveyor_data1_t2 --storage sqlite3 /surveyor/ExperienceMap/Map /surveyor/ExperienceMap/RobotPose /surveyor/view_template /surveyor/PoseCell/TopologicalAction  /surveyor/LocalView/Template /surveyor/gps_fix
```
**Example: irataus**
```bash
ros2 bag record -o irataus_220226 --storage sqlite3 /irat_red/ExperienceMap/Map /irat_red/ExperienceMap/RobotPose /irat_red/view_template /irat_red/PoseCell/TopologicalAction /irat_red/LocalView/Template /overhead/pose 
```

## Python Scripts Overview

The first step is to run the **three python scripts below**:

### `extract_gps_data.py`
Extracts **GPS data** from ROS 2 bag files and exports it to CSV format.


#### Usage:
```bash
python3 extract_gps_data.py <path_to_ros2_bag> --topic <gps_topic_name> --gps_data <output_csv_path>
```

#### Parameters:
| Argument          | Type    | Default      | Description                                           |
|-------------------|---------|--------------|-------------------------------------------------------|
| `input_path`      | string  | -            | Path to ROS 2 bag folder (must contain metadata.yaml) |
| `--topic`, `-t`   | string  | `/gps/fix`   | ROS topic to process                                  |
| `output_path`     | string  | `gps.csv`    | output file for gps data CSV                          |

#### Example:

```bash
python3 extract_gps_data.py output_bags/surveyor_data1_t3 --topic /surveyor/gps_fix --gps_data exported_data/gps.csv
```

### `extract_nodes_edges.py`
Extracts **experience map data** from ROS 2 bag files and exports it to CSV format.


#### Usage:
```bash
python3 extract_nodes_edges.py <path_to_ros2_bag> --topic_root <topic_root_name> --output_path <output_path>
```

#### Parameters:
| Argument          | Type    | Default         | Description                                                  |
|-------------------|---------|-----------------|--------------------------------------------------------------|
| `input_file`      | string  | -               | Path to ROS 2 bag folder (in this case include /bagfile.db3) |
| `topic_root`      | string  | `surveyor`      | topic root name                                              |
| `output_path`     | string  | `exported_data` | output file for links data CSV                               |

#### Example:

```bash
python3 extract_nodes_edges_map.py output_bags/surveyor_data1_t3/surveyor_data1_t3_0.db3 --topic_root surveyor --output_path exported_data
```

### `extract_topics.py`
Extracts **LocalView, TopologicalAction and RobotPose data** from ROS 2 bag files and exports it to CSV format.

#### Usage:
```bash
python3 extract_topics.py <path_to_ros2_bag> --topic_root <topic_root_name> --output_path <output_path>
```

#### Parameters:
| Argument          | Type    | Default         | Description                                           |
|-------------------|---------|-----------------|-------------------------------------------------------|
| `input_path`      | string  | -               | Path to ROS 2 bag folder (must contain metadata.yaml) |
| `topic_root`      | string  | `surveyor`      | topic root name                                       |
| `output_path`     | string  | `exported_data` | output file for links data CSV                        |

#### Example:

```bash
python3 extract_topics.py output_bags/surveyor_data1_t3 --topic_root surveyor --output_path exported_data
```

## Matlab Scripts Overview

The Figures and the error metric are generated by the following matlab scripts:

- show_id.m: shows the experience and visual templates created over time.
- show_deadreckoning.m: shows the ground truth and de poses estimates by the odometry.
- show_em: show de experience map evolution and compute the average error. You can store a video file of the map evolution.
