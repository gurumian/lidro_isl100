# LIDRO ISL-100
LIDRO ISL-100 ros2 driver.

## 1. Network Settings

`netplan` can be an option to connect to the lidar.

#### How to install `netplan`
```bash
sudo apt install -y netplan.io
```


It is assumed that it will be configured as follows:
- Host: 192.168.1.100
- LiDAR: 192.168.1.10


Update `/etc/netplan/01-network-manager-all.yaml` as follows:
```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
      eth2:
          addresses:
              - 192.168.1.100/24
          routes:
              - to: 192.168.1.10

```


## 2. Config file

### 2.1 Basic Configuration
```yaml
lidro_isl100_node:
    ros__parameters:
        # stream-mode options:
        # [1] 96×480 (~300m)
        # [2] 96×960 (~300m)
        # [3] 192×480 (~150m)
        # [4] 192×960 (~150m)
        mode: 1
        frame_id: lidar_link
        host_addr: "192.168.1.100"
        host_ctrl_port: 5051
        host_data_port: 5055
        lidar_addr: "192.168.1.10"
        lidar_ctrl_port: 1011  
        frequency: 10.0 #(Hz)
        publish_ambient_data: true
        min_distance: 0.5 #(m)
        max_distance: 300.0 #(m) maximum distance varies depending on the stream-mode
```

## 3. Run the Samples
```bash
colcon build --symlink-install
source install/local_setup.bash

ros2 launch bringup.launch.py
```

To visualize,
```bash
rviz2 -d lidro_isl100/rviz/config_isl100.rviz
```

## 4. Publications
- `/lidro/image`: `<sensor_msgs::msg::Image>`
- `/lidro/points`: `<sensor_msgs::msg::PointCloud2>`

