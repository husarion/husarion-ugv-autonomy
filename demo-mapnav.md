# Map-nav demo: Nav2 behind the airlock of Husarion UGV OS

This is Nav2 with slam_toolbox, or AMCL on a saved map, running on the user computer of a Panther or Lynx with Husarion UGV OS. It uses an Ouster 3D LiDAR, or a Slamtec RPLIDAR S3 2D LiDAR (§8, not tested on a robot yet), whose driver runs in the same compose file. On UGV OS the robot's built-in computer runs the airlock, which carries exactly one drive input from the user computer: `/<namespace>/autonomous/cmd_vel`. The stack in `compose.mapnav.yaml` is built for that:

- **The namespace is your robot's.** UGV OS names a robot `{model}_{serial}`, for example `panther_0957`. Every topic, action and frame of the stack follows it: `/<ns>/navigate_to_pose`, `<ns>/map`, `<ns>/base_link`.
- **Nav2's output** is a stamped twist on `/<ns>/autonomous/cmd_vel`.
- **Nav2's collision monitor** sits between the velocity smoother and that output. It stops the robot for an obstacle inside the footprint plus 15 cm, and slows it to half speed inside the footprint plus 60 cm. It reads the LiDAR's 3D cloud from 0.1 m above the ground to 0.2 m above the robot's top (`ROBOT_HEIGHT`). If the cloud stops arriving for a second, it stops the robot.
- **Recoveries pass the same chain.** Spin and back-up go through the smoother and the collision monitor too.
- **The 3D cloud also feeds the costmaps.** A 2D slice of it, 0.25 m to 1.0 m above the ground, feeds slam_toolbox and AMCL.

- **One LiDAR, chosen by a Compose profile.** `--profile ouster` runs the Ouster's driver and the stack on its cloud (§2 to §6); `--profile rplidar` runs the RPLIDAR S3's driver and the stack on its 2D scan (§8). Give exactly one. With neither, nothing starts.

- **The OAK's video, if you ask for it** (§7): the OAK encodes H.264 on its own chip, and the airlock carries that stream to the robot's Cockpit. No raw image leaves the user computer, and the OAK stays out of navigation.

The older demos (`compose.hardware.yaml`, `compose.simulation.yaml`, `demo-hw.md`, `demo-sim.md`) are unchanged.

> **This is a reference project, not a product.** Tune it for your robot and your site. Keep the e-stop in reach whenever the robot drives autonomously.

## Before you start

- Docker with the compose plugin on the user computer, which sits at `10.15.20.3` on the robot's LAN.
- The robot runs Husarion UGV OS. Its fleet side is ROS 2 domain 0 with Cyclone DDS, which is what this stack uses.
- Your robot's namespace: the robot's name in the Cockpit, or `ROS_NAMESPACE` on the robot. Below it is `NS`:

  ```bash
  export NS=panther_0957   # yours
  ```

## 1. The user computer's clock

The robot judges how old a drive command is by its own clock, so the user computer must keep the built-in computer's time. Point chrony at it:

```bash
sudo apt install -y chrony
echo "server 10.15.20.2 iburst prefer" | sudo tee /etc/chrony/sources.d/husarion-robot.sources
sudo chronyc reload sources
chronyc sources        # 10.15.20.2 must be the selected source, marked ^*
```

`prefer` makes chrony follow the robot even while Ubuntu's internet pools are reachable. If `^*` sits on another source, remove the `pool` lines from `/etc/chrony/chrony.conf` and restart chrony.

## 2. The LiDAR on the robot: `components.yaml`

The robot describes the LiDAR (and its sensor gate) in its own URDF, so the LiDAR's frames are on the robot's TF tree like every other part. In the Cockpit open **Manage › Robot › Driver files**, edit `components.yaml` and apply; the driver restarts with the new description. For a Panther with the DEV09 large gate and an Ouster on the front of the mounting rails, centred:

```yaml
components:
  - type: DEV09
  - type: LDR17            # Ouster OS2-64; LDR10 for an OS0-32
    parent_link: mount_link
    xyz: 0.22 0.0 0.0      # the front crossbar of the rails, centred
    rpy: 0.0 0.0 0.0       # connector facing the rear
```

On a Panther the mounting rails run from x = −0.229 m to +0.229 m about `mount_link` (the body mesh), so a sensor on the 20 mm front crossbar has its base centre at x = 0.22 m, right above the front axle. `xyz` is the centre of the sensor's base on the rail: `x` forward from the middle of the rails, `y` to the left, `z` up from the rail. With the connector facing forward, set `rpy: 0.0 0.0 3.14159`. The model places the lidar origin 74.3 mm (OS2) or 36.2 mm (OS0, OS1) above the base and turns it 180°, as the sensor itself does. A unit can differ by a few millimetres; its own value is in the sensor's `lidar_intrinsics`. This needs a driver image whose `husarion_components_description` has the OS2 codes and the 180° turn.

Check: drive the robot towards a wall. In the Cockpit's 3D view the scan must show the wall in front of the robot, not behind it.

## 3. The Ouster driver

The driver runs as the `ouster` service of `compose.mapnav.yaml`, with the same Cyclone DDS profile as Nav2. Set the sensor to the static address `10.15.20.5` first (Ouster's web page or its HTTP API), or set `OUSTER_HOSTNAME`.

If the `husarion-ouster` snap is installed on the user computer, stop it first. Two drivers would fight over the sensor:

```bash
sudo snap stop --disable husarion-ouster
```

What the service sets, and why:

| Setting | Why |
| -- | -- |
| `ouster_ns:=$ROBOT_NAMESPACE` | The cloud is `/$ROBOT_NAMESPACE/points` |
| `pub_static_tf:=false` | The robot's URDF already places the sensor (step 2); a second transform would give `os_lidar` two parents |
| `lidar_frame` = `point_cloud_frame` = `$ROBOT_NAMESPACE/os_lidar` | The cloud is stamped in the URDF's lidar frame. The two must be exactly the same string, or the driver silently falls back to the lidar frame. With a `name:` on the component the frame is `<name>_os_lidar` |
| `proc_mask:=PCL` | The point cloud only. The driver's own `SCAN` output would publish a second `scan` next to the stack's slice; images, IMU and telemetry only cost CPU |
| `timestamp_mode:=TIME_FROM_ROS_TIME` | Otherwise the sensor keeps the clock mode it saved, and stamps from another clock break every transform lookup |
| `lidar_mode:=1024x10` | 1024 columns at 10 Hz; the stack's scan slice has one bin per column |
| `use_system_default_qos:=false` | Sensor data, best effort; the stack subscribes the same way |
| `persist_config:=false` | The sensor's saved configuration stays as it is |

Check that the cloud arrives at 10 Hz once the stack is up:

```bash
docker compose -f compose.mapnav.yaml exec ouster bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic hz /$NS/points"
```

## 4. Start

```bash
ROBOT_NAMESPACE=$NS docker compose -f compose.mapnav.yaml --profile ouster up -d
docker compose -f compose.mapnav.yaml --profile ouster logs -f
```

The settings, each with its default:

| Variable | Default | What |
| -- | -- | -- |
| `ROBOT_NAMESPACE` | `panther` | Your robot's namespace |
| `ROBOT_MODEL` | `panther` | `panther` or `lynx` |
| `OBSERVATION_TOPIC` | `/$ROBOT_NAMESPACE/points` | The LiDAR's cloud |
| `LIDAR_PROFILE` | `os0_32` | `os0_32` or `os2_64`, below |
| `ROBOT_HEIGHT` | `0.7` | The top of your robot with everything mounted on it, in metres above the ground. The costmaps and the collision monitor treat points up to this height plus 0.2 m as obstacles, so a table edge the robot would hit stops it. **Measure your robot** |
| `SCAN_MIN_HEIGHT`, `SCAN_MAX_HEIGHT` | `0.25`, `1.0` | The 2D slice for SLAM and AMCL, in metres above the ground: above grass and kerbs, below the canopy. On a slope the ground can enter the slice at range; raise the floor if you see arcs of ground in the map |
| `SLAM` | `True` | `True`: map with slam_toolbox. `False`: AMCL on a saved map |
| `MAP_NAME` | `map` | With `SLAM=False`, the saved map's directory under `./maps` |
| `OAK_IP`, `OAK_BITRATE` | `10.15.20.6`, `3400000` | With `--profile oak` (§7): the OAK's address and its H.264 bitrate in bit/s |
| `RPLIDAR_DEVICE`, `RPLIDAR_FRAME`, `RPLIDAR_SPEED_CAP` | `/dev/ttyUSB0`, `laser`, `os2_64` | With `--profile rplidar` (§8) instead of `--profile ouster`: the S3's serial port, its frame below the namespace, and its 0.3 m/s speed cap (empty: none) |

**Release the e-stop before sending a goal.** Nav2 reads the robot's e-stop (`/$NS/hardware/e_stop`, which the airlock carries to the user computer) and aborts every goal while it is engaged, or before its first message arrives. The robot latches the e-stop whenever its driver restarts, including after a `components.yaml` save.

At start the stack reads the rail's height (`$NS/mount_link` above `$NS/base_footprint`) from the robot's TF. It fails after 30 s if the robot's TF is not visible from the user computer. Everything inside the robot's footprint up to `ROBOT_HEIGHT` is the robot itself (the gate, cameras, antennas) and is dropped from the cloud.

### LiDAR profiles

| | `os0_32` | `os2_64` |
| -- | -- | -- |
| Sensor | Ouster OS0-32 (90° vertical field) | Ouster OS2-64 (22.5° vertical field) |
| The slice's minimum range | 0.5 m | 0.8 m |
| Linear speed | the stack's own (0.8 m/s) | **at most 0.3 m/s, at all times** |

> **The OS2-64 cannot see near the robot.** Its lowest beam meets the ground about 2 m out. A 10 cm obstacle closer than about 1.5 m, and a table edge closer than about 2 m, are not seen, so the costmaps and the collision monitor can miss them. With `os2_64` the robot never drives faster than 0.3 m/s. **Low and overhanging obstacles near the robot are not seen; watch the robot while it drives.**

## 5. Mapping and a saved map

With `SLAM=True` slam_toolbox maps. The map is published every 5 s, and an autosaved copy goes to `./maps/map.{yaml,png}` every 15 s.

A saved map that AMCL can start on is a directory `./maps/<name>/` with `map.yaml`, its image and a sidecar `map.georef.yaml`. The sidecar carries `map_sha256`, the sha256 of the map image, and `saved_pose`, the robot's pose `{x, y, yaw}` in the map frame when the map was saved. AMCL starts at that pose, so start the robot where the map was saved:

```bash
ROBOT_NAMESPACE=$NS SLAM=False MAP_NAME=<name> docker compose -f compose.mapnav.yaml --profile ouster up -d
```

The stack refuses to start when the sidecar is missing, has no `saved_pose`, or belongs to another image. AMCL never starts at 0, 0, 0 on a saved map by accident.

## 6. Stop

```bash
docker compose -f compose.mapnav.yaml --profile ouster --profile rplidar --profile oak down
```

Naming every profile stops whichever LiDAR runs and the OAK's service when it runs (§7); a profile that is not running changes nothing.

## 7. The OAK's video in the Cockpit (optional)

An OAK-D Pro PoE on the robot's LAN (`10.15.20.6` on the reference robots) can show its colour camera in the Cockpit. One service does it, `oak`, behind the compose profile `oak`, so nothing about it starts unless you ask. It runs the depthai image (`husarion/depthai`: stock depthai-ros 2.13.0 plus Husarion's H.264 relay) with its launch file `/ros2_ws/launch/oak_h264.launch.py`, which loads two components into one container, so each packet crosses from one to the other in-process:

- **depthai's camera** (node `oak`): the OAK encodes its colour stream as H.264 on its own chip and the driver publishes it as `ffmpeg_image_transport_msgs/FFMPEGPacket` on `/$NS/oak/rgb/image_raw/compressed`, about 0.4 MB/s at 1280×720 and 30 fps. No raw image is published.
- **the relay** (node `ffmpeg_to_compressed_video`) copies each packet's bytes into a `foxglove_msgs/CompressedVideo` on `/$NS/oak/rgb/image_raw/h264`, with no decode and no re-encode. The robot's airlock carries `CompressedVideo` but not yet `FFMPEGPacket`; the relay goes when it does.

On the robot, Husarion UGV OS carries `/$NS/oak/rgb/image_raw/h264` to its camera node through an inbound H.264 lane, and the camera node passes the stream to the Cockpit as it is. That lane is part of the robot's own configuration (UGV OS), not of this project.

If the `husarion-depthai` snap is installed on the user computer, stop it first. Two drivers would fight over the camera:

```bash
sudo snap stop --disable husarion-depthai
```

Describe the OAK in `components.yaml` as well (Manage › Robot › Driver files), so its frames are on the robot's TF tree like the LiDAR's. `CAM12` is the OAK-D Pro PoE (`CAM11` is the USB OAK-D Pro); measure `xyz` and `rpy` on your robot, these values are only an example:

```yaml
  - type: CAM12            # Luxonis OAK-D Pro PoE
    parent_link: mount_link
    xyz: 0.20 0.0 0.10     # yours
    rpy: 0.0 0.0 0.0
```

Start the stack with the OAK:

```bash
ROBOT_NAMESPACE=$NS docker compose -f compose.mapnav.yaml --profile ouster --profile oak up -d
```

What the launch file sets, and why (`namespace`, `ip` and `bitrate` come from the compose file; the rest is fixed in the launch file, `depthai-docker` `launch/oak_h264.launch.py`):

| Setting | Why |
| -- | -- |
| depthai's `Camera` component, not its `camera.launch.py` | That launch file also publishes the camera's own description and transforms; the robot's URDF already places the OAK (`CAM12`) |
| `camera.i_ip:=$OAK_IP` (default `10.15.20.6`) | The PoE camera's address |
| `camera.i_pipeline_type:=RGB`, `camera.i_nn_type:=none`, IMU and IR off | The colour camera only: no depth, no network on the chip, no IR projector |
| `rgb.i_low_bandwidth:=true`, `rgb.i_publish_compressed:=true` | The chip encodes, and the driver publishes the encoded stream instead of raw frames |
| `rgb.i_low_bandwidth_profile:=0` | H.264 baseline: no B-frames, so the browser shows each frame as it arrives |
| `rgb.i_low_bandwidth_bitrate:=$OAK_BITRATE` (default 3.4 Mbit/s), `rgb.i_low_bandwidth_frame_freq:=30` | The bitrate of the reference robots' snap; a keyframe every second at 30 fps, so a new viewer waits at most a second for a picture |
| `rgb.i_fps:=30.0` | 30 frames a second at the driver's default 1280×720 |

Check that the stream arrives at 30 Hz on the user computer:

```bash
docker compose -f compose.mapnav.yaml exec oak bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic hz /$NS/oak/rgb/image_raw/h264"
```

## 8. A Slamtec RPLIDAR S3 instead of the Ouster

> **Not tested on a robot yet.** This path has run only against a simulated S3 (a fake robot replaying the driver's scan geometry). Watch the robot closely the first time it drives on it, with the e-stop in reach.

The `rplidar` profile replaces the `ouster` profile: it runs the S3's driver (`rplidar`, from `husarion/rplidar`, sllidar_ros2) and Nav2 on its 2D scan (`navigation_rplidar`). Use it instead of `--profile ouster`, never with it:

```bash
ROBOT_NAMESPACE=$NS docker compose -f compose.mapnav.yaml --profile rplidar up -d
docker compose -f compose.mapnav.yaml --profile rplidar logs -f
```

Everything else is as above: the clock (§1), the e-stop, `SLAM`, `MAP_NAME` and a saved map (§5), the OAK (`--profile oak`, §7), stopping (§6). `LIDAR_PROFILE`, `ROBOT_HEIGHT`, `SCAN_MIN_HEIGHT`, `SCAN_MAX_HEIGHT` and `OBSERVATION_TOPIC` belong to the Ouster and do nothing here.

**What differs from the Ouster.** The S3 measures one horizontal plane, so the stack takes its scan as it is: no crop box and no slice. slam_toolbox, AMCL, both costmaps and the collision monitor (the same stop and slowdown polygons) all read `/$NS/scan`. As with the Ouster, the stack reads the rail height (`$NS/mount_link`) from the robot's TF at start.

**Linear speed is capped at 0.3 m/s, at all times**, in the controller and the velocity smoother, the same cap as the OS2-64 (§4), because the S3 cannot see what is below or above its plane. The compose file does it with `lidar_profile:=os2_64` (`RPLIDAR_SPEED_CAP`, default `os2_64`); on a 2D scan only the cap of that profile acts. To drive at the stack's own speed (0.8 m/s) once you have judged your site, remove the cap knowingly:

```bash
RPLIDAR_SPEED_CAP= ROBOT_NAMESPACE=$NS docker compose -f compose.mapnav.yaml --profile rplidar up -d
```

`RPLIDAR_SPEED_CAP` set but empty means no cap; there is no value in between with this image.

**Clearing in the costmaps.** With the image pinned here, the costmaps clear what the S3 no longer sees only in a 60° cone in front of the sensor (a depth camera's frustum); elsewhere an obstacle that has gone stays marked until it decays, within a second. The next autonomy image clears over the whole circle (husarion_ugv_autonomy_ros, `nav2_params.yaml`, the `laserscan` source); it takes effect when this file pins that image.

> **The S3 sees only its own plane.** Anything lower than the sensor (a kerb, a box, a person's feet) or higher (a table top, a barrier arm) is not seen, by the costmaps or by the collision monitor. Use it where obstacles reach the height of the scan plane, and watch the robot while it drives.

**Connect it** to a USB port of the user computer. The driver opens `RPLIDAR_DEVICE` (default `/dev/ttyUSB0`) at 1,000,000 baud, the S3's rate, in its `DenseBoost` mode. A name that does not change between boots is safer:

```bash
ls /dev/serial/by-id/        # e.g. usb-Silicon_Labs_CP2102N_..._0001-if00-port0
export RPLIDAR_DEVICE=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_..._0001-if00-port0
```

If the `husarion-rplidar` snap is installed on the user computer, stop it first; two drivers cannot share the port:

```bash
sudo snap stop --disable husarion-rplidar
```

**Place it on the robot** in `components.yaml` (Manage › Robot › Driver files), like the Ouster in §2. `LDR06` is the RPLIDAR S3 (`rplidar_s3` also works with a newer driver image). The robot's URDF then carries `$NS/laser` (`$NS/<name>_laser` with a component `name:`, then set `RPLIDAR_FRAME=<name>_laser`), and the driver stamps the scan in that frame and publishes no transform. For a Panther with the S3 centred on the front crossbar of the rails:

```yaml
components:
  - type: LDR06            # Slamtec RPLIDAR S3
    parent_link: mount_link
    xyz: 0.22 0.0 0.0      # yours: the centre of the sensor's base on the rail
    rpy: 0.0 0.0 0.0       # yours: check it with the wall test below
```

`xyz` is the centre of the sensor's base, as for the Ouster. The model puts the scan plane 30.5 mm above the base and turns `laser` 180° to match the driver's angles. If the sensor is turned on its mount, put the turn in `rpy`.

> **Keep the robot out of the scan plane.** With no crop box, every part of the robot the S3 sees (a gate post, an antenna, a mast) is an obstacle. Inside the footprint plus 15 cm the collision monitor's stop polygon then holds the robot still, and Nav2 will not move it. Mount the S3 above everything else on the robot, or where nothing of the robot is level with it.

Check: the scan arrives at about 10 Hz, and in the Cockpit's 3D view the wall the robot faces is in front of it, not behind it.

```bash
docker compose -f compose.mapnav.yaml exec rplidar bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic hz /$NS/scan"
```

**In the Cockpit's installer** (when it ships) the choice is a knob of the project, not two commands: the LiDAR is `ouster` (profile `ouster`, with the Ouster model as a second field) or `rplidar_s3` (profile `rplidar`), and the installer passes the matching `--profile`. Until the manifest schema is published, the draft is:

```yaml
- key: LIDAR
  label: LiDAR
  type: choice
  default: ouster
  options:
    - {value: ouster, label: Ouster, compose_profiles: [ouster]}
    - {value: rplidar_s3, label: RPLIDAR S3, compose_profiles: [rplidar]}
```
