# husarion-ugv-autonomy

A collection of packages containing autonomous functionalities for Husarion UGV vehicles.

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/husarion_ugv/husarion_ugv_autonomy.webp)

> **Disclaimer.** This is a reference project, provided as is, without warranty of any kind. It makes the robot move on its own. Run it only where the robot can do no harm, watch the robot while it drives, and keep the e-stop in reach. It does not replace your own safety assessment for your robot, your site and your use. The Husarion Cockpit can install it on a robot's user computer (`husarion-project.yaml`); that does not change any of the above.

## 🚀 Demo

Below are two articles describing how to run the autonomy demo — one in simulation and the other on a physical robot.
We recommend starting with the simulation demo, as it lets you explore the autonomy features without the need to configure or interact with the actual hardware.

- [Simulation Demo](./demo-sim.md)
- [Hardware Demo](./demo-hw.md)
- [Map-nav Demo](./demo-mapnav.md): Nav2 with an Ouster 3D LiDAR (or a Slamtec RPLIDAR A3 or S3, not yet driven on a robot) on a robot running Husarion UGV OS, behind its airlock, with a collision monitor
