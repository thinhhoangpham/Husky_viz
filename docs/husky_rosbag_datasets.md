# Husky / ROS Bag Dataset Download Links

Reference list of public ROS bag datasets relevant to the UMD Husky syscall
project. Compiled July 2026.

> **Important caveat that applies to every entry below:** none of these bags
> contain a system-call topic. Syscalls are not a ROS topic — UMD's data only
> has them because they built custom strace-publisher instrumentation. To get
> syscall data from any of these, you must run the ROS nodes yourself and trace
> them with strace/eBPF. These bags supply **sensor/odometry input** and serve
> as **ROS-layer attack targets**, not ready-made syscall data.

---

## 1. SubT Tunnel Circuit — Clearpath Husky (BEST MATCH)

Recorded on an actual Clearpath Husky during teleop, Ouster OS1-64 LiDAR.
Collected by the Army Research Laboratory (same institution as the UMD data).
ROS 1 `.bag` format, compressed with `rosbag compress`.

| File | Course | Size |
|------|--------|------|
| https://subt-data.s3.amazonaws.com/SubT_Tunnel_Ckt/ex_B_route1.bag | Experimental, route 1 | 33 GB |
| https://subt-data.s3.amazonaws.com/SubT_Tunnel_Ckt/sr_B_route1.bag | Safety Research, route 1 | 19.6 GB |
| https://subt-data.s3.amazonaws.com/SubT_Tunnel_Ckt/sr_B_route2.bag | Safety Research, route 2 | 16.3 GB |

Support / docs:
- Usage notes:  https://subt-data.s3.amazonaws.com/SubT_Tunnel_Ckt/usage.txt
- Ground truth + annotations:  https://subt-data.s3.amazonaws.com/SubT_Tunnel_Ckt/support.tgz
- Catkin workspace / launch files:  git clone https://bitbucket.org/subtchallenge/subt_reference_datasets.git
- Repo (source of truth for links):  https://github.com/subtchallenge/tunnel_urban_reference_datasets

Notes:
- Provided install stack targets ROS **Melodic**; bags read fine under Noetic /
  bagpy regardless. For syscall tracing you supply your own nodes anyway.
- Decompress with `rosbag decompress` if playback stutters (~2x size).

---

## 2. SubT Urban Circuit — GVRbot / PackBot (NOT Husky)

Same repo, but recorded on a tracked PackBot, not a Husky. Listed for
completeness — use Tunnel bags above for Husky work.

| File | Course | Notes |
|------|--------|-------|
| https://subt-data.s3.amazonaws.com/SubT_Urban_Ckt/a_lvl_1.bag | Alpha, upper floor | Config 2 |
| https://subt-data.s3.amazonaws.com/SubT_Urban_Ckt/a_lvl_2.bag | Alpha, lower floor | descends stairs |
| https://subt-data.s3.amazonaws.com/SubT_Urban_Ckt/b_lvl_1.bag | Beta, upper floor | Config 2 |
| https://subt-data.s3.amazonaws.com/SubT_Urban_Ckt/b_lvl_2.bag | Beta, lower floor | no Ouster data |

---

## 3. UMA Husky — Natural Environments (Gazebo, labeled)

Husky in Gazebo with 3D LiDAR, stereo, GNSS, IMU, wheel odometry. Provided as
ROS bags with 3D pose ground-truth and automatic object labels. Simulated but
purpose-built for supervised learning / navigation benchmarking.

- Paper (Sánchez et al., Sensors 2022, has dataset link):
  https://www.researchgate.net/publication/362280834_Automatically_Annotated_Dataset_of_a_Ground_Mobile_Robot_in_Natural_Environments_via_Gazebo_Simulations

---

## 4. TorWIC — Clearpath Warehouse (OTTO 100, not Husky)

Clearpath robot (OTTO 100) with Ouster OS1-128. Raw rosbags available on
request per the repo. Good Clearpath-lineage proxy.

- https://github.com/Viky397/TorWICDataset

---

## 5. Generic / small test bags

For pipeline testing rather than realistic Husky load.

- Ouster sample LiDAR data:  https://ouster.com/downloads/sample-lidar-data
- Ouster ROS driver (Melodic/Noetic, replay launch files):  https://github.com/ouster-lidar/ouster-ros
- Autoware datasets (ROS 2):  https://autowarefoundation.github.io/autoware-documentation/main/datasets/
- MATLAB lccSample.bag (LiDAR+camera calibration example):  https://www.mathworks.com/help/lidar/ug/read-lidar-and-camera-data-from-rosbag.html
- NPS LIDAR+camera dataset (campus_1.bag):  https://wiki.nps.edu/pages/viewpage.action?pageId=925958215

---

## Recommended workflow for syscall generation

1. Download a SubT Tunnel Husky bag (e.g. `sr_B_route2.bag`, smallest at 16.3 GB).
2. `rosbag decompress` if needed.
3. Stand up ROS 1 Noetic + Ubuntu 20.04 in Docker (isolated, safe to attack).
4. `rosbag play` the bag into a stack running the Ouster driver + a planner.
5. Trace those node processes with strace (matches UMD's method) or eBPF
   (lower overhead, truer timing).
6. For attack data: stage host-level attacks (reverse shell, injected process,
   miner) in the same container; keep normal and attack in one environment;
   log attack timestamps for window-level labels.

## Reminder on links

These URLs were taken from the dataset repos/pages, not independently
re-downloaded. The SubT files are 2019-era research artifacts on S3 — if a
link 404s, the GitHub repo (entry 1) is the source of truth for any updated
location.
