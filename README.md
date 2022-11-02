# Virtual Scanning Simulations in Python

This repository contains code and sample data for creating virtual scanning simulations like the ones below.
The code produces point clouds with realistic characteristics such as non-uniformity and missing data from self-occlusions. However, it is mainly meant for visualising the scanning process.

## Range scanning simulation

```python
sc=Scanner(model="bunny",steps=360)
sc.scan(n_rays=100)
sc.scanVisualize(texture=True)
sc.pointsVisualize(with_mesh=True)
```

<p float="left">
  <img style="width:400px;" src="./data/bunny/scan.gif">
  <img style="width:400px;" src="./data/bunny/pts_mesh.gif">
</p>


## MVS simulation

```python
sc=Scanner(model="airplane",steps=360)
sc.scanMVS(n_points=4000)
sc.scanVisualizeMVS(texture=False)
sc.pcVisualize(with_mesh=False,type="mvs")
```

<p float="left">
  <img style="width:400px;" src="./data/airplane/scan_mvs.gif">
  <img style="width:400px;" src="./data/airplane/pc.gif">
</p>


## Real scanning

You can find example code for MVS and LiDAR scanning visualisation of the ETH3D dataset [in this repository](https://github.com/raphaelsulzer/eth3d).

<p float="left">
  <img style="width:400px;" src="https://github.com/raphaelsulzer/eth3d/blob/master/out/terrace_lidar.png?raw=true">
  <img style="width:400px;" src="https://github.com/raphaelsulzer/eth3d/blob/master/out/terrace_mvs.png?raw=true">
</p>


<p float="left">
  <img style="width:400px;" src="https://github.com/raphaelsulzer/eth3d/blob/master/out/terrace_lidar_sensor.png?raw=true">
  <img style="width:400px;" src="https://github.com/raphaelsulzer/eth3d/blob/master/out/terrace_mvs_sensor.png?raw=true">
</p>


<p float="left">
  <img style="width:400px;" src="https://github.com/raphaelsulzer/eth3d/blob/master/out/terrace_lidar_mesh.png?raw=true">
  <img style="width:400px;" src="https://github.com/raphaelsulzer/eth3d/blob/master/out/terrace_mvs_mesh.png?raw=true">
</p>





