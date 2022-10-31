# Virtual Scanning Simulations in Python

This repository includes code and sample data for creating virtual scanning simulations like the ones below.

## Range scanning simulation

```python
sc=Scanner(model="bunny",steps=360)
sc.scan(n_rays=100)
sc.scanVisualize(texture=True)
sc.pointsVisualize(with_mesh=True)
```

<p float="left">
  <img style="width:250px;" src="./data/bunny/scan.gif">
  <img style="width:250px;" src="./data/bunny/pts_mesh.gif">
</p>


## MVS simulation

```python
sc=Scanner(model="airplane",steps=360)
sc.scanMVS(n_points=4000)
sc.scanVisualizeMVS(texture=False)
sc.pcVisualize(with_mesh=False,type="mvs")
```

<p float="left">
  <img style="width:250px;" src="./data/airplane/scan_mvs.gif">
  <img style="width:250px;" src="./data/airplane/pc.gif">
</p>

