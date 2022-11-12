#!/bin/bash
conda create --name scan python=3.8
conda activate scan
conda install -y -c conda-forge vedo trimesh plyfile tqdm imageio rtree addict
conda install -y -c anaconda pillow yaml pyaml pandas
conda install -y -c open3d-admin open3d
conda install -y -c intel scikit-learn



