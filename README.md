# nuScenes point cloud aggregation
## Download the dataset and Install nuScenes devkit
```sh
git clone git@github.com:ZewdieMc/nuScenes_pc_aggregation.git
cd nuScenes_pc_aggregation
mkdir -p data/sets/nuscenes                         # Create folder to save the dataset in
wget https://www.nuscenes.org/data/v1.0-mini.tgz    # Download the mini split
tar -xf v1.0-mini.tgz -C data/sets/nuscenes         # Extract the mini split
pip install nuscenes-devkit &> /dev/null            # Install nuScenes.
```
## Run
```sh
chmod u+x main.py
./main.py
```
## outputs
  <p style="display:flex; flex-wrap:nowrap; gap:2px">
  <img src="./images/filterd.png" height="300" width="auto"/> &nbsp;&nbsp;
  <img src="./images/moving_removed.png" height="300" width="auto"/> &nbsp;&nbsp;
  </p><br>

## 
  <p style="display:flex; flex-wrap:nowrap; gap:2px">
  <img src="./images/moving_f.gif" height="300" width="auto"/> &nbsp;&nbsp;
  <img src="./images/colored.gif" height="300" width="auto"/> &nbsp;&nbsp;
  <img src="./images/colored2.gif" height="300" width="auto"/> &nbsp;&nbsp;
  </p><br>
  
  ## Video
[Watch the video here](https://youtu.be/TQPIMGpfR2s)


## TODO
<font color='red'>Only add color information to the static points and remove the  moving objects(trivial)</font>