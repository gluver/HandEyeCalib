# HandEyeCalibration
### Install python dependency
```shell
pip install -r requirements.txt
```
## Hand to eye set up 
### Prepare the data
Take photos of the chessborad and corresponding diverse robot end pose (with changes in all 6d deminsons)
with file stucture(make sure your image timestamp format match `datetime.now().strftime('%Y-%m-%d_%H-%M-%S')`): 
```shell
DI0226/DI
├── DI0225.csv
├── frame2025-02-25_10-20-09.jpg
├── frame2025-02-25_10-26-04.jpg
├── frame2025-02-25_10-27-20.jpg
├── frame2025-02-25_10-27-51.jpg
├── frame2025-02-25_10-28-22.jpg
├── frame2025-02-25_10-29-08.jpg
├── frame2025-02-25_10-29-51.jpg
├── frame2025-02-25_10-30-15.jpg
├── frame2025-02-25_10-31-08.jpg
├── frame2025-02-25_10-31-35.jpg
├── frame2025-02-25_10-32-25.jpg
├── frame2025-02-25_10-33-09.jpg
├── frame2025-02-25_10-33-39.jpg
├── frame2025-02-25_10-34-48.jpg
└── frame2025-02-25_10-35-03.jpg
```
Generate the input file for hand to eye clibration 

``` shell
cp generate_calibrate_input_config_template.yaml generate_calibrate_input_config.yaml
```
Open the yaml you got,change the input generation config sections based on your setup
```yaml
pose_file: 'DI0226/DI/6_data.csv' # Path to the pose file containing the pose data: x,y,z,rx,ry,rz
src_dir: 'DI0226/DI' # Directory containing the images
dst_base: './input_data'
camera_params:
  - 1123.9  # fx
  - 1123.4  # fy
  - 982.364 # cx
  - 567.264 # cy
  - 0.0769521  # k1
  - -0.105434  # k2
  - 6.25417e-05  # p1
  - 3.9459e-5  # p2
  - 0.0428337  # k3
square_size: 100  # Size of a square in the chessboard pattern
pattern_size: [7, 5]  # Size of the chessboard pattern as columns,rows
corner_color: 'black'  # Corner color, 'black' or 'white'
error_threshold: 0.9
```
```shell
python generate_calibrate_input.py --config-file generate_calibrate_input_config.yaml
```
```shell
usage: generate_calibrate_input.py [-h] [--config-file CONFIG_FILE]

Process chessboard images and generate calibration input.

options:
  -h, --help            show this help message and exit
  --config-file CONFIG_FILE
                        Path to the configuration YAML file
```
You can find example output in `input_data` with path `input_data/2025-02-26_14-26-39` `input_data/filtered_pose_data_2025-02-26_14-26-39.csv` as a reference 

### Run calibration script to get the camera to base transfrom matrix
``` shell
cp hand_to_eye_config_template.yaml hand_to_eye_config.yaml
```
####  Change the hand to eye calibration config sections based on your setup
```yaml
pose_file: 'input_data/filtered_pose_data_2025-02-26_14-26-39.csv'
image_folder: 'input_data/2025-02-26_14-26-39'
camera_params:
  - 1123.9  # fx
  - 1123.4  # fy
  - 982.364 # cx
  - 567.264 # cy
  - 0.0769521  # k1
  - -0.105434  # k2
  - 6.25417e-05  # p1
  - 3.9459e-5  # p2
  - 0.0428337  # k3
square_size: 100  # Size of a square in the chessboard pattern
pattern_size: [7, 5]  # Size of the chessboard pattern as columns,rows
corner_color: 'black'  # Corner color, 'black' or 'white'
result_file: 'result_matrix.txt' 
```
```shell
python hand_to_eye.py --config-file hand_to_eye_config.yaml
```
```shell
usage: hand_to_eye.py [-h] [--config_file CONFIG_FILE]

Hand-eye calibration using chessboard images.

options:
  -h, --help            show this help message and exit
  --config_file CONFIG_FILE
                        Path to the YAML configuration file.
```
##### Example output
```shell
WARNING:__main__:Using default configuration template. The parameter settings inside may not adapt to your current setup.
INFO:__main__:Loaded pose vectors from CSV:input_data/filtered_pose_data_2025-02-26_14-26-39.csv
INFO:__main__:Using input image from input_data/2025-02-26_14-26-39
INFO:__main__:Camera to base rotation matrix:
INFO:__main__:[[ 0.99128407 -0.12835439 -0.02968225]
 [-0.12758403 -0.99147255  0.02654234]
 [-0.03283596 -0.02252402 -0.99920692]]
INFO:__main__:Camera to base translation vector:
INFO:__main__:[[1924.24637018]
 [-774.55760843]
 [2664.42951569]]
INFO:__main__:Camera to base pose matrix:
INFO:__main__:[[   0.99128407   -0.12835439   -0.02968225 1924.24637018]
 [  -0.12758403   -0.99147255    0.02654234 -774.55760843]
 [  -0.03283596   -0.02252402   -0.99920692 2664.42951569]
 [   0.            0.            0.            1.        ]]
INFO:__main__:Camera to base pose matrix saved to result_matrix.txt
```

##### Further validation 
You can use the script in `eye_hand_validation.py` to transform your recorded point cloud  into the robot arm's frame with the martix just cauclated from the calibration 
```shell
usage: eye_hand_validation.py [-h] [--points_dir POINTS_DIR] [--matrix_file MATRIX_FILE]

Transform points to robot base frame.

options:
  -h, --help            show this help message and exit
  --points_dir POINTS_DIR
                        Path to the PLY file containing points.
  --matrix_file MATRIX_FILE
                        Path to the file containing cam2base matrix.
```