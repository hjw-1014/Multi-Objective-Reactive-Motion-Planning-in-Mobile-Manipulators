# MORMG

Multi-object reactive motion generation based on composable energy policy

## Installation

Tested in Python 3.7

Install the repository. Inside the repository

``pip install -e  .``

### Install all packages
``pip install -r requirements.txt``

### Requirements

``pip install numpy matplotlib pybullet future``

Pinocchio

``conda install pinocchio -c conda-forge``

PyTorch

``conda install pytorch torchvision torchaudio cudatoolkit=10.2 -c pytorch``


## Test Repository

Test everything is working by running the python file

``python scripts/darias_energy_control/test_simple_darias.py``

``python scripts/darias_energy_control/test_simple_tiago.py``

# Results
![Vector field](https://github.com/hjw-1014/Multi-Objective-Reactive-Motion-Planning-in-Mobile-Manipulators/blob/main/scripts/_scripts_run/vector_field/rrt_tree_exp_2021-08-17-22_51_47.png)
