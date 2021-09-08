# MORMG | Multi-Objective Reactive Motion Generation in Mobile-Manipulators

Multi-object reactive motion generation based on [composable energy policy](https://arxiv.org/pdf/2105.04962.pdf)

# TIAGo++ mobile manipulator
![TIAGo++](https://github.com/hjw-1014/Multi-Objective-Reactive-Motion-Planning-in-Mobile-Manipulators/blob/devel/TIAGo%2B%2B.png)

## Installation

* Tested in Python 3.7

Install the repository. Inside the repository

``pip install -e  .``

### Install all packages
``pip install -r requirements.txt``

### Requirements

``pip install numpy matplotlib pybullet future``

* Pinocchio

``conda install pinocchio -c conda-forge``

* PyTorch

``conda install pytorch torchvision torchaudio cudatoolkit=10.2 -c pytorch``

## Run test for OSC(Operation Space control)

* Test simple OSC only for left-arm control

`` python /scripts/darias_energy_control/OSC/TIAGo_OnePoint_Baseline.py ``


* Test simple OSC for whole-body control(Left arm and base)

`` python /scripts/darias_energy_control/OSC/TIAGo_OnePoint_with_holoBaseCXY.py ``


## Run test for CEP(Composable Energy Control)

* Test simple original CEP by running the python file

``python scripts/darias_energy_control/test_simple_darias.py``

``python scripts/darias_energy_control/test_simple_tiago.py``

* Test CEP of TIAGo++ with different energy components

``python scripts/darias_energy_control/_test_tiago_lefthand_base_taskgotoAndpathplan.py``

``python scripts/darias_energy_control/_test_cep_tiago_lefthand_base_pathplan.py``

``python scripts/darias_energy_control/_test_simple_tiago_lefthand_base.py``

## Result pictures

![RRT tree](https://github.com/hjw-1014/Multi-Objective-Reactive-Motion-Planning-in-Mobile-Manipulators/blob/main/scripts/_scripts_run/figures/rrt_tree_0705_1.png)
* RRT tree
 
![Vector field](https://github.com/hjw-1014/Multi-Objective-Reactive-Motion-Planning-in-Mobile-Manipulators/blob/main/scripts/_scripts_run/vector_field/rrt_tree_exp_2021-08-17-22_51_47.png)
* Vector fields based on rrt tree

![One path generated by RRT-based vector field](https://github.com/hjw-1014/Multi-Objective-Reactive-Motion-Planning-in-Mobile-Manipulators/blob/main/scripts/_scripts_run/figures/run_exp_2021-08-09-16_59_44.png)
* One path generated by rrt-based vector field

