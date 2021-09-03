# MORMG

Multi-object reactive motion generation based on [composable energy policy](https://arxiv.org/pdf/2105.04962.pdf)

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

Test simple original CEP by running the python file

``python scripts/darias_energy_control/test_simple_darias.py``

``python scripts/darias_energy_control/test_simple_tiago.py``

Test CEP of TIAGo++ with different energy components

``python scripts/darias_energy_control/_test_tiago_lefthand_base_taskgotoAndpathplan.py``
``python scripts/darias_energy_control/_test_cep_tiago_lefthand_base_pathplan.py``
``python scripts/darias_energy_control/_test_simple_tiago_lefthand_base.py``

# Vector fields based on rrt tree
![Vector field](https://github.com/hjw-1014/Multi-Objective-Reactive-Motion-Planning-in-Mobile-Manipulators/blob/main/scripts/_scripts_run/vector_field/rrt_tree_exp_2021-08-17-22_51_47.png)
