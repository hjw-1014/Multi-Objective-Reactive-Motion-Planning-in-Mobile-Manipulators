import torch
import numpy as np
from cep.utils import eul2rot

from cep.kinematics import TiagoRobot_lefthand_Base
from cep.controllers import Multi_EBMControl, EBMControl, EnergyTree


from cep import maps
from cep import energies



GPU_ON = torch.cuda.is_available()
GPU_ON = False
if GPU_ON:
    device = torch.device("cuda:0")
else:
    device = torch.device("cpu")


def cep_simple_model_tiago_lefthand_base():  # TODO: 08.06
    ##Get all the FK maps##
    tiago_kin_lefthand_base = TiagoRobot_lefthand_Base()
    fk_map = maps.FK_ALL_lefthand_base(tiago_kin_lefthand_base)

    ## End Effector Branch ##
    b = torch.Tensor([1.8, 0., 0.8])
    R = eul2rot(torch.Tensor([0., 0., 0.]))

    H = torch.eye(4)
    H[:3, :3] = R
    H[:3, -1] = b

    A = torch.eye(6)
    ee_goto_leaf = energies.TaskGoToLeaf(dim=6, b=b, A=A, R=H, var=torch.eye(6)*10.)
    #ee_obj_avoid = energies.ObjAvoidLeaf()  # TODO: add branches here???
    pick_map = maps.SelectionMap(idx=6)
    ee_energy_tree = EnergyTree(branches=[ee_goto_leaf], map=pick_map)
    #########################
    q_branches = [ee_energy_tree]   # TODO: add branches here???
    energy_tree = EnergyTree(branches=q_branches, map=fk_map).to(device)
    policy = EBMControl(energy_tree=energy_tree, device=device, optimization_steps=5, dt=0.005, n_particles=10000)
    return policy


def joint_cep_simple_model_tiago():  # TODO: 06.26 Done

    identity_map = maps.SimplePosVel(dim=10)
    q_goto_leaf = energies.JointGoToLeaf()

    q_branches = [q_goto_leaf]   # TODO: add branches here
    energy_tree = EnergyTree(branches=q_branches, map=identity_map).to(device)
    policy = EBMControl(energy_tree=energy_tree, device=device, optimization_steps=5, dt=0.005, n_particles=10000)
    return policy

def jsc_and_goto_cep_simple_model_lefthandBase():  # TODO: 06.28, 06.29

    # TODO: TaskGoToLeaf
    ##Get all the FK maps##
    tiago_kin = TiagoRobot_lefthand_Base()
    fk_map = maps.FK_ALL(tiago_kin)
    pick_map = maps.SelectionMap(idx=6)
    ## End Effector Branch ##
    b = torch.Tensor([1.8, 0., 0.8])
    R = eul2rot(torch.Tensor([0., 0., 0.]))
    H = torch.eye(4)
    H[:3, :3] = R
    H[:3, -1] = b
    A = torch.eye(6)
    ## Leaf and Tree ##
    ee_goto_leaf = energies.TaskGoToLeaf(dim=6, b=b, A=A, R=H, var=torch.eye(6)*10.)
    ee_energy_tree = EnergyTree(branches=[ee_goto_leaf], map=pick_map)
    q_branches = [ee_energy_tree]
    task_energy_tree = EnergyTree(branches=q_branches, map=fk_map).to(device)

    # TODO: JointGoToLeaf
    ## Identity map ##
    identity_map = maps.SimplePosVel(dim=10)
    ## Leaf and Tree ##
    q_goto_leaf = energies.JointGoToLeaf_lefthand_and_base()
    q_energy_tree = EnergyTree(branches=[q_goto_leaf], map=identity_map).to(device)

    #########################
    energy_trees = [task_energy_tree, q_energy_tree]
    #energy_trees = task_energy_tree

    #ee_obj_avoid_leaf = energies.ObjAvoidLeaf()  # TODO: add branches here LATER!!!

    # TODO: Whole network
    policy = Multi_EBMControl(energy_tree=energy_trees, device=device, optimization_steps=10, dt=0.005, n_particles=10000)

    return policy