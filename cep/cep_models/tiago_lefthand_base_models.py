import torch
import numpy as np
from cep.utils import eul2rot

from cep.kinematics import TiagoRobot_lefthand_Base
from cep.controllers import Multi_EBMControl, EBMControl, EnergyTree#, EBMControl_x, EnergyTree_x


from cep import maps
from cep import energies

# GPU_ON = torch.cuda.is_available()
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
    b = torch.Tensor([1.5, 1.2, 0.8])
    R = eul2rot(torch.Tensor([0., 0., 0.]))

    H = torch.eye(4)
    H[:3, :3] = R
    H[:3, -1] = b

    A = torch.eye(6)
    ee_goto_leaf = energies.TaskGoToLeaf(dim=6, b=b, A=A, R=H, var=torch.eye(6)*10.)
    #ee_obj_avoid = energies.ObjAvoidLeaf()  # TODO: add branches here???
    pick_map = maps.SelectionMap(idx=9)  # TODO 08.12 WHY????
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

def jsc_and_goto_cep_simple_model_lefthandBase():  # TODO: 06.28, 06.29, 08.13

    # TODO: TaskGoToLeaf
    ##Get all the FK maps##
    tiago_kin = TiagoRobot_lefthand_Base()
    fk_map = maps.FK_ALL_lefthand_base(tiago_kin)
    pick_map = maps.SelectionMap(idx=9)
    ## End Effector Branch ##
    b = torch.Tensor([1.5, 1.2, 0.8])
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

    ## TODO: JointGoToLeaf
    ## Identity map ##
    identity_map = maps.SimplePosVel(dim=10)
    ## Leaf and Tree ##
    q_goto_leaf = energies.JointGoToLeaf_lefthand_and_base()
    q_energy_tree = EnergyTree(branches=[q_goto_leaf], map=identity_map).to(device)

    #########################
    energy_trees = [task_energy_tree, q_energy_tree]

    #ee_obj_avoid_leaf = energies.ObjAvoidLeaf()  # TODO: add branches here LATER!!!

    # TODO: Whole network
    policy = Multi_EBMControl(energy_tree=energy_trees, device=device, optimization_steps=20, dt=0.005, n_particles=1000)

    return policy

def cep_model_lefthandBase_taskgotoAndPathplan():  ## TODO: Added on 08.12, 08.17, fixed on 08.27, but need to improve the temperatures

    # TODO: TaskGoToLeaf
    ##Get all the FK maps##
    tiago_kin = TiagoRobot_lefthand_Base()
    fk_map = maps.FK_ALL_lefthand_base(tiago_kin)
    pick_map = maps.SelectionMap(idx=8)
    ## End Effector Branch ##
    b = torch.Tensor([1.7, 1.1, 0.8])
    R = eul2rot(torch.Tensor([0., 0., 0.]))
    H = torch.eye(4)
    H[:3, :3] = R
    H[:3, -1] = b
    A = torch.eye(6)
    ## Leaf and Tree ##
    ee_goto_leaf = energies.TaskGoToLeaf(dim=6, b=b, A=A, R=H, var=torch.eye(6) * 0.1)
    ee_energy_tree = EnergyTree(branches=[ee_goto_leaf], map=pick_map)
    q_branches = [ee_energy_tree]
    task_energy_tree = EnergyTree(branches=q_branches, map=fk_map).to(device)

    # TODO: PathPlanLeaf
    ## Identity map ##
    # base_map = maps.PathplanMap(idx=2)
    # ## Leaf and Tree ##
    # base_goto_leaf = energies.PathPlanLeaf_lefthand_and_base()
    # base_energy_tree = EnergyTree(branches=[base_goto_leaf], map=base_map).to(device)

    base_map = maps.PathplanMap(idx=2)  # Map R^10 to R^2
    identity_map = maps.SimpleBase(dim=2)  # Keep x and y as the same
    ## Leaf and Tree ##
    base_goto_leaf = energies.PathPlanLeaf_lefthand_and_base(var=torch.eye(2).float() * .1)
    base_energy_tree = EnergyTree(branches=[base_goto_leaf], map=identity_map).to(device)

    #########################
    q_branches = [base_energy_tree]
    base_energy_tree = EnergyTree(branches=q_branches, map=base_map).to(device)

    #########################
    energy_trees = [task_energy_tree, base_energy_tree]

    #ee_obj_avoid_leaf = energies.ObjAvoidLeaf()  # TODO: add branches here LATER!!!

    # TODO: Whole network
    policy = Multi_EBMControl(energy_tree=energy_trees, device=device, optimization_steps=20, dt=0.02, n_particles=10000)

    return policy

def cep_tiago_lefthand_base_pathplan():  # TODO: 08.15, 08.17, 08.18 | 12.06

    # TODO: PathPlanLeaf
    base_map = maps.PathplanMap(idx=2)  # Map R^10 to R^2
    identity_map = maps.SimpleBase(dim=2)  # Keep x and y as the same
    ## Leaf and Tree ##
    base_goto_leaf = energies.PathPlanLeaf_lefthand_and_base()
    base_energy_tree = EnergyTree(branches=[base_goto_leaf], map=identity_map).to(device)

    #########################
    q_branches = [base_energy_tree]
    energy_tree = EnergyTree(branches=q_branches, map=base_map).to(device)
    policy = EBMControl(energy_tree=energy_tree, device=device, optimization_steps=5, dt=0.005, n_particles=1000)
    return policy

def cep_tiago_lefthand_base_pathplan_np():  # TODO: 08.15, 08.17, 08.18, 09.08

    # TODO: PathPlanLeaf
    base_map = maps.PathplanMap(idx=2)  # Map R^10 to R^2
    identity_map = maps.SimpleBase(dim=2)  # Keep x and y as the same
    ## Leaf and Tree ##
    base_goto_leaf = energies.PathPlanLeaf_lefthand_and_base_np()
    base_energy_tree = EnergyTree(branches=[base_goto_leaf], map=identity_map).to(device)

    #########################
    q_branches = [base_energy_tree]
    energy_tree = EnergyTree(branches=q_branches, map=base_map).to(device)
    policy = EBMControl(energy_tree=energy_tree, device=device, optimization_steps=5, dt=0.005, n_particles=1000)
    return policy

def cep_tiago_base_pathplan_x():  # TODO: 09.13

    # TODO: PathPlanLeaf
    base_map = maps.PathplanMap(idx=2)  # Map R^10 to R^2
    identity_map = maps.SimpleBase(dim=2)  # Keep x and y as the same
    ## Leaf and Tree ##
    base_goto_leaf = energies.PathPlanLeaf_pos()
    base_energy_tree = EnergyTree_x(branches=[base_goto_leaf], map=identity_map).to(device)

    #########################
    q_branches = [base_energy_tree]
    energy_tree = EnergyTree_x(branches=q_branches, map=base_map).to(device)
    policy = EBMControl_x(energy_tree=energy_tree, device=device, optimization_steps=5, dt=0.005, n_particles=1000)
    return policy

def cep_tiago_base_pathplan_n_x():  # TODO: 09.16

    # TODO: PathPlanLeaf
    base_map = maps.PathplanMap(idx=2)  # Map R^10 to R^2
    identity_map = maps.SimpleBase(dim=2)  # Keep x and y as the same
    ## Leaf and Tree ##
    base_goto_leaf = energies.PathPlanLeaf_n_pos()
    base_energy_tree = EnergyTree_x(branches=[base_goto_leaf], map=identity_map).to(device)

    #########################
    q_branches = [base_energy_tree]
    energy_tree = EnergyTree_x(branches=q_branches, map=base_map).to(device)
    policy = EBMControl_x(energy_tree=energy_tree, device=device, optimization_steps=5, dt=0.005, n_particles=1000)
    return policy

def cep_tiago_pathplan_trackfather():  # TODO: 09.23

    # TODO: PathPlanLeaf
    base_map = maps.PathplanMap(idx=2)  # Map R^10 to R^2
    identity_map = maps.SimpleBase(dim=2)  # Keep x and y as the same
    ## Leaf and Tree ##
    base_goto_leaf = energies.PathPlanLeaf_trackfather()
    base_energy_tree = EnergyTree(branches=[base_goto_leaf], map=identity_map).to(device)

    #########################
    q_branches = [base_energy_tree]
    energy_tree = EnergyTree(branches=q_branches, map=base_map).to(device)
    policy = EBMControl(energy_tree=energy_tree, device=device, optimization_steps=5, dt=0.005, n_particles=1000)
    return policy

def cep_tiago_pathplan_track_Nfather(K):  # TODO: 09.23

    # TODO: PathPlanLeaf
    base_map = maps.PathplanMap(idx=2)  # Map R^10 to R^2
    identity_map = maps.SimpleBase(dim=2)  # Keep x and y as the same
    ## Leaf and Tree ##
    base_goto_leaf = energies.PathPlanLeaf_track_Nfather(K)
    base_energy_tree = EnergyTree(branches=[base_goto_leaf], map=identity_map).to(device)

    #########################
    q_branches = [base_energy_tree]
    energy_tree = EnergyTree(branches=q_branches, map=base_map).to(device)
    policy = EBMControl(energy_tree=energy_tree, device=device, optimization_steps=10, dt=0.005, n_particles=5000)
    return policy