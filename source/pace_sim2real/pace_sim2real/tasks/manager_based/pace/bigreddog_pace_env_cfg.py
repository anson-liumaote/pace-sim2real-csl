# © 2025 ETH Zurich, Robotic Systems Lab
# Author: Filip Bjelonic
# Licensed under the Apache License 2.0

from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg
from pace_sim2real.utils import PaceDCMotorCfg
from pace_sim2real import PaceSim2realEnvCfg, PaceSim2realSceneCfg, PaceCfg
import torch

from pace_sim2real.assets.assets.robots.cslrobotics import BIGREDDOG_CFG  # isort: skip

BIGREDDOG_PACE_ACTUATOR_CFG = PaceDCMotorCfg(
    joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
    saturation_effort=80.0,
    effort_limit=80.0,
    velocity_limit=20.0,
    stiffness={".*": 40.0},  # P gain in Nm/rad
    damping={".*": 3.0},  # D gain in Nm s/rad
    encoder_bias=[0.0] * 12,  # encoder bias in radians
    max_delay=10,  # max delay in simulation steps
)


@configclass
class BigRedDogPaceCfg(PaceCfg):
    """Pace configuration for BigRedDog robot."""
    robot_name: str = "bigreddog"
    data_dir: str = "bigreddog/chirp_data.pt"  # located in pace_sim2real/data/bigreddog/chirp_data.pt
    bounds_params: torch.Tensor = torch.zeros((49, 2))  # 12 + 12 + 12 + 12 + 1 = 49 parameters to optimize
    joint_order: list[str] = [
        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_calf_joint",
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_calf_joint",
        "RL_hip_joint",
        "RL_thigh_joint",
        "RL_calf_joint",
        "RR_hip_joint",
        "RR_thigh_joint",
        "RR_calf_joint",
    ]

    def __post_init__(self):
        # set bounds for parameters
        self.bounds_params[:12, 0] = 1e-5
        self.bounds_params[:12, 1] = 0.1  # armature between 1e-5 - 0.1 [kgm2]
        self.bounds_params[12:24, 1] = 0.7  # dof_damping between 0.0 - 0.7 [Nm s/rad]
        self.bounds_params[24:36, 1] = 0.5  # friction between 0.0 - 0.5
        self.bounds_params[36:48, 0] = -0.1
        self.bounds_params[36:48, 1] = 0.1  # bias between -0.1 - 0.1 [rad]
        self.bounds_params[48, 1] = 10.0  # delay between 0.0 - 10.0 [sim steps]


@configclass
class BigreddogPaceSceneCfg(PaceSim2realSceneCfg):
    """Configuration for BigRedDog robot in Pace Sim2Real environment."""
    
    # robot: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot", init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 1.0)),
    #                                               actuators={"base_legs": GO2_PACE_ACTUATOR_CFG})
    robot: ArticulationCfg = BIGREDDOG_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.0),
            joint_pos={
                ".*L_hip_joint": 0.1,
                ".*R_hip_joint": -0.1,
                "FL_thigh_joint": 0.4,
                "R[L,R]_thigh_joint": -0.4,
                "F[L,R]_calf_joint": -1.0,
                "R[L,R]_calf_joint": 1.0,
            },
            joint_vel={".*": 0.0},
        ),
        actuators={"base_legs": BIGREDDOG_PACE_ACTUATOR_CFG},
    )


@configclass
class BigreddogPaceEnvCfg(PaceSim2realEnvCfg):
    scene: BigreddogPaceSceneCfg = BigreddogPaceSceneCfg()
    sim2real: BigRedDogPaceCfg = BigRedDogPaceCfg()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # robot sim and control settings
        self.sim.dt = 0.005  # 200Hz simulation
        self.decimation = 1  # 200Hz control