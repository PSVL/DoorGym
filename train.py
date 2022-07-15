import os
import sys
import time
import pickle
from collections import deque
import gym
import numpy as np

import torch
import torch.nn as nn
from tensorboardX import SummaryWriter
from curl_navi import DoorGym_utils

from trained_visionmodel.visionmodel import VisionModelXYZ
from enjoy import onpolicy_inference, offpolicy_inference
from util import add_vision_noise, add_joint_noise,load_visionmodel, prepare_trainer, prepare_env

from a2c_ppo_acktr import algo, utils
from a2c_ppo_acktr.arguments import get_args
from a2c_ppo_acktr.envs import make_vec_envs
from a2c_ppo_acktr.model import Policy
from a2c_ppo_acktr.storage import RolloutStorage
from a2c_ppo_acktr.utils import get_vec_normalize

import rlkit.torch.pytorch_util as ptu
from rlkit.data_management.env_replay_buffer import EnvReplayBuffer
from rlkit.launchers.launcher_util import setup_logger
from rlkit.samplers.data_collector import MdpPathCollector
from rlkit.torch.torch_rl_algorithm import TorchBatchRLAlgorithm

import doorenv
import doorenv2

def offpolicy_main(variant):
    print("offpolicy main")  

    if args.algo == 'sac':
        algo = "SAC"
    elif args.algo == 'td3':
        algo = "TD3"
    
    setup_logger('{0}_{1}'.format(args.env_name, args.save_name), variant=variant)
    ptu.set_gpu_mode(True)  # optionally set the GPU (default=True)

    expl_env, eval_env, env_obj = prepare_env(args.env_name, args.visionmodel_path, **env_kwargs)
    obs_dim = expl_env.observation_space.low.size
    action_dim = expl_env.action_space.low.size    
    expl_policy, eval_policy, trainer = prepare_trainer(algo, expl_env, obs_dim, action_dim, args.pretrained_policy_load, variant)

    if args.env_name.find('doorenv')>-1:
        expl_policy.knob_noisy = eval_policy.knob_noisy = args.knob_noisy
        expl_policy.nn = eval_policy.nn = env_obj.nn
        expl_policy.visionnet_input = eval_policy.visionnet_input = env_obj.visionnet_input

    if args.visionnet_input:
        visionmodel = load_visionmodel(expl_env._wrapped_env.xml_path, args.visionmodel_path, VisionModelXYZ())
        visionmodel.to(ptu.device)
        expl_policy.visionmodel = visionmodel.eval()
    else:
        expl_policy.visionmodel = None

    # print("intput stepskip:", args.step_skip)
    eval_path_collector = MdpPathCollector(
        eval_env,
        eval_policy,
        doorenv=args.env_name.find('doorenv')>-1,
        pos_control=args.pos_control,
        step_skip=args.step_skip,
    )
    expl_path_collector = MdpPathCollector(
        expl_env,
        expl_policy,
        doorenv=args.env_name.find('doorenv')>-1,
        pos_control=args.pos_control,
        step_skip=args.step_skip,
    )

    if not args.replaybuffer_load:
        replay_buffer = EnvReplayBuffer(
            variant['replay_buffer_size'],
            expl_env,
        )
    else:
        replay_buffer = pickle.load(open(args.replaybuffer_load,"rb"))
        replay_buffer._env_info_keys = replay_buffer.env_info_sizes.keys()
        print("Loaded the replay buffer that has length of {}".format(replay_buffer.get_diagnostics()))

    algorithm = TorchBatchRLAlgorithm(
        trainer=trainer,
        exploration_env=expl_env,
        evaluation_env=eval_env,
        exploration_data_collector=expl_path_collector,
        evaluation_data_collector=eval_path_collector,
        replay_buffer=replay_buffer,
        **variant['algorithm_kwargs']
    )

    algorithm.save_interval = args.save_interval
    algorithm.save_dir = args.save_dir
    algorithm.algo = args.algo
    algorithm.env_name = args.env_name
    algorithm.save_name = args.save_name
    algorithm.env_kwargs = env_kwargs
    algorithm.env_kwargs_val = env_kwargs_val
    algorithm.eval_function = offpolicy_inference
    algorithm.eval_interval = args.eval_interval
    algorithm.knob_noisy = knob_noisy
    algorithm.visionnet_input = args.visionnet_input
    algorithm.pos_control = args.pos_control
    algorithm.step_skip = args.step_skip
    algorithm.max_path_length = variant['algorithm_kwargs']['max_path_length']

    summary_name = args.log_dir + '{0}_{1}'
    writer = SummaryWriter(summary_name.format(args.env_name, args.save_name))
    algorithm.writer = writer

    algorithm.to(ptu.device)
    algorithm.train()


def parse(args):
    import datetime
    opt = args
    args = vars(opt)
    verbose = True
    if verbose:
        print('------------ Options -------------')
        print("start time:", datetime.datetime.now())
        for k, v in sorted(args.items()):
            print('%s: %s' % (str(k), str(v)))
        print('-------------- End ----------------')
    # save to the disk
    expr_dir = os.path.join(opt.params_log_dir)
    file_name = os.path.join(expr_dir, '{}.txt'.format(opt.env_name))
    with open(file_name, 'wt') as opt_file:
        opt_file.write('------------ Options -------------\n')
        opt_file.write('start time:' + str(datetime.datetime.now()))
        for k, v in sorted(args.items()):
            opt_file.write('%s: %s\n' % (str(k), str(v)))
        opt_file.write('-------------- End ----------------\n')

if __name__ == "__main__":
    # args = get_args()
    args = DoorGym_utils.get_training_args()

    knob_noisy = args.knob_noisy
    obs_noisy = args.obs_noisy
    pretrained_policy_load = args.pretrained_policy_load
    env_kwargs = dict(port = args.port,
                    visionnet_input = args.visionnet_input,
                    unity = args.unity,
                    world_path = args.world_path,
                    pos_control = args.pos_control)

    env_kwargs_val = env_kwargs.copy()
    if args.val_path: env_kwargs_val['world_path'] = args.val_path

    if args.algo == 'sac':
        variant = dict(
            algorithm=args.algo,
            version="normal",
            layer_size=100,
            algorithm_kwargs=dict(
                num_epochs=6000,
                num_eval_steps_per_epoch=512, #512
                num_trains_per_train_loop=1000, #1000
                num_expl_steps_per_train_loop=512, #512
                min_num_steps_before_training=512, #1000
                max_path_length=512, #512
                batch_size=128,
                ),
            trainer_kwargs=dict(
                discount=0.99,
                soft_target_tau=5e-3,
                target_update_period=1,
                policy_lr=1E-3,
                qf_lr=1E-3,
                reward_scale=0.1,
                use_automatic_entropy_tuning=True,
                ),
            replay_buffer_size=int(1E6),
        )
        # args_variant = {**vars(args), **variant}
        # parse(args_variant)
        offpolicy_main(variant)
    elif args.algo == 'td3':
        variant = dict(
            algorithm=args.algo,
            algorithm_kwargs=dict(
                num_epochs=1500,
                num_eval_steps_per_epoch=512,
                num_trains_per_train_loop=1000,
                num_expl_steps_per_train_loop=512,
                min_num_steps_before_training=512,
                max_path_length=512,
                batch_size=128,
            ),
            trainer_kwargs=dict(
                discount=0.99,
                policy_learning_rate=1e-3,
                qf_learning_rate=1e-3,
                policy_and_target_update_period=2,
                tau=0.005,
            ),
            qf_kwargs=dict(
                hidden_sizes=[100, 100],
            ),
            policy_kwargs=dict(
                hidden_sizes=[100, 100],
            ),
            replay_buffer_size=int(1E6),
        )
        # args_variant = {**args, **variant}
        # parse(args_variant)
        offpolicy_main(variant)
    elif args.algo == 'a2c' or args.algo == 'ppo':
        # parse(args_variant)
        DoorGym_utils.onpolicy_main(args)
    else:
        raise Exception("unknown algorithm")