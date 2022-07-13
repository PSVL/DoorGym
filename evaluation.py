import argparse
import os
import sys
import numpy as np
import torch
import time

from util import load_visionmodel, prepare_env
from a2c_ppo_acktr.envs import VecPyTorch, make_vec_envs
from a2c_ppo_acktr.utils import get_render_func, get_vec_normalize
from trained_visionmodel.visionmodel import VisionModelXYZ, VisionModel

from rlkit.samplers.rollout_functions import rollout
from rlkit.torch.pytorch_util import set_gpu_mode
from rlkit.core import logger
from rlkit.envs.wrappers import NormalizedBoxEnv
import uuid

from curl_navi import DoorGym_utils 

import doorenv
import doorenv2

mujoco_timestep = 0.02
def eval_print(dooropen_counter, counter, start_time, total_time):
    opening_rate = dooropen_counter/counter *100
    if dooropen_counter != 0:
        opening_timeavg = total_time/dooropen_counter
    else:
        opening_timeavg = -1
    print("opening rate {}%. Average time to open is {}.".format(opening_rate, opening_timeavg))
    print("took {}sec to evaluate".format( int(time.mktime(time.localtime())) - start_time ))

    return opening_rate, opening_timeavg

def add_noise(obs, epoch=100):
    satulation = 100.
    sdv = torch.tensor([3.440133806003181, 3.192113342496682,  1.727412865751099]) /satulation  #Vision SDV for arm
    noise = torch.distributions.Normal(torch.tensor([0.0, 0.0, 0.0]), sdv).sample().cuda()
    noise *= min(1., epoch/satulation)
    obs[:,-3:] += noise
    return obs

def offpolicy_inference(
                seed, 
                env_name, 
                det, 
                load_name, 
                evaluation, 
                render, 
                knob_noisy, 
                visionnet_input, 
                env_kwargs,
                actor_critic=None,
                verbose=True,
                pos_control=True,
                step_skip=4):

    import time 
    from gym import wrappers

    print("evaluatin started!")

    filename = str(uuid.uuid4())

    gpu = True

    env, _, _ = prepare_env(env_name, **env_kwargs)
    
    if not actor_critic:
        snapshot = torch.load(load_name)
        policy = snapshot['evaluation/policy']
    else:
        policy = actor_critic
    if env_name.find('doorenv')>-1:
        policy.knob_noisy = knob_noisy
        policy.nn = env._wrapped_env.nn
        policy.visionnet_input = env_kwargs['visionnet_input']

    epi_counter = 1
    dooropen_counter = 0
    total_time = 0
    test_num = 100

    start_time = int(time.mktime(time.localtime()))

    if gpu:
        set_gpu_mode(True)
    while True:
        # print("new env")
        if env_name.find('doorenv')>-1:
            if evaluation:
                path, door_opened, opening_time = rollout(
                    env,
                    policy,
                    max_path_length=512,
                    render=render,
                    evaluate=evaluation,
                    verbose=True,
                    doorenv=True,
                    pos_control=pos_control,
                    step_skip=step_skip,
                )
                if hasattr(env, "log_diagnostics"):
                    env.log_diagnostics([path])
                logger.dump_tabular()
                # if evaluation:
                    # print("1")
                env, _, _ = prepare_env(env_name, **env_kwargs)
                if door_opened:
                    dooropen_counter += 1
                    total_time += opening_time
                    if verbose:
                        print("{} ep end >>>>>>>>>>>>>>>>>>>>>>>>".format(epi_counter))
                        eval_print(dooropen_counter, epi_counter, start_time, total_time)
            else:
                path = rollout(
                    env,
                    policy,
                    max_path_length=512,
                    render=render,
                    evaluate=evaluation,
                    verbose=True,
                    doorenv=True,
                    pos_control=pos_control,
                    step_skip=step_skip,
                )
                if hasattr(env, "log_diagnostics"):
                    env.log_diagnostics([path])
                logger.dump_tabular()

        else:
            path = rollout(
                env,
                policy,
                max_path_length=512,
                doorenv=False,
                render=render,
            )
            if hasattr(env, "log_diagnostics"):
                env.log_diagnostics([path])
            logger.dump_tabular()

        if evaluation:
            if verbose:
                print("{} ep end >>>>>>>>>>>>>>>>>>>>>>>>".format(epi_counter))
                eval_print(dooropen_counter, epi_counter, start_time, total_time)
            epi_counter += 1

            if env_name.find('door')>-1 and epi_counter>test_num:
                if verbose:
                    print( "dooropening counter:",dooropen_counter, " epi counter:", epi_counter)
                    eval_print(dooropen_counter, epi_counter, start_time, total_time)
                break

    opening_rate, opening_timeavg = eval_print(dooropen_counter, epi_counter-1, start_time, total_time)
    return opening_rate, opening_timeavg

if __name__ == "__main__":

    args = DoorGym_utils.get_inference_args()

    env_kwargs = dict(port = args.port,
                    visionnet_input = args.visionnet_input,
                    unity = args.unity,
                    world_path = args.world_path)

    if args.load_name.find("ppo")>-1 or args.load_name.find("husky_ur5")>-1: 
        DoorGym_utils.onpolicy_inference(
            seed=args.seed, 
            env_name=args.env_name, 
            det=args.det, 
            load_name=args.load_name, 
            evaluation=args.eval, 
            render=args.render, 
            knob_noisy=args.knob_noisy, 
            visionnet_input=args.visionnet_input, 
            env_kwargs=env_kwargs,
            pos_control=args.pos_control,
            step_skip=args.step_skip)
    elif args.load_name.find("sac")>-1 or args.load_name.find("td3")>-1:
        offpolicy_inference(
            seed=args.seed, 
            env_name=args.env_name, 
            det=args.det, 
            load_name=args.load_name, 
            evaluation=args.eval, 
            render=args.render, 
            knob_noisy=args.knob_noisy, 
            visionnet_input=args.visionnet_input, 
            env_kwargs=env_kwargs,
            actor_critic=None,
            verbose=True,
            pos_control=args.pos_control,
            step_skip=args.step_skip)
    else:
        raise "not sure which type of algorithm."