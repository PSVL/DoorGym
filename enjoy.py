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

def onpolicy_inference(
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

    env = make_vec_envs(
        env_name,
        seed + 1000,
        1,
        None,
        None,
        device='cuda:0',
        allow_early_resets=False,
        env_kwargs=env_kwargs,)

    env_obj = env.venv.venv.envs[0].env.env
    if env_name.find('door')<=-1:
        env_obj.unity = None

    render_func = get_render_func(env)
    if evaluation and not render:
        render_func = None

    if env_kwargs['visionnet_input']:
        visionmodel = VisionModelXYZ()
        visionmodel = load_visionmodel(load_name, args.visionmodel_path, VisionModelXYZ())  
    
    if not actor_critic:
            actor_critic, ob_rms = torch.load(load_name)
    actor_critic = actor_critic.eval()
    if env_kwargs['visionnet_input'] and env_name.find('doorenv')>-1:
        actor_critic.visionmodel = visionmodel
        actor_critic.visionnet_input = env_obj.visionnet_input
    actor_critic.to("cuda:0")

    if env_name.find('doorenv')>-1:
        actor_critic.nn = env_obj.nn

    recurrent_hidden_states = torch.zeros(1,actor_critic.recurrent_hidden_state_size)
    masks = torch.zeros(1, 1)

    full_obs = env.reset()
    initial_state = full_obs[:,:env.action_space.shape[0]]

    if env_name.find('doorenv')>-1 and env_obj.visionnet_input:
        obs = actor_critic.obs2inputs(full_obs, 0)
    else:
        if knob_noisy:
            obs = add_noise(full_obs)
        else:
            obs = full_obs

    if render_func is not None:
        render_func('human')

    # if env_name.find('doorenv')>-1:
    #     if env_obj.xml_path.find("baxter")>-1:
    #         doorhinge_idx = 20
    #     elif env_obj.xml_path.find("float")>-1:
    #         if env_obj.xml_path.find("hook")>-1:
    #             doorhinge_idx = 6
    #         elif env_obj.xml_path.find("gripper")>-1:
    #             doorhinge_idx = 11
    #     else:
    #         if env_obj.xml_path.find("mobile")>-1:
    #             if env_obj.xml_path.find("hook")>-1:
    #                 doorhinge_idx = 9
    #             if env_obj.xml_path.find("gripper")>-1:
    #                 doorhinge_idx = 14
    #         else:
    #             if env_obj.xml_path.find("hook")>-1:
    #                 doorhinge_idx = 7
    #             if env_obj.xml_path.find("gripper")>-1:
    #                 doorhinge_idx = 12

    start_time = int(time.mktime(time.localtime()))

    i=0
    epi_step = 0
    total_time = 0
    epi_counter = 1
    dooropen_counter = 0
    door_opened = False
    test_num = 100

    while True:
        with torch.no_grad():
            value, action, _, recurrent_hidden_states = actor_critic.act(
                obs, recurrent_hidden_states, masks, deterministic=det)
        next_action = action

        if pos_control:
            # print("enjoy step_skip",step_skip)
            if i%(512/step_skip-1)==0: current_state = initial_state
            next_action = current_state + next_action
            for kk in range(step_skip):
                full_obs, reward, done, infos = env.step(next_action)

            current_state = full_obs[:,:env.action_space.shape[0]]
        else:
            for kk in range(step_skip):
                full_obs, reward, done, infos = env.step(next_action)


        if env_name.find('doorenv')>-1 and env_obj.visionnet_input:
            obs = actor_critic.obs2inputs(full_obs, 0)
        else:
            if knob_noisy:
                obs = add_noise(full_obs)
            else:
                obs = full_obs

        masks.fill_(0.0 if done else 1.0)

        if render_func is not None:
            render_func('human')

        i+=1
        epi_step += 1

        if env_name.find('doorenv')>-1:
            # if not door_opened and abs(env_obj.sim.data.qpos[doorhinge_idx])>=0.2:
            if not door_opened and abs(env_obj.get_doorangle())>=0.2:
                dooropen_counter += 1
                opening_time = epi_step/(1.0/mujoco_timestep)*step_skip
                if verbose:
                    print("door opened! opening time is {}".format(opening_time))
                total_time += opening_time
                door_opened = True

        if env_name.find('Fetch')>-1:
            if not door_opened and infos[0]['is_success']==1:
                dooropen_counter += 1
                opening_time = epi_step/(1.0/mujoco_timestep)*step_skip
                if verbose:
                    print("Reached destenation! Time is {}".format(opening_time))
                total_time += opening_time
                door_opened = True
                
        if evaluation:
            if i%(512/step_skip-1)==0:
                if env_obj.unity:
                    env_obj.close()
                env = make_vec_envs(
                env_name,
                seed + 1000,
                1,
                None,
                None,
                device='cuda:0',
                allow_early_resets=False,
                env_kwargs=env_kwargs,)

                if render:
                    render_func = get_render_func(env)
                env_obj = env.venv.venv.envs[0].env.env
                if env_name.find('doorenv')<=-1:
                    env_obj.unity = None
                env.reset()
                if verbose:
                    print("{} ep end >>>>>>>>>>>>>>>>>>>>>>>>".format(epi_counter))
                    eval_print(dooropen_counter, epi_counter, start_time, total_time)
                epi_counter += 1
                epi_step = 0
                door_opened = False


        if i>=512/step_skip*test_num:
            if verbose:
                print( "dooropening counter:",dooropen_counter, " epi counter:", epi_counter)
                eval_print(dooropen_counter, epi_counter-1, start_time, total_time)
            break

    opening_rate, opening_timeavg = eval_print(dooropen_counter, epi_counter-1, start_time, total_time)
    return opening_rate, opening_timeavg

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
            if evaluation:
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
    # sys.path.append('a2c_ppo_acktr')

    parser = argparse.ArgumentParser(description='RL')
    parser.add_argument(
        '--seed', type=int, default=1, help='random seed (default: 1)')
    parser.add_argument(
        '--log-interval',
        type=int,
        default=10,
        help='log interval, one log per n updates (default: 10)')
    parser.add_argument(
        '--env-name',
        default='doorenv-v0',
        help='environment to train on (default: PongNoFrameskip-v4)')
    parser.add_argument(
        '--non-det',
        action='store_true',
        default=False,
        help='whether to use a non-deterministic policy')
    parser.add_argument(
        '--load-name',
        type=str,
        default='',
        help='which model to load')
    parser.add_argument(
        '--eval',
        action='store_true',
        default=False,
        help="Measure the opening ratio among 100 trials")
    parser.add_argument(
        '--render',
        action='store_true',
        default=False,
        help="force rendering")
    parser.add_argument(
        '--knob-noisy',
        action='store_true',
        default=False,
        help='add noise to knob position to resemble the noise from the visionnet')
    parser.add_argument(
        '--visionnet-input',
        action="store_true",
        default=False,
        help='Use vision net for knob position estimation')
    parser.add_argument(
        '--unity',
        action="store_true",
        default=False,
        help='Use unity for an input of a vision net')
    parser.add_argument(
        '--port',
        type=int,
        default=1050,
        help='Unity connection port (Only for off-policy)')
    parser.add_argument(
        '--visionmodel-path',
        type=str,
        default="./trained_visionmodel/",
        help='load the replay buffer')
    parser.add_argument(
        '--world-path',
        type=str,
        default="/u/home/urakamiy/pytorch-a2c-ppo-acktr-gail/random_world/world/pull_floatinghook",
        help='load the vision network model')
    parser.add_argument(
        '--pos-control',
        action="store_true",
        default=False,
        help='use pos control')
    parser.add_argument(
        '--step-skip',
        type=int,
        default=4,
        help='number of step skip in pos control')
    args = parser.parse_args()
    det = not args.non_det

    env_kwargs = dict(port = args.port,
                    visionnet_input = args.visionnet_input,
                    unity = args.unity,
                    world_path = args.world_path)

    if args.load_name.find("a2c")>-1 or args.load_name.find("ppo")>-1: 
        onpolicy_inference(
            seed=args.seed, 
            env_name=args.env_name, 
            det=det, 
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
            det=det, 
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