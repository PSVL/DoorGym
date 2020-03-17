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

from model.visionmodel import VisionModelXYZ
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

def onpolicy_main():
    print("onpolicy main")

    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    if args.cuda and torch.cuda.is_available() and args.cuda_deterministic:
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
    torch.set_num_threads(1)
    device = torch.device("cuda:0" if args.cuda else "cpu")

    summary_name = args.log_dir + '{0}_{1}'
    writer = SummaryWriter(summary_name.format(args.env_name, args.save_name))

    # Make vector env
    envs = make_vec_envs(args.env_name,
                         args.seed,
                         args.num_processes,
                         args.gamma, 
                         args.log_dir, 
                         device, 
                         False, 
                         env_kwargs=env_kwargs,)

    # agly ways to access to the environment attirubutes
    if args.env_name.find('doorenv')>-1:
        if args.num_processes>1:
            visionnet_input = envs.venv.venv.visionnet_input
            nn = envs.venv.venv.nn
            env_name = envs.venv.venv.xml_path
        else:
            visionnet_input = envs.venv.venv.envs[0].env.env.env.visionnet_input
            nn = envs.venv.venv.envs[0].env.env.env.nn
            env_name = envs.venv.venv.envs[0].env.env.env.xml_path
        dummy_obs = np.zeros(nn*2+3)
    else:
        dummy_obs = envs.observation_space
        visionnet_input = None
        nn = None

    if pretrained_policy_load:
        print("loading", pretrained_policy_load)
        actor_critic, ob_rms = torch.load(pretrained_policy_load)
    else:
        actor_critic = Policy(
            dummy_obs.shape,
            envs.action_space,
            base_kwargs={'recurrent': args.recurrent_policy})
    
    if visionnet_input: 
            visionmodel = load_visionmodel(save_name, args.visionmodel_path, VisionModelXYZ())  
            actor_critic.visionmodel = visionmodel.eval()
    actor_critic.nn = nn
    actor_critic.to(device)

    #disable normalizer
    vec_norm = get_vec_normalize(envs)
    vec_norm.eval()
    
    if args.algo == 'a2c':
        agent = algo.A2C_ACKTR(
            actor_critic,
            args.value_loss_coef,
            args.entropy_coef,
            lr=args.lr,
            eps=args.eps,
            alpha=args.alpha,
            max_grad_norm=args.max_grad_norm)
    elif args.algo == 'ppo':
        agent = algo.PPO(
            actor_critic,
            args.clip_param,
            args.ppo_epoch,
            args.num_mini_batch,
            args.value_loss_coef,
            args.entropy_coef,
            lr=args.lr,
            eps=args.eps,
            max_grad_norm=args.max_grad_norm)

    rollouts = RolloutStorage(args.num_steps, args.num_processes,
                              dummy_obs.shape, envs.action_space,
                              actor_critic.recurrent_hidden_state_size)

    full_obs = envs.reset()
    initial_state = full_obs[:,:envs.action_space.shape[0]]

    if args.env_name.find('doorenv')>-1 and visionnet_input:
        obs = actor_critic.obs2inputs(full_obs, 0)
    else:
        if knob_noisy:
            obs = add_vision_noise(full_obs, 0)
        elif obs_noisy:
            obs = add_joint_noise(full_obs)
        else:
            obs = full_obs

    rollouts.obs[0].copy_(obs)
    rollouts.to(device)

    episode_rewards = deque(maxlen=10)

    start = time.time()
    num_updates = int(
        args.num_env_steps) // args.num_steps // args.num_processes

    for j in range(num_updates):

        if args.use_linear_lr_decay:
            # decrease learning rate linearly
            utils.update_linear_schedule(
                agent.optimizer, j, num_updates, args.lr)

        total_switches = 0
        prev_selection = ""
        for step in range(args.num_steps):
            with torch.no_grad():
                value, action, action_log_prob, recurrent_hidden_states = actor_critic.act(
                    rollouts.obs[step], rollouts.recurrent_hidden_states[step],
                    rollouts.masks[step])
                next_action = action 

            if args.pos_control:
                # print("main step_skip",args.step_skip)
                if step%(512/args.step_skip-1)==0: current_state = initial_state
                next_action = current_state + next_action
                for kk in range(args.step_skip):
                    full_obs, reward, done, infos = envs.step(next_action)
                    
                current_state = full_obs[:,:envs.action_space.shape[0]]
            else:
                for kk in range(args.step_skip):
                    full_obs, reward, done, infos = envs.step(next_action)

            # convert img to obs if door_env and using visionnet 
            if args.env_name.find('doorenv')>-1 and visionnet_input:
                obs = actor_critic.obs2inputs(full_obs, j)
            else:
                if knob_noisy:
                    obs = add_vision_noise(full_obs, j)
                elif obs_noisy:
                    obs = add_joint_noise(full_obs)
                else:
                    obs = full_obs

            for info in infos:
                if 'episode' in info.keys():
                    episode_rewards.append(info['episode']['r'])

            masks = torch.FloatTensor(
                [[0.0] if done_ else [1.0] for done_ in done])
            bad_masks = torch.FloatTensor(
                [[0.0] if 'bad_transition' in info.keys() else [1.0]
                 for info in infos])
            rollouts.insert(obs, recurrent_hidden_states, action,
                            action_log_prob, value, reward, masks, bad_masks)
            
        with torch.no_grad():
            next_value = actor_critic.get_value(
                rollouts.obs[-1], rollouts.recurrent_hidden_states[-1],
                rollouts.masks[-1]).detach()

        rollouts.compute_returns(next_value, args.use_gae, args.gamma,
                                 args.gae_lambda, args.use_proper_time_limits)

        value_loss, action_loss, dist_entropy = agent.update(rollouts)
        rollouts.after_update()

        # Get total number of timesteps
        total_num_steps = (j + 1) * args.num_processes * args.num_steps

        writer.add_scalar("Value loss", value_loss, j)
        writer.add_scalar("action loss", action_loss, j)
        writer.add_scalar("dist entropy loss", dist_entropy, j)
        writer.add_scalar("Episode rewards", np.mean(episode_rewards), j)

        # save for every interval-th episode or for the last epoch
        if (j % args.save_interval == 0
                or j == num_updates - 1) and args.save_dir != "":
            save_path = os.path.join(args.save_dir, args.algo)
            try:
                os.makedirs(save_path)
            except OSError:
                pass
            torch.save([
                actor_critic,
                getattr(utils.get_vec_normalize(envs), 'ob_rms', None)
            ], os.path.join(save_path, args.env_name + "_{}.{}.pt".format(args.save_name,j)))

        if j % args.log_interval == 0 and len(episode_rewards) > 1:
            end = time.time()
            print(
                "Updates {}, num timesteps {}, FPS {} \n Last {} training episodes: mean/median reward {:.1f}/{:.1f}, min/max reward {:.1f}/{:.1f}\n"
                .format(j, total_num_steps,
                        int(total_num_steps / (end - start)),
                        len(episode_rewards), np.mean(episode_rewards),
                        np.median(episode_rewards), np.min(episode_rewards),
                        np.max(episode_rewards), dist_entropy, value_loss,
                        action_loss))

        if (args.eval_interval is not None and len(episode_rewards) > 1
                and j % args.eval_interval == 0):

            opening_rate, opening_timeavg = onpolicy_inference(
                                                seed=args.seed, 
                                                env_name=args.env_name, 
                                                det=True, 
                                                load_name=args.save_name, 
                                                evaluation=True, 
                                                render=False, 
                                                knob_noisy=args.knob_noisy, 
                                                visionnet_input=args.visionnet_input, 
                                                env_kwargs=env_kwargs_val,
                                                actor_critic=actor_critic,
                                                verbose=False,
                                                pos_control=args.pos_control,
                                                step_skip=args.step_skip)

            print("{}th update. {}th timestep. opening rate {}%. Average time to open is {}.".format(j, total_num_steps, opening_rate, opening_timeavg))
            writer.add_scalar("Opening rate per envstep", opening_rate, total_num_steps)
            writer.add_scalar("Opening rate per update", opening_rate, j)

        DR=True #Domain Randomization
        ################## for multiprocess world change ######################
        if DR:
            print("changing world")

            envs.close_extras()
            envs.close()
            del envs

            envs = make_vec_envs(args.env_name,
                        args.seed,
                        args.num_processes,
                        args.gamma, 
                        args.log_dir, 
                        device, 
                        False, 
                        env_kwargs=env_kwargs,)

            full_obs = envs.reset()
            if args.env_name.find('doorenv')>-1 and visionnet_input:
                obs = actor_critic.obs2inputs(full_obs, j)
            else:
                obs = full_obs
        #######################################################################


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

    eval_path_collector = MdpPathCollector(
        eval_env,
        eval_policy,
        doorenv=args.env_name.find('doorenv')>-1,
    )
    expl_path_collector = MdpPathCollector(
        expl_env,
        expl_policy,
        doorenv=args.env_name.find('doorenv')>-1,
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
    summary_name = args.log_dir + '{0}_{1}'
    writer = SummaryWriter(summary_name.format(args.env_name, args.save_name))
    algorithm.writer = writer

    algorithm.to(ptu.device)
    algorithm.train()

if __name__ == "__main__":
    args = get_args()

    knob_noisy = args.knob_noisy
    obs_noisy = args.obs_noisy
    pretrained_policy_load = args.pretrained_policy_load
    env_kwargs = dict(port = args.port,
                    visionnet_input = args.visionnet_input,
                    unity = args.unity,
                    world_path = args.world_path,
                    pos_control = args.pos_control)

    env_kwargs_val = env_kwargs.copy()
    env_kwargs_val['world_path'] = args.val_path

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
        offpolicy_main(variant)
    elif args.algo == 'td3':
        variant = dict(
            algorithm=args.algo,
            algorithm_kwargs=dict(
                num_epochs=3000,
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
        offpolicy_main(variant)
    elif args.algo == 'a2c' or args.algo == 'ppo':
        onpolicy_main()
    else:
        raise Exception("unknown algorithm")