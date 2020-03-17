import torch
import os
import numpy as np
import gym
from gym import wrappers

from rlkit.envs.wrappers import NormalizedBoxEnv
from rlkit.torch.networks import FlattenMlp, TanhMlpPolicy
from rlkit.torch.sac.policies import TanhGaussianPolicy, MakeDeterministic
from rlkit.exploration_strategies.base import PolicyWrappedWithExplorationStrategy
from rlkit.exploration_strategies.gaussian_strategy import GaussianStrategy
from rlkit.torch.sac.sac import SACTrainer
from rlkit.torch.td3.td3 import TD3Trainer


def add_vision_noise(obs, epoch):
        satulation = 100.
        sdv = torch.tensor([3.440133806003181, 3.192113342496682, 1.727412865751099]) /100.  #Vision SDV for arm
        noise = torch.distributions.Normal(torch.tensor([0.0, 0.0, 0.0]), sdv).sample().cuda()
        noise *= min(1., epoch/satulation)
        obs[:,-3:] += noise
        return obs

def add_joint_noise(obs):
        sdv = torch.ones(obs.size(1))*0.03
        # print("joint_sdv", sdv.size()
        noise = torch.distributions.Normal(torch.zeros(sdv.size()), sdv).sample().cuda()
        # print("noise", noise.size(), noise)
        obs[:] += noise
        # print("obs", obs)
        return obs

def load_visionmodel(xml_path, model_path, visionmodel):
    visionmodel_path = model_path + "floatinghook_pull/checkpoint.pth.tar"
    if not os.path.isfile(visionmodel_path):
        raise RuntimeError("=> no checkpoint found at '{}'" .format(visionmodel_path))
    checkpoint = torch.load(visionmodel_path)
    visionmodel.load_state_dict(checkpoint['state_dict'])
    best_pred = checkpoint['best_pred']
    print("=> loaded checkpoint '{}' (epoch {}), best pred {}".format(visionmodel_path, checkpoint['epoch'], best_pred))
    return visionmodel

def prepare_env(env_name, visionmodel_path, **env_kwargs):
    from gym.spaces import Box

    if env_name.find('doorenv')>-1:
        expl_env = NormalizedBoxEnv(gym.make(env_name, **env_kwargs))
        xml_path = expl_env._wrapped_env.xml_path
        if env_kwargs['visionnet_input']:
            print("using vision")
            eval_env = None
            if env_kwargs['unity']:
                expl_env._wrapped_env.init()
        else:
            print("no vision")
            eval_env = NormalizedBoxEnv(gym.make(env_name, **env_kwargs))

        env_obj = expl_env._wrapped_env
        expl_env.observation_space = Box(np.zeros(env_obj.nn*2+3), np.zeros(env_obj.nn*2+3), dtype=np.float32)
        if eval_env:
            eval_env.observation_space = Box(np.zeros(env_obj.nn*2+3), np.zeros(env_obj.nn*2+3), dtype=np.float32)
    elif env_name.find("Fetch")>-1:
        env = gym.make(env_name, reward_type='sparse')
        env = wrappers.FlattenDictWrapper(env, dict_keys=['observation', 'desired_goal'])
        expl_env = NormalizedBoxEnv(env)
        eval_env = NormalizedBoxEnv(env)
        env_obj = None
    else:
        expl_env = NormalizedBoxEnv(gym.make(env_name))
        eval_env = NormalizedBoxEnv(gym.make(env_name))
        env_obj = None

    return expl_env, eval_env, env_obj


def prepare_trainer(algorithm, expl_env, obs_dim, action_dim, pretrained_policy_load, variant):
    print("Preparing for {} trainer.".format(algorithm))
    if algorithm == "SAC":
        if not pretrained_policy_load:
            M = variant['layer_size']
            qf1 = FlattenMlp(
                input_size=obs_dim + action_dim,
                output_size=1,
                hidden_sizes=[M, M],
            )
            qf2 = FlattenMlp(
                input_size=obs_dim + action_dim,
                output_size=1,
                hidden_sizes=[M, M],
            )
            target_qf1 = FlattenMlp(
                input_size=obs_dim + action_dim,
                output_size=1,
                hidden_sizes=[M, M],
            )
            target_qf2 = FlattenMlp(
                input_size=obs_dim + action_dim,
                output_size=1,
                hidden_sizes=[M, M],
            )
            policy = TanhGaussianPolicy(
                obs_dim=obs_dim,
                action_dim=action_dim,
                hidden_sizes=[M, M],
            )
        else:
            snapshot = torch.load(pretrained_policy_load)
            qf1 = snapshot['trainer/qf1']
            qf2 = snapshot['trainer/qf2']
            target_qf1 = snapshot['trainer/target_qf1']
            target_qf2 = snapshot['trainer/target_qf2']
            policy = snapshot['exploration/policy']
            if variant['trainer_kwargs']['use_automatic_entropy_tuning']:
                log_alpha = snapshot['trainer/log_alpha'] 
                variant['trainer_kwargs']['log_alpha'] = log_alpha
                alpha_optimizer = snapshot['trainer/alpha_optimizer'] 
                variant['trainer_kwargs']['alpha_optimizer'] = alpha_optimizer
            print("loaded the pretrained policy {}".format(pretrained_policy_load))
        
        eval_policy = MakeDeterministic(policy)
        expl_policy = policy

        trainer = SACTrainer(
            env=expl_env,
            policy=policy,
            qf1=qf1,
            qf2=qf2,
            target_qf1=target_qf1,
            target_qf2=target_qf2,
            **variant['trainer_kwargs']
        )

    elif algorithm == "TD3":
        if not pretrained_policy_load:
            qf1 = FlattenMlp(
                input_size=obs_dim + action_dim,
                output_size=1,
                **variant['qf_kwargs']
            )
            qf2 = FlattenMlp(
                input_size=obs_dim + action_dim,
                output_size=1,
                **variant['qf_kwargs']
            )
            target_qf1 = FlattenMlp(
                input_size=obs_dim + action_dim,
                output_size=1,
                **variant['qf_kwargs']
            )
            target_qf2 = FlattenMlp(
                input_size=obs_dim + action_dim,
                output_size=1,
                **variant['qf_kwargs']
            )
            policy = TanhMlpPolicy(
                input_size=obs_dim,
                output_size=action_dim,
                **variant['policy_kwargs']
            )
            target_policy = TanhMlpPolicy(
                input_size=obs_dim,
                output_size=action_dim,
                **variant['policy_kwargs']
            )
            es = GaussianStrategy(
                action_space=expl_env.action_space,
                max_sigma=0.1,
                min_sigma=0.1,  # Constant sigma
            )
            exploration_policy = PolicyWrappedWithExplorationStrategy(
                exploration_strategy=es,
                policy=policy,
            )
            expl_policy = exploration_policy
            eval_policy = policy
        else:
            pass

        trainer = TD3Trainer(
            policy=policy,
            qf1=qf1,
            qf2=qf2,
            target_qf1=target_qf1,
            target_qf2=target_qf2,
            target_policy=target_policy,
            **variant['trainer_kwargs']
        )

    return expl_policy, eval_policy, trainer