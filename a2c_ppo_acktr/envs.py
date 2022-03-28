import os
import sys

import gym
import numpy as np
import torch
from gym.spaces.box import Box

from baselines import bench
from baselines.common.atari_wrappers import make_atari, wrap_deepmind
from baselines.common.vec_env.vec_env import \
    VecEnvWrapper, VecEnv, CloudpickleWrapper, clear_mpi_env_vars
from baselines.common.vec_env.dummy_vec_env import DummyVecEnv
from baselines.common.vec_env.shmem_vec_env import ShmemVecEnv, _subproc_worker
from baselines.common.vec_env.vec_normalize import \
    VecNormalize as VecNormalize_
from baselines.common.vec_env.util import dict_to_obs, obs_space_info, obs_to_dict

import multiprocessing as mp
import ctypes
from baselines import logger

_NP_TO_CT = {np.float64: ctypes.c_double,
             np.float32: ctypes.c_float,
             np.int32: ctypes.c_int32,
             np.int8: ctypes.c_int8,
             np.uint8: ctypes.c_char,
             np.bool: ctypes.c_bool}

try:
    import dm_control2gym
except ImportError:
    pass

try:
    import roboschool
except ImportError:
    pass

try:
    import pybullet_envs
except ImportError:
    pass

# Derived from
# https://github.com/openai/baselines/blob/master/baselines/common/vec_env/shmem_vec_env.py
class ShmemVecEnv_DR(ShmemVecEnv):
    def __init__(self, env_fns, spaces=None, context='spawn'):
        """
        If you don't specify observation_space, we'll have to create a dummy
        environment to get it.
        """

        ctx = mp.get_context(context)
        if spaces:
            observation_space, action_space = spaces
        else:
            logger.log('Creating dummy env object to get spaces')
            with logger.scoped_configure(format_strs=[]):
                dummy = env_fns[0]()
                observation_space, action_space = dummy.observation_space, dummy.action_space
                #dummy.close()
                try:
                    self.visionnet_input = dummy.env.env.env.visionnet_input
                    self.nn = dummy.env.env.env.nn
                    self.xml_path = dummy.env.env.env.xml_path
                    if dummy.env.env.env.unity:
                        dummy.env.env.env.close() ## HACK>>>
                except Exception as e:
                    print(e)
                    pass
                del dummy

        VecEnv.__init__(self, len(env_fns), observation_space, action_space)
        self.obs_keys, self.obs_shapes, self.obs_dtypes = obs_space_info(observation_space)
        self.obs_bufs = [
            {k: ctx.Array(_NP_TO_CT[self.obs_dtypes[k].type], int(np.prod(self.obs_shapes[k]))) for k in self.obs_keys}
            for _ in env_fns]
        self.parent_pipes = []
        self.procs = []
        with clear_mpi_env_vars():
            for env_fn, obs_buf in zip(env_fns, self.obs_bufs):
                wrapped_fn = CloudpickleWrapper(env_fn)
                parent_pipe, child_pipe = ctx.Pipe()
                proc = ctx.Process(target=_subproc_worker,
                            args=(child_pipe, parent_pipe, wrapped_fn, obs_buf, self.obs_shapes, self.obs_dtypes, self.obs_keys))
                proc.daemon = True
                self.procs.append(proc)
                self.parent_pipes.append(parent_pipe)
                proc.start()
                child_pipe.close()
        self.waiting_step = False
        self.viewer = None

    def step_async(self, actions):
        assert len(actions) == len(self.parent_pipes)
        for pipe, act in zip(self.parent_pipes, actions):
            pipe.send(('step', act))

    def step_wait(self):
        outs = [pipe.recv() for pipe in self.parent_pipes]
        obs, rews, dones, infos = zip(*outs)
        return self._decode_obses(obs), np.array(rews), np.array(dones), infos

def make_env(env_id, seed, rank, log_dir, allow_early_resets, env_kwargs=None):
    def _thunk():
        if env_id.find("doorenv")>-1:
            env = gym.make(env_id, **env_kwargs) #
            env._max_episode_steps = 512
            if env_kwargs['unity']:
                env.env.init(rank)
        elif env_id.find('Fetch')>-1:
            env = gym.make(env_id, reward_type="sparse")
            env = gym.wrappers.FlattenDictWrapper(env, dict_keys=['observation', 'desired_goal'])
        else:
            if env_id.startswith("dm"):
                _, domain, task = env_id.split('.')
                env = dm_control2gym.make(domain_name=domain, task_name=task)
            else:
                env = gym.make(env_id) 
                env._max_episode_steps = 512
        is_atari = hasattr(gym.envs, 'atari') and isinstance(
            env.unwrapped, gym.envs.atari.atari_env.AtariEnv)
        if is_atari:
            env = make_atari(env_id)

        env.seed(seed + rank)

        obs_shape = env.observation_space.shape

        if str(env.__class__.__name__).find('TimeLimit') >= 0:
            env = TimeLimitMask(env)

        if log_dir is not None:
            env = bench.Monitor(
                env,
                os.path.join(log_dir, str(rank)),
                allow_early_resets=allow_early_resets)

        if is_atari:
            if len(env.observation_space.shape) == 3:
                env = wrap_deepmind(env)
        elif len(env.observation_space.shape) == 3:
            raise NotImplementedError(
                "CNN models work only for atari,\n"
                "please use a custom wrapper for a custom pixel input env.\n"
                "See wrap_deepmind for an example.")

        # If the input has shape (W,H,3), wrap for PyTorch convolutions
        obs_shape = env.observation_space.shape
        if len(obs_shape) == 3 and obs_shape[2] in [1, 3]:
            env = TransposeImage(env, op=[2, 0, 1])

        return env

    return _thunk

def make_vec_envs(env_name,
                  seed,
                  num_processes,
                  gamma,
                  log_dir,
                  device,
                  allow_early_resets,
                  num_frame_stack=None,
                  env_kwargs=None,):
    envs = [
        make_env(env_name, seed, i, log_dir, allow_early_resets, env_kwargs)
        for i in range(num_processes)
    ]

    if len(envs) > 1:
        envs = ShmemVecEnv_DR(envs, context='fork')
    else:
        envs = DummyVecEnv(envs)

    if len(envs.observation_space.shape) == 1:
        if gamma is None:
            envs = VecNormalize(envs, ret=False)
        else:
            envs = VecNormalize(envs, gamma=gamma)

    envs = VecPyTorch(envs, device)

    if num_frame_stack is not None:
        envs = VecPyTorchFrameStack(envs, num_frame_stack, device)
    elif len(envs.observation_space.shape) == 3:
        envs = VecPyTorchFrameStack(envs, 4, device)

    return envs


# Checks whether done was caused my timit limits or not
class TimeLimitMask(gym.Wrapper):
    def step(self, action):
        obs, rew, done, info = self.env.step(action)
        if done and self.env._max_episode_steps == self.env._elapsed_steps:
            info['bad_transition'] = True

        return obs, rew, done, info

    def reset(self, **kwargs):
        return self.env.reset(**kwargs)


# Can be used to test recurrent policies for Reacher-v2
class MaskGoal(gym.ObservationWrapper):
    def observation(self, observation):
        if self.env._elapsed_steps > 0:
            observation[-2:] = 0
        return observation


class TransposeObs(gym.ObservationWrapper):
    def __init__(self, env=None):
        """
        Transpose observation space (base class)
        """
        super(TransposeObs, self).__init__(env)


class TransposeImage(TransposeObs):
    def __init__(self, env=None, op=[2, 0, 1]):
        """
        Transpose observation space for images
        """
        super(TransposeImage, self).__init__(env)
        assert len(op) == 3, f"Error: Operation, {str(op)}, must be dim3"
        self.op = op
        obs_shape = self.observation_space.shape
        self.observation_space = Box(
            self.observation_space.low[0, 0, 0],
            self.observation_space.high[0, 0, 0], [
                obs_shape[self.op[0]], obs_shape[self.op[1]],
                obs_shape[self.op[2]]
            ],
            dtype=self.observation_space.dtype)

    def observation(self, ob):
        return ob.transpose(self.op[0], self.op[1], self.op[2])


class VecPyTorch(VecEnvWrapper):
    def __init__(self, venv, device):
        """Return only every `skip`-th frame"""
        super(VecPyTorch, self).__init__(venv)
        self.device = device
        # TODO: Fix data types

    def reset(self):
        obs = self.venv.reset()
        obs = torch.from_numpy(obs).float().to(self.device)
        return obs

    def step_async(self, actions):
        if isinstance(actions, torch.LongTensor):
            # Squeeze the dimension for discrete actions
            actions = actions.squeeze(1)
        actions = actions.cpu().numpy()
        self.venv.step_async(actions)

    def step_wait(self):
        obs, reward, done, info = self.venv.step_wait()
        obs = torch.from_numpy(obs).float().to(self.device)
        reward = torch.from_numpy(reward).unsqueeze(dim=1).float()
        return obs, reward, done, info


class VecNormalize(VecNormalize_):
    def __init__(self, *args, **kwargs):
        super(VecNormalize, self).__init__(*args, **kwargs)
        self.training = True

    def _obfilt(self, obs, update=True):
        obfilt = False
        if obfilt:
            if self.ob_rms:
                if self.training and update:
                    self.ob_rms.update(obs)
                obs = np.clip((obs - self.ob_rms.mean) /
                            np.sqrt(self.ob_rms.var + self.epsilon),
                            -self.clipob, self.clipob)
                return obs
        return obs


    def train(self):
        self.training = True

    def eval(self):
        self.training = False


# Derived from
# https://github.com/openai/baselines/blob/master/baselines/common/vec_env/vec_frame_stack.py
class VecPyTorchFrameStack(VecEnvWrapper):
    def __init__(self, venv, nstack, device=None):
        self.venv = venv
        self.nstack = nstack

        wos = venv.observation_space  # wrapped ob space
        self.shape_dim0 = wos.shape[0]

        low = np.repeat(wos.low, self.nstack, axis=0)
        high = np.repeat(wos.high, self.nstack, axis=0)

        if device is None:
            device = torch.device('cpu')
        self.stacked_obs = torch.zeros((venv.num_envs, ) +
                                       low.shape).to(device)

        observation_space = gym.spaces.Box(
            low=low, high=high, dtype=venv.observation_space.dtype)
        VecEnvWrapper.__init__(self, venv, observation_space=observation_space)

    def step_wait(self):
        obs, rews, news, infos = self.venv.step_wait()
        self.stacked_obs[:, :-self.shape_dim0] = \
            self.stacked_obs[:, self.shape_dim0:]
        for (i, new) in enumerate(news):
            if new:
                self.stacked_obs[i] = 0
        self.stacked_obs[:, -self.shape_dim0:] = obs
        return self.stacked_obs, rews, news, infos

    def reset(self):
        obs = self.venv.reset()
        if torch.backends.cudnn.deterministic:
            self.stacked_obs = torch.zeros(self.stacked_obs.shape)
        else:
            self.stacked_obs.zero_()
        self.stacked_obs[:, -self.shape_dim0:] = obs
        return self.stacked_obs

    def close(self):
        self.venv.close()
