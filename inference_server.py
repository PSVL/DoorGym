import argparse
import os
import sys
import numpy as np
import torch
import time
import zmq

from util import load_visionmodel, prepare_env
# from a2c_ppo_acktr.envs import VecPyTorch, make_vec_envs
# from a2c_ppo_acktr.utils import get_render_func, get_vec_normalize
from trained_visionmodel.visionmodel import VisionModelXYZ, VisionModel

from rlkit.samplers.rollout_functions import rollout
from rlkit.torch.pytorch_util import set_gpu_mode
from rlkit.core import logger
from rlkit.envs.wrappers import NormalizedBoxEnv
import uuid

def eval_print(dooropen_counter, counter, start_time, total_time):
    opening_rate = dooropen_counter/counter *100
    if dooropen_counter != 0:
        opening_timeavg = total_time/dooropen_counter
    else:
        opening_timeavg = -1
    print("opening rate {}%. Average time to open is {}.".format(opening_rate, opening_timeavg))
    print("took {}sec to evaluate".format( int(time.mktime(time.localtime())) - start_time ))

class Enjoy():
    def __init__(self):
        import sys
        sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
        sys.path.remove('/home/demo/baxter_ws/devel/lib/python2.7/dist-packages')
        # print(sys.path)

        if env_kwargs['visionnet_input']:
            visionmodel = VisionModelXYZ()
            visionmodel = load_visionmodel(args.load_name, args.visionmodel_path, VisionModelXYZ())  

        actor_critic, ob_rms = torch.load(args.load_name)
        self.actor_critic = actor_critic.eval()
        if env_kwargs['visionnet_input']:
            actor_critic.visionmodel = visionmodel
            actor_critic.visionnet_input = True
        # self.actor_critic.to("cuda:0")
        self.actor_critic.to("cpu")
        self.actor_critic.nn = 8

    def inference(self, full_obs):
        full_obs = torch.from_numpy(full_obs).float()
        # print("size",full_obs.size())
        if env_kwargs['visionnet_input']:
            obs = self.actor_critic.obs2inputs(full_obs, 0)
        else:
            obs = full_obs

        recurrent_hidden_states = torch.zeros(1, self.actor_critic.recurrent_hidden_state_size)
        masks = torch.zeros(1, 1)

        value, action, _, recurrent_hidden_states = self.actor_critic.act(
                    obs, recurrent_hidden_states, masks, deterministic=args.det)

        return action

    def server(self):
        port = "5556"       
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:%s" % port)
        print("server started")
        while True:
            obs = np.array(socket.recv_json())
            obs = np.reshape(obs, (1,obs.shape[0]))
            action = self.inference(obs)
            action = action.data.cpu().numpy()
            action = np.squeeze(action)
            # print("action shape", action.shape)
            action = [float(x) for x in list(action)]
            # print("action list?", action)
            time.sleep (0.01)
            socket.send_json(action)

def onpolicy_inference():
    env = make_vec_envs(
        args.env_name,
        args.seed + 1000,
        1,
        None,
        None,
        device='cuda:0',
        allow_early_resets=False,
        env_kwargs=env_kwargs,)
    env_obj = env.venv.venv.envs[0].env.env
    if args.env_name.find('door')<=-1:
        env_obj.unity = None

    render_func = get_render_func(env)
    if evaluation:
        render_func = None

    if env_kwargs['visionnet_input']:
        visionmodel = VisionModelXYZ()
        visionmodel = load_visionmodel(args.load_name, args.visionmodel_path, VisionModelXYZ())  

    actor_critic, ob_rms = torch.load(args.load_name)
    actor_critic = actor_critic.eval()
    if env_kwargs['visionnet_input'] and args.env_name.find('doorenv')>-1:
        actor_critic.visionmodel = visionmodel
        actor_critic.visionnet_input = env_obj.visionnet_input
    actor_critic.to("cuda:0")

    # if args.env_name.find('doorenv')>-1:
    actor_critic.nn = env_obj.nn

    recurrent_hidden_states = torch.zeros(1,actor_critic.recurrent_hidden_state_size)
    masks = torch.zeros(1, 1)

    full_obs = env.reset()
    if args.env_name.find('doorenv')>-1 and env_obj.visionnet_input:
        obs = actor_critic.obs2inputs(full_obs, 0)
    else:
        obs = full_obs

    if render_func is not None:
        render_func('human')

    if args.env_name.find('doorenv')>-1:
        if env_obj.xml_path.find("float")>-1:
            if env_obj.xml_path.find("hook")>-1:
                doorhinge_idx = 6
            elif env_obj.xml_path.find("gripper")>-1:
                doorhinge_idx = 11
        else:
            if env_obj.xml_path.find("mobile")>-1:
                if env_obj.xml_path.find("hook")>-1:
                    doorhinge_idx = 9
                if env_obj.xml_path.find("gripper")>-1:
                    doorhinge_idx = 14
            else:
                if env_obj.xml_path.find("hook")>-1:
                    doorhinge_idx = 7
                if env_obj.xml_path.find("gripper")>-1:
                    doorhinge_idx = 12

    start_time = int(time.mktime(time.localtime()))

    i=0
    epi_step = 0
    total_time = 0
    epi_counter = 1
    dooropen_counter = 0
    door_opened = False

    test_num = 100

    while True:
        i+=1
        epi_step += 1
        with torch.no_grad():
            value, action, _, recurrent_hidden_states = actor_critic.act(
                obs, recurrent_hidden_states, masks, deterministic=args.det)

        full_obs, reward, done, infos = env.step(action)

        if args.env_name.find('doorenv')>-1 and env_obj.visionnet_input:
            obs = actor_critic.obs2inputs(full_obs, 0)
        else:
            obs = full_obs

        masks.fill_(0.0 if done else 1.0)

        if render_func is not None:
            render_func('human')

        if args.env_name.find('doorenv')>-1:
            if not door_opened and abs(env_obj.sim.data.qpos[doorhinge_idx])>=0.2:
                dooropen_counter += 1
                opening_time = epi_step/50
                print("door opened! opening time is {}".format(opening_time))
                total_time += opening_time
                door_opened = True

        if args.env_name.find('Fetch')>-1:
            if not door_opened and infos[0]['is_success']==1:
                dooropen_counter += 1
                opening_time = epi_step/50
                print("Reached destenation! Time is {}".format(opening_time))
                total_time += opening_time
                door_opened = True
                
        if evaluation:
            if i%512==511:
                if env_obj.unity:
                    env_obj.close()
                env = make_vec_envs(
                args.env_name,
                args.seed + 1000,
                1,
                None,
                None,
                device='cuda:0',
                allow_early_resets=False,
                env_kwargs=env_kwargs,)
                env_obj = env.venv.venv.envs[0].env.env
                if args.env_name.find('doorenv')<=-1:
                    env_obj.unity = None
                env.reset()
                print("{} ep end >>>>>>>>>>>>>>>>>>>>>>>>".format(epi_counter))
                eval_print(dooropen_counter, epi_counter, start_time, total_time)
                epi_counter += 1
                epi_step = 0
                door_opened = False

        if i>=512*test_num:
            eval_print(dooropen_counter, epi_counter-1, start_time, total_time)
            break

def offpolicy_inference():
    import time 
    from gym import wrappers

    filename = str(uuid.uuid4())

    gpu = True

    env, _, _ = prepare_env(args.env_name, args.visionmodel_path, **env_kwargs)

    print("before snapshot")

    snapshot = torch.load(args.load_name)
    policy = snapshot['evaluation/policy']
    if args.env_name.find('doorenv')>-1:
        policy.knob_noisy = args.knob_noisy
        policy.nn = env._wrapped_env.nn
        policy.visionnet_input = env_kwargs['visionnet_input']

    epi_counter = 1
    dooropen_counter = 0
    total_time = 0
    test_num = 100

    if evaluation:
        render = False
    else:
        if not args.unity:
            render = True
        else:
            render = False

    start_time = int(time.mktime(time.localtime()))

    if gpu:
        set_gpu_mode(True)
    while True:
        if args.env_name.find('doorenv')>-1:
            path, door_opened, opening_time = rollout(
                env,
                policy,
                max_path_length=512,
                doorenv=True,
                render=render,
                evaluate=True,
            )
            print("done first")
            if hasattr(env, "log_diagnostics"):
                env.log_diagnostics([path])
            logger.dump_tabular()
            if evaluation:
                env, _, _ = prepare_env(args.env_name, args.visionmodel_path, **env_kwargs)
                if door_opened:
                    dooropen_counter += 1
                    total_time += opening_time
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
            print("{} ep end >>>>>>>>>>>>>>>>>>>>>>>>".format(epi_counter))
            epi_counter += 1

            if args.env_name.find('door')>-1 and epi_counter>test_num:
                eval_print(dooropen_counter, epi_counter, start_time, total_time)
                break

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
    args = parser.parse_args()

    args.det = not args.non_det
    evaluation = args.eval

    env_kwargs = dict(port = args.port,
                    visionnet_input = args.visionnet_input,
                    unity = args.unity,
                    world_path = args.world_path)

    en = Enjoy()
    en.server()
    # obs = np.zeros((1,19))
    # action = en.inference(obs)
    # print("action", action)
    # if args.load_name.find("a2c")>-1 or args.load_name.find("ppo")>-1: 
    #     onpolicy_inference()
    # elif args.load_name.find("sac")>-1 or args.load_name.find("td3")>-1:
    #     offpolicy_inference()
    # else:
    #     raise "not sure which type of algorithm."
