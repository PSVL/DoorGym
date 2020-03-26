import numpy as np
import torch

def obs2img_vec(inputs, joints_nn):
    img_size = 256
    joints = inputs[:joints_nn*2]
    finger_tip_target = inputs[joints_nn*2:joints_nn*2+3]
    img_front = torch.from_numpy(inputs[joints_nn*2+3:-3*img_size*img_size]).view(-1, 3, img_size, img_size).float().cuda()
    img_top   = torch.from_numpy(inputs[-3*img_size*img_size:]).view(-1, 3, img_size, img_size).float().cuda()

    return joints, finger_tip_target, img_front, img_top

def obs2inputs(inputs, visionmodel, joints_nn):
    joints, finger_tip_target, img_front, img_top = obs2img_vec(inputs, joints_nn)

    with torch.no_grad():
        pp, hm1, hm2 = visionmodel(img_top, img_front)

    knob_target = pp.data.cpu().numpy()
    dist_vec = finger_tip_target - knob_target
    dist_vec = np.squeeze(dist_vec)

    inputs = np.concatenate((joints, dist_vec), 0)
    return inputs

def add_noise(obs, epoch=50):
    satulation = 50.
    obs_tensor = torch.from_numpy(obs).float()
    sdv = torch.tensor([3.440133806003181, 3.192113342496682, 1.727412865751099]) /100.  #Vision SDV for arm
    noise = torch.distributions.Normal(torch.tensor([0.0, 0.0, 0.0]), sdv).sample()
    noise *= min(1., epoch/satulation)
    obs_tensor[-3:] += noise
    obs = obs_tensor.data.cpu().numpy()
    return obs

def multitask_rollout(
        env,
        agent,
        max_path_length=np.inf,
        render=False,
        render_kwargs=None,
        observation_key=None,
        desired_goal_key=None,
        get_action_kwargs=None,
        return_dict_obs=False,
):
    if render_kwargs is None:
        render_kwargs = {}
    if get_action_kwargs is None:
        get_action_kwargs = {}
    dict_obs = []
    dict_next_obs = []
    observations = []
    actions = []
    rewards = []
    terminals = []
    agent_infos = []
    env_infos = []
    next_observations = []
    path_length = 0
    agent.reset()
    o = env.reset()
    if render:
        env.render(**render_kwargs)
    goal = o[desired_goal_key]
    while path_length < max_path_length:
        dict_obs.append(o)
        if observation_key:
            o = o[observation_key]
        new_obs = np.hstack((o, goal))
        a, agent_info = agent.get_action(new_obs, **get_action_kwargs)
        next_o, r, d, env_info = env.step(a)
        if render:
            env.render(**render_kwargs)
        observations.append(o)
        rewards.append(r)
        terminals.append(d)
        actions.append(a)
        next_observations.append(next_o)
        dict_next_obs.append(next_o)
        agent_infos.append(agent_info)
        env_infos.append(env_info)
        path_length += 1
        if d:
            break
        o = next_o
    actions = np.array(actions)
    if len(actions.shape) == 1:
        actions = np.expand_dims(actions, 1)
    observations = np.array(observations)
    next_observations = np.array(next_observations)
    if return_dict_obs:
        observations = dict_obs
        next_observations = dict_next_obs
    return dict(
        observations=observations,
        actions=actions,
        rewards=np.array(rewards).reshape(-1, 1),
        next_observations=next_observations,
        terminals=np.array(terminals).reshape(-1, 1),
        agent_infos=agent_infos,
        env_infos=env_infos,
        goals=np.repeat(goal[None], path_length, 0),
        full_observations=dict_obs,
    )

def rollout(
        env,
        agent,
        max_path_length=np.inf,
        render=False,
        render_kwargs=None,
        evaluate=False,
        verbose=True,
        doorenv=True,
        pos_control=False,
        step_skip=1,
        epoch=0,
):
    """
    The following value for the following keys will be a 2D array, with the
    first dimension corresponding to the time dimension.
     - observations
     - actions
     - rewards
     - next_observations
     - terminals

    The next two elements will be lists of dictionaries, with the index into
    the list being the index into the time
     - agent_infos
     - env_infos
    """
    if doorenv:
        env_obj = env.wrapped_env

    if render_kwargs is None:
        render_kwargs = {}
    observations = []
    actions = []
    rewards = []
    terminals = []
    agent_infos = []
    env_infos = []
    o = env.reset()
    # print(o, o.shape)
    initial_state = o[:env.action_space.shape[0]]
    agent.reset()
    if doorenv:
        if agent.visionnet_input:
            o = obs2inputs(o, agent.visionmodel, agent.nn)
        else:
            if agent.knob_noisy:
                if evaluate:
                    o = add_noise(o)
                else:
                    o = add_noise(o, epoch)
    next_o = None
    path_length = 0
    if render:
        env.render(**render_kwargs)

    door_opened = False
    opening_time = None
    current_state = initial_state

    while path_length < max_path_length:
        a, agent_info = agent.get_action(o)
        next_a = a

        if pos_control:
            # print("main step_skip",args.step_skip)
            # if path_length%(512/step_skip-1)==0: current_state = initial_state
            next_a = current_state + next_a
            # print("stepskip:",step_skip)
            for kk in range(step_skip):
                next_o, r, d, env_info = env.step(next_a)
                
            current_state = next_o[:env.action_space.shape[0]]
        else:
            for kk in range(step_skip):
                next_o, r, d, env_info = env.step(next_a)
                # print("reward ",r)
                # full_obs, reward, done, infos = env.step(next_action)

        
        # next_o, r, d, env_info = env.step(a)
        if doorenv:
            if agent.visionnet_input:
                next_o = obs2inputs(next_o, agent.visionmodel, agent.nn)
            else:
                if agent.knob_noisy:
                    if evaluate:
                        next_o = add_noise(next_o)
                    else:
                        next_o = add_noise(next_o, epoch)

        observations.append(o)
        rewards.append(r)
        terminals.append(d)
        actions.append(a)
        agent_infos.append(agent_info)
        env_infos.append(env_info)
        path_length += 1
        if d:
            break
        o = next_o
        if render:
            env.render(**render_kwargs)

        if doorenv:
            if evaluate and not door_opened and abs(env_obj.get_doorangle())>=0.2:
                opening_time = path_length/50
                if verbose:
                    print("door opened! opening time is {}".format(opening_time))
                door_opened = True
            
    actions = np.array(actions)
    if len(actions.shape) == 1:
        actions = np.expand_dims(actions, 1)
    observations = np.array(observations)
    if len(observations.shape) == 1:
        observations = np.expand_dims(observations, 1)
        next_o = np.array([next_o])
    next_observations = np.vstack(
        (
            observations[1:, :],
            np.expand_dims(next_o, 0)
        )
    )

    if evaluate:
        return (dict(
            observations=observations,
            actions=actions,
            rewards=np.array(rewards).reshape(-1, 1),
            next_observations=next_observations,
            terminals=np.array(terminals).reshape(-1, 1),
            agent_infos=agent_infos,
            env_infos=env_infos,
        ),
        door_opened,
        opening_time)
    else:
        return dict(
            observations=observations,
            actions=actions,
            rewards=np.array(rewards).reshape(-1, 1),
            next_observations=next_observations,
            terminals=np.array(terminals).reshape(-1, 1),
            agent_infos=agent_infos,
            env_infos=env_infos,
        )
