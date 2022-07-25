from gym.envs.registration import register

register(
    id='doorenv-v0',
    entry_point='doorenv.envs:DoorEnv',
    max_episode_steps=512,
    #timestep_limit=512,
)

register(
    id='doorenv_gym-v0',
    entry_point='doorenv.envs:DoorEnv_gym',
    max_episode_steps=512,
    #timestep_limit=512,
)

register(
    id='doorenv_6joints-v0',
    entry_point='doorenv.envs:DoorEnv_6joints',
    max_episode_steps=512,
    #timestep_limit=512,
)


#register(
#    id='reacher3D-v0',
#    entry_point='reacher.envs:Reacher3DEnv',
#    #max_episode_steps=512,
#    timestep_limit=512,
#)
# register(
#     id='doorenv-extrahard-v0',
#     entry_point='doorenv.envs:DoorEnvExtraHard',
# )
