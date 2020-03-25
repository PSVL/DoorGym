import abc
import gym

import gtimer as gt
from rlkit.core.rl_algorithm import BaseRLAlgorithm
from rlkit.data_management.replay_buffer import ReplayBuffer
from rlkit.samplers.data_collector import PathCollector
from rlkit.envs.wrappers import NormalizedBoxEnv
from rlkit.samplers.data_collector import MdpPathCollector
import doorenv
import doorenv2


class BatchRLAlgorithm(BaseRLAlgorithm, metaclass=abc.ABCMeta):
    def __init__(
            self,
            trainer,
            exploration_env,
            evaluation_env,
            exploration_data_collector: PathCollector,
            evaluation_data_collector: PathCollector,
            replay_buffer: ReplayBuffer,
            batch_size,
            max_path_length,
            num_epochs,
            num_eval_steps_per_epoch,
            num_expl_steps_per_train_loop,
            num_trains_per_train_loop,
            num_train_loops_per_epoch=10,
            min_num_steps_before_training=0,
        ):
        super().__init__(
            trainer,
            exploration_env,
            evaluation_env,
            exploration_data_collector,
            evaluation_data_collector,
            replay_buffer,
        )
        self.batch_size = batch_size
        self.max_path_length = max_path_length
        self.num_epochs = num_epochs
        self.num_eval_steps_per_epoch = num_eval_steps_per_epoch
        self.num_trains_per_train_loop = num_trains_per_train_loop
        self.num_train_loops_per_epoch = num_train_loops_per_epoch
        self.num_expl_steps_per_train_loop = num_expl_steps_per_train_loop
        self.min_num_steps_before_training = min_num_steps_before_training

    def _train(self):
        print()
        print("start training")
        if self.min_num_steps_before_training > 0:
            init_expl_paths = self.expl_data_collector.collect_new_paths(
                self.max_path_length,
                self.min_num_steps_before_training,
                discard_incomplete_paths=False,
            )
            self.replay_buffer.add_paths(init_expl_paths)
            self.expl_data_collector.end_epoch(-1)

        refill_replay_buffer = False
        ######## Fill up the replay buffer by Domain Randomized trajectry ########
        if refill_replay_buffer:
            print("refilling the replay buffer")
            nums_path_before_training = 50
            for i in range(nums_path_before_training):
                curr_expl_policy = self.expl_data_collector._policy
                curr_expl_paths = self.expl_data_collector.get_epoch_paths()
                if self.env_name.find('doorenv')>-1:
                    if self.env_kwargs['unity']:
                            self.expl_data_collector._env._wrapped_env.close()
                            del self.expl_data_collector
                            print("env disconneted")
                    self.expl_data_collector = MdpPathCollector(
                        NormalizedBoxEnv(gym.make(self.env_name , **self.env_kwargs)),
                        curr_expl_policy,)
                    if self.env_kwargs['unity']:
                        self.expl_data_collector._env.init()

                    self.expl_data_collector._epoch_paths = curr_expl_paths
                new_expl_paths = self.expl_data_collector.collect_new_paths(
                    self.max_path_length,
                    self.num_expl_steps_per_train_loop,
                    discard_incomplete_paths=False,
                )
                self.replay_buffer.add_paths(new_expl_paths)
                print(i ,"th pretrain data collection. current replay_buffer length", self.replay_buffer.get_diagnostics())
        #############################################################################


        for epoch in gt.timed_for(
                range(self._start_epoch, self.num_epochs),
                save_itrs=True,
        ):
            if self.eval_env:
                ###################### DR ######################
                eval_DR = False # Utilized Domain Randomization
                if eval_DR:
                    curr_eval_policy = self.eval_data_collector._policy
                    curr_eval_paths = self.eval_data_collector.get_epoch_paths()
                    if self.env_name.find('doorenv')>-1: 
                        if self.env_kwargs['unity']:
                                pass
                        self.eval_data_collector = MdpPathCollector(
                            NormalizedBoxEnv(gym.make(self.env_name, **self.env_kwargs)),
                            curr_eval_policy,)
                    self.eval_data_collector._epoch_paths = curr_eval_paths
                    self.eval_data_collector._epoch = epoch
                ################################################
                self.eval_data_collector.collect_new_paths(
                    self.max_path_length,
                    self.num_eval_steps_per_epoch,
                    discard_incomplete_paths=True,
                )
                gt.stamp('evaluation sampling')

            print("collecting new data and train")
            for _ in range(self.num_train_loops_per_epoch):
                print()
                print("getting {}th new training data".format(_))
                ###################### DR ######################
                DR = True
                if DR:
                    curr_expl_policy = self.expl_data_collector._policy
                    curr_expl_paths = self.expl_data_collector.get_epoch_paths()
                    if self.env_name.find('doorenv')>-1:
                        if self.env_kwargs['unity']:
                            self.expl_data_collector._env._wrapped_env.close()
                            del self.expl_data_collector
                            print("env disconneted")
                        self.expl_data_collector = MdpPathCollector(
                                NormalizedBoxEnv(gym.make(self.env_name, **self.env_kwargs)),
                                curr_expl_policy)
                        if self.env_kwargs['unity']:
                            self.expl_data_collector._env.init()
                    self.expl_data_collector._epoch_paths = curr_expl_paths
                    self.expl_data_collector._epoch = epoch
                ################################################

                new_expl_paths = self.expl_data_collector.collect_new_paths(
                    self.max_path_length,
                    self.num_expl_steps_per_train_loop,
                    discard_incomplete_paths=False,
                )
                gt.stamp('exploration sampling', unique=False)

                self.replay_buffer.add_paths(new_expl_paths)
                gt.stamp('data storing', unique=False)

                self.training_mode(True)
                for _ in range(self.num_trains_per_train_loop):
                    train_data = self.replay_buffer.random_batch(
                        self.batch_size)
                    self.trainer.train(train_data)
                gt.stamp('training', unique=False)
                self.training_mode(False)

            self._end_epoch(epoch)

