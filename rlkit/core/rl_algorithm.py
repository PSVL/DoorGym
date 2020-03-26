import abc
from collections import OrderedDict

import gtimer as gt
import os
import torch
import numpy as np
import pickle

from rlkit.core import logger, eval_util
from rlkit.data_management.replay_buffer import ReplayBuffer
from rlkit.samplers.data_collector import DataCollector
from tensorboardX import SummaryWriter


def _get_epoch_timings():
    times_itrs = gt.get_times().stamps.itrs
    times = OrderedDict()
    epoch_time = 0
    for key in sorted(times_itrs):
        time = times_itrs[key][-1]
        epoch_time += time
        times['time/{} (s)'.format(key)] = time
    times['time/epoch (s)'] = epoch_time
    times['time/total (s)'] = gt.get_times().total
    return times


class BaseRLAlgorithm(object, metaclass=abc.ABCMeta):
    def __init__(
            self,
            trainer,
            exploration_env,
            evaluation_env,
            exploration_data_collector: DataCollector,
            evaluation_data_collector: DataCollector,
            replay_buffer: ReplayBuffer,
    ):
        self.trainer = trainer
        self.expl_env = exploration_env
        self.eval_env = evaluation_env
        self.expl_data_collector = exploration_data_collector
        self.eval_data_collector = evaluation_data_collector
        self.replay_buffer = replay_buffer
        self._start_epoch = 0

        #####################################
        self.save_interval = None
        self.save_dir = None
        self.algo = None
        self.env_name = None
        self.save_name = None
        self.env_kwargs = None
        self.eval_function = None
        self.eval_interval = None
        self.env_kwargs_val = None
        self.pos_control = None
        self.step_skip = None
        self.max_path_length = None
        #####################################

        self.post_epoch_funcs = []
        self.writer = None
        # print("writer has been made")

    def train(self, start_epoch=0):
        snapshot = self._get_snapshot()
        # print("what is in a snapshot", snapshot)

        self._start_epoch = start_epoch
        self._train()

    def _train(self):
        """
        Train model.
        """
        raise NotImplementedError('_train must implemented by inherited class')

    def _end_epoch(self, epoch):
        snapshot = self._get_snapshot()
        # print("what is in a snapshot", snapshot)
        # logger.save_itr_params(epoch, snapshot)
        gt.stamp('saving')
        self._log_stats(epoch)

        self.expl_data_collector.end_epoch(epoch)
        self.eval_data_collector.end_epoch(epoch)
        self.replay_buffer.end_epoch(epoch)
        self.trainer.end_epoch(epoch)

        for post_epoch_func in self.post_epoch_funcs:
            post_epoch_func(self, epoch)

        ####################################
        if (epoch % self.save_interval == 0) and self.save_dir != "":
            save_path = os.path.join(self.save_dir, self.algo)
            try:
                os.makedirs(save_path)
            except OSError:
                pass
            torch.save(snapshot,
            os.path.join(save_path, self.env_name + "_{}.{}.pt".format(self.save_name,epoch)))

        replaybuffer_save = False
        if replaybuffer_save:
            if (epoch == 0 or epoch == 100 or epoch == 200 ) and self.save_dir != "":
                save_path = os.path.join(self.save_dir, self.algo)
                save_file = open( os.path.join(save_path, self.env_name + "_{}.replaybuffer.{}".format(self.save_name,epoch)), "wb")
                self.replay_buffer._env_info_keys = list(self.replay_buffer.env_info_sizes)
                pickle.dump(self.replay_buffer, save_file)
        ####################################

        ####################################
        if (self.eval_interval is not None and epoch % self.eval_interval == 0):
            total_num_steps = (epoch + 1) * self.max_path_length
            opening_rate, opening_timeavg = self.eval_function(
                                        seed=99, 
                                        env_name=self.env_name, 
                                        det=True, 
                                        load_name=self.save_name, 
                                        evaluation=True, 
                                        render=False, 
                                        knob_noisy=self.knob_noisy, 
                                        visionnet_input=self.visionnet_input, 
                                        env_kwargs=self.env_kwargs_val,
                                        actor_critic=snapshot['evaluation/policy'],
                                        verbose=False,
                                        pos_control=self.pos_control,
                                        step_skip=self.step_skip)
            print("{}th update. {}th timestep. opening rate {}%. Average time to open is {}.".format(epoch, total_num_steps, opening_rate, opening_timeavg))
            self.writer.add_scalar("Opening rate per envstep", opening_rate, total_num_steps)
            self.writer.add_scalar("Opening rate per update", opening_rate, epoch)
        ####################################

    def _get_snapshot(self):
        snapshot = {}
        for k, v in self.trainer.get_snapshot().items():
            print("trainer: ", k)
            snapshot['trainer/' + k] = v
        for k, v in self.expl_data_collector.get_snapshot().items():
            print("expl_data_collector: ", k)
            if not k=="env":
                snapshot['exploration/' + k] = v
        for k, v in self.eval_data_collector.get_snapshot().items():
            print("eval_data_collector: ", k)
            if not k=="env":
                snapshot['evaluation/' + k] = v
        for k, v in self.replay_buffer.get_snapshot().items():
            print("replay_buffer: ", k)
            snapshot['replay_buffer/' + k] = v
        return snapshot

    def _log_stats(self, epoch):
        print()
        print("####################### RESULT OF EPOCH {} #######################".format(epoch))
        for k, v in self.trainer.get_diagnostics().items():
            if str(k) == "QF1 Loss":
                print("QF1 Loss", v)
                self.writer.add_scalar("QF1 Loss", v, epoch)
            elif str(k) == "QF2 Loss":
                print("QF2 Loss", v)
                self.writer.add_scalar("QF2 Loss", v, epoch)
            elif str(k) == "Policy Loss":
                print("Policy Loss", v)
                self.writer.add_scalar("Policy Loss", v, epoch)
            elif str(k) == "Alpha":
                print("Alpha", v)
                self.writer.add_scalar("Alpha", v, epoch)
            elif str(k) == "Log Pis Mean":
                print("Log Pis Mean", v)
                self.writer.add_scalar("Log Pis Mean", v, epoch)
            elif str(k) == "Target Entropy":
                print("Target Entropy", v)
                self.writer.add_scalar("Target Entropy", v, epoch)

        total_num_steps = (epoch + 1) * self.max_path_length

        expl_paths = self.expl_data_collector.get_epoch_paths()
        d = eval_util.get_generic_path_information(expl_paths)
        for k, v in d.items():
            if str(k) == "Average Returns":
                print("Exploration_rewards", v)
                self.writer.add_scalar("Episode_rewards", v, epoch)
                self.writer.add_scalar("Episode_rewards_envstep", v, total_num_steps)

        eval_util.print_returns_info(expl_paths, epoch)
        if self.eval_env:
            eval_paths = self.eval_data_collector.get_epoch_paths()
            d = eval_util.get_generic_path_information(eval_paths)
            for k, v in d.items():
                if str(k) == "Average Returns":
                    print("Evaluation_rewards", v)
                    self.writer.add_scalar("Evaluation_rewards", v, epoch)
                    self.writer.add_scalar("Evaluation_rewards_envstep", v, total_num_steps)
        print("################################################################")

    @abc.abstractmethod
    def training_mode(self, mode):
        """
        Set training mode to `mode`.
        :param mode: If True, training will happen (e.g. set the dropout
        probabilities to not all ones).
        """
        pass
