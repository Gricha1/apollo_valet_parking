import json
from policy_gradient.curriculum_train import trainCurriculum
import ray.rllib.agents.ppo as ppo
import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

with open("configs/train_configs.json", 'r') as f:
    train_config = json.load(f)

with open("configs/environment_configs.json", 'r') as f:
    our_env_config = json.load(f)

with open("configs/reward_weight_configs.json", 'r') as f:
    reward_config = json.load(f)

with open("configs/car_configs.json", 'r') as f:
    car_config = json.load(f)


config = ppo.DEFAULT_CONFIG.copy()
config['framework'] = train_config['framework']
config['train_batch_size'] = train_config['train_batch_size']
config['lambda'] = train_config['lambda']
config['use_critic'] = train_config['use_critic']
config['use_gae'] = train_config['use_gae']
config['horizon'] = train_config['horizon']
config['rollout_fragment_length'] = train_config['rollout_fragment_length']
config['num_gpus'] = train_config['num_gpus']
config['num_workers'] = train_config['num_workers']
config['lr'] = train_config['lr']
config['sgd_minibatch_size'] = train_config['sgd_minibatch_size']
config['num_sgd_iter'] = train_config['num_sgd_iter']
config['clip_param'] = train_config['clip_param']
config["optimizer"] = train_config["optimizer"]
config['entropy_coeff'] = train_config['entropy_coeff']
config['vf_clip_param'] = train_config['vf_clip_param']
config['normalize_actions'] = train_config['normalize_actions']
config['batch_mode'] = train_config['batch_mode']
config['_fake_gpus'] = train_config['_fake_gpus']
config['vf_loss_coeff'] =  train_config['vf_loss_coeff']
config["model"]['fcnet_hiddens'] = [512, 512]
config["model"]["use_lstm"] = train_config['use_lstm']
config["model"]["lstm_use_prev_action"] = train_config['lstm_use_prev_action']
config["model"]["free_log_std"] = train_config["free_log_std"]

trainCurriculum(config, train_config, our_env_config, reward_config, car_config)