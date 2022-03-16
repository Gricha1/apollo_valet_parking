import json
from policy_gradient.curriculum_train import trainCurriculum
from ray.rllib.agents import ddpg

with open("configs/train_configs_ddpg.json", 'r') as f:
    train_config = json.load(f)

with open("configs/environment_configs_ddpg.json", 'r') as f:
    our_env_config = json.load(f)

with open("configs/reward_weight_configs_ddpg.json", 'r') as f:
    reward_config = json.load(f)

with open("configs/car_configs_ddpg.json", 'r') as f:
    car_config = json.load(f)


config = ddpg.td3.TD3_DEFAULT_CONFIG.copy()

config["framework"] = train_config['framework']
config["policy_delay"] = train_config['policy_delay']
config["train_batch_size"] = train_config['train_batch_size']
config["timesteps_per_iteration"] = train_config['timesteps_per_iteration']
config["horizon"] = train_config['horizon']
config["rollout_fragment_length"] = train_config['rollout_fragment_length']
config["num_gpus"] = train_config['num_gpus']
config["num_workers"] = train_config['num_workers']
config["critic_lr"] = train_config['critic_lr']
config["actor_lr"] = train_config['actor_lr']
config["optimizer"] = train_config['optimizer']
config["normalize_actions"] = train_config['normalize_actions']
config['prioritized_replay'] = train_config['prioritized_replay']
config["batch_mode"] = train_config['batch_mode']
config["_fake_gpus"] = train_config['_fake_gpus']
config["actor_hiddens"] = [512, 512]
config["critic_hiddens"] = [512, 512]
config["actor_hidden_activation"] = "tanh"
config["critic_hidden_activation"] = "tanh"
config["learning_starts"] = train_config["learning_starts"]
config["use_huber"] = train_config["use_huber"]
config["tau"] = train_config["tau"]
# print(config["model"])
# print(config)

trainCurriculum(config, train_config, our_env_config, reward_config, car_config, ppo_algorithm=False)