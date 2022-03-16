from EnvLib.ObstGeomEnv import *
from time import sleep
import gym
import torch
import matplotlib.pyplot as plt
import numpy as np
import os

from sklearn.utils import shuffle

# device = torch.device("cuda:2" if torch.cuda.is_available() else "cpu")


class Memory:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []

    def clear_memory(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]
    
    def rewards_monte_carlo(self, gamma):    
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.rewards), reversed(self.is_terminals)):
            # обнуляем накопленную награду, если попали в терминальное состояние
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + gamma * discounted_reward
            #discounted_reward += reward
            
            rewards.insert(0, discounted_reward)

        self.rewards = rewards

    def shuffle(self):
        randomize = np.arange(len(self.rewards))
        np.random.shuffle(randomize)
        self.actions = list(np.array(self.actions)[randomize])
        self.states = list(np.array(self.states)[randomize])
        self.logprobs = list(np.array(self.logprobs)[randomize])
        self.rewards = list(np.array(self.rewards)[randomize])
        #self.is_terminals = self.is_terminals[randomize]



# def validate(env, agent, max_steps, saveImage=True):
#     agent.eval()
#     state = env.reset()
#     if saveImage:
#         images = []
#         images.append(env.render())
#     isDone = False
#     t = 0
#     sum_reward = 0
#     while not isDone and t < max_steps:
#         #action = agent.act(state, False)
#         action = agent.get_action(state)
#         #print("action: ", action)
#         state, reward, isDone, _ = env.step(action)
#         #print("state: ", state)
#         sum_reward += reward
#         if saveImage:
#             #print("env.render(): ", env.render())
#             images.append(env.render())
#         t += 1
        
#     env.close()
#     if saveImage:
#         images = np.transpose(np.array(images), axes=[0, 3, 1, 2])
#     return sum_reward if not saveImage else images


def validate(env, agent, train_config, idx=None, saveImage=True, soft=False, val_key=None):
    # print("#######validate: ", idx)
    env = deepcopy(env)
    max_timesteps = train_config["max_time_steps"]
    frame_stack = train_config["frame_stack"]
    frame_skip = train_config["frame_skip"]

    state = env.reset(idx=idx, fromTrain=False, val_key=val_key)
    
    min_distance = float('inf')
    sum_reward = 0
    if saveImage:
        images = []
        images.append(env.render(sum_reward))
    isDone = False
    t = 0
    lst_states = []
    while not isDone and t < max_timesteps:
        lst_states.append(state)
        if len(lst_states) > frame_stack:
            lst_states = lst_states[1:]
        else:
            lst_states = lst_states * frame_stack
                    
        if not t % frame_skip:
            frame_state = np.hstack(lst_states)
            action = agent.get_action(frame_state, deterministic=True)

        state, reward, isDone, info = env.step(action, soft=soft)
        
        if "EuclideanDistance" in info:
            if min_distance >= info["EuclideanDistance"]:
                min_distance = info["EuclideanDistance"]
        
        sum_reward += reward
        if saveImage:
            images.append(env.render(sum_reward))
        t += 1

        if "SoftEps" in info or "Collision" in info:
            break
        
    env.close()
    if saveImage:
        images = np.transpose(np.array(images), axes=[0, 3, 1, 2])
    return isDone if not saveImage else images, min_distance, info


    
def ppo_batch_train(env, agent, train_config, wandb=None, saveImage=True):
    acceptable_success = train_config["acceptable_success"]
    val_interval_time = train_config["val_time_steps"]
    # val_interval_time = train_config["batch_size"]
    update_timestep = train_config["batch_size"]
    log_interval = train_config["log_interval"]
    max_episodes = train_config["max_episodes"]
    max_timesteps = train_config["max_time_steps"]
    name_save = train_config["name_save"]
    frame_stack = train_config["frame_stack"]
    frame_skip = train_config["frame_skip"]

    running_reward = 0
    timestep = 0
    counter_done = 0
    counter_collision = 0
    total_distance = 0
    old_val_rate = -1
    val_rate = 0 
    curr_counter = 0
    memory = Memory()

    # цикл обучения
    for i_episode in range(1, max_episodes + 1):
        lst_states = []
        state = env.reset()
        # env.render(100, save_image=False)
        min_distance = float('inf')
        # print("i: ", i_episode)
        for t in range(max_timesteps):
            timestep += 1
            lst_states.append(state)

            if len(lst_states) > frame_stack:
                lst_states = lst_states[1:]
            else:
                lst_states = lst_states * frame_stack
                
            if not t % frame_skip:
                frame_state = np.hstack(lst_states)
                action, log_prob = agent.policy_old.act(frame_state)
                # add or delete .cpu()
                # memory.states.append(torch.FloatTensor(frame_state))
                memory.states.append(torch.FloatTensor(frame_state).to(agent.device))
                memory.actions.append(action)
                memory.logprobs.append(log_prob)
                action_step = action.cpu().numpy()
            
            state, reward, done, info = env.step(action_step)
            
            if not t % frame_skip:
                memory.rewards.append(reward)
                memory.is_terminals.append(done)
            
            if "EuclideanDistance" in info:
                if min_distance >= info["EuclideanDistance"]:
                    min_distance = info["EuclideanDistance"]
            
            running_reward += reward

            # выполняем обновление
            if timestep % update_timestep == 0:
                update = agent.update(memory)
                # total_loss, policy_loss, value_loss, dist_entropy = update
                # wandb.log({'total_loss': total_loss, 'policy_loss': policy_loss, 'value_loss': value_loss, 'dist_entropy': dist_entropy})
                _, _, _, dist_entropy = update
                wandb.log({'dist_entropy': dist_entropy})
                memory.clear_memory()
            
            if timestep % val_interval_time == 0:
                valDoneCounter = 0
                min_total_val_dist = 0
                val_counter_collision = 0
                n_vals = 0
                for key in env.valTasks:
                    # print("len(env.valTasks[key]): ", len(env.valTasks[key]))
                    for i in range(len(env.valTasks[key])):
                        isDone, min_val_dist, info = validate(env, agent, train_config, idx=i, saveImage=False, val_key=key)
                        if "Collision" in info:
                            val_counter_collision += 1
                        valDoneCounter += int(isDone)
                        min_total_val_dist += min_val_dist 
                    n_vals += len(env.valTasks[key])
                    res, _, _ = validate(env, agent, train_config, val_key=key)

                if not n_vals:
                    n_vals = 1
                val_rate = valDoneCounter / n_vals * 100
                min_total_val_dist /= n_vals
                val_counter_collision /= n_vals
                val_counter_collision *= 100

                if val_rate >= old_val_rate:
                    if train_config["curriculum"]:
                        agent.save(os.path.join('./myModelWeight1', 'Alpha' + name_save + 'Curriculum'))
                    else:
                        agent.save(os.path.join('./myModelWeight1', 'Alpha' + name_save))
                    old_val_rate = val_rate
                
                if ((val_rate >= acceptable_success) and (curr_counter < 2)):
                    # print(acceptable_success)
                    curr_counter += 1
                    old_val_rate = -1
                    env.weakenRestrictions()

                if wandb is not None:
                    wandb.log({'val_rate': val_rate, 'min_val_distance' : min_total_val_dist, "val_collision_rate": val_counter_collision})
            
            if timestep % (val_interval_time * 10) == 0:
                if saveImage and wandb is not None:
                    wandb.log({f"Trajectory_{timestep}_validate": wandb.Video(res, fps=10, format="gif")})
            
            if "SoftEps" in info or "Collision" in info:
                if "Collision" in info:
                    counter_collision += 1
                break

            if done:
                counter_done += 1
                break

        total_distance += min_distance 
                
        # логирование
        if i_episode % log_interval == 0:
            running_reward /= log_interval
            counter_done /= log_interval
            counter_done *= 100
            total_distance /= log_interval
            counter_collision /= log_interval
            counter_collision *= 100
            # print(running_reward)
            # print(counter_done)
            # print(total_distance)
            # print(counter_collision)
            wandb.log({'running_reward': running_reward, 'success_rate': counter_done, 'min_distance' : total_distance, 'collision_rate': counter_collision})
            running_reward = 0
            counter_done = 0
            total_distance = 0
            counter_collision = 0
        