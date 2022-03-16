import torch
import torch.nn as nn
from torch.distributions import Categorical
from torch.distributions import MultivariateNormal
import gym
import numpy as np
import os

from .models import ActorCritic

# device = torch.device("cuda:2" if torch.cuda.is_available() else "cpu")


class PPO:
    def __init__(self, state_dim, action_dim, train_config):
        self.lr = train_config["lr"]
        self.betas = train_config["betas"]
        self.gamma = train_config["gamma"]
        self.eps_clip = train_config["eps_clip"]
        self.K_epochs = train_config["K_epochs"]
        self.actor = train_config["actor"]
        self.critic = train_config["critic"]
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # print("Train PPO", self.device)

        self.policy = ActorCritic(state_dim, action_dim, self.actor, self.critic, self.device).to(self.device)
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=self.lr, betas=self.betas)
        self.policy_old = ActorCritic(state_dim, action_dim, self.actor, self.critic, self.device).to(self.device)
        self.policy_old.load_state_dict(self.policy.state_dict())

        self.MseLoss = nn.MSELoss()

    def update(self, memory):
        # Monte Carlo оценка вознаграждений:
        """rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(memory.rewards), reversed(memory.is_terminals)):
            # обнуляем накопленную награду, если попали в терминальное состояние
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + self.gamma * discounted_reward
            #discounted_reward += reward
            
            rewards.insert(0, discounted_reward)

        rewards = torch.tensor(rewards, dtype=torch.float32).to(device)
        # выполните нормализацию вознаграждений (r - mean(r)) / std(r + 1e-5):
        rewards = (rewards - rewards.mean()) / (rewards + 1.0 ** -5).std()  """
        
        memory.rewards_monte_carlo(self.gamma)
        memory.shuffle()
        rewards = memory.rewards
        rewards = torch.tensor(rewards, dtype=torch.float32).to(self.device)
        # выполните нормализацию вознаграждений (r - mean(r)) / std(r + 1e-5):
        rewards = (rewards - rewards.mean()) / (rewards + 1.0 ** -5).std()
        
        # конвертация list в tensor
        old_states = torch.stack(memory.states).to(self.device).detach()
        # print("memory.actions: ", memory.actions)
        old_actions = torch.stack(memory.actions).to(self.device).detach()
        # print("memory.logprobs: ", memory.logprobs)
        old_logprobs = torch.Tensor(memory.logprobs).to(self.device).detach()

        total_loss = 0
        total_policy_loss = 0
        total_mse_loss = 0
        total_dist_entropy = 0
        
        # оптимизация K epochs:
        for _ in range(self.K_epochs):
            # получаем logprobs, state_values, dist_entropy от старой стратегии:
            #print("old_actions:", old_actions)
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)

            #print("logprobs.shape: ", logprobs.shape)
            #print("state_values.shape: ", state_values.shape)
            #print("dist_entropy.shape: ", dist_entropy.shape)

            # находим отношение стратегий (pi_theta / pi_theta__old), через logprobs и old_logprobs.detach():
            ratios =  torch.exp(logprobs - old_logprobs.detach())
            
            # считаем advantages
            advantages = rewards - state_values.detach()
            #print("dist_entropy: ", dist_entropy)
            # Находим surrogate loss:
            surr1 = ratios * advantages
            #print("surr1: ", surr1)
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            #print("surr2: ", surr2)
            mse = 0.5 * self.MseLoss(state_values, rewards)
            #print("mse: ", mse)
            policy_loss = -torch.min(surr1, surr2).mean()
            #print("policy_loss: ", policy_loss)
            #print("dist_entropy: ", dist_entropy)
            loss = policy_loss + mse - 0.01 * dist_entropy
            
            # делаем шаг градиента
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()

            total_loss += loss
            total_mse_loss += mse
            total_policy_loss += policy_loss
            total_dist_entropy += dist_entropy   

        # копируем веса
        self.policy_old.load_state_dict(self.policy.state_dict())

        return total_loss / self.K_epochs, total_policy_loss/ self.K_epochs, total_mse_loss / self.K_epochs, total_dist_entropy / self.K_epochs
        #, np.mean(np.array(lst_dist_entropy))

    def get_action(self, state, deterministic=False):
        #state = torch.FloatTensor(state).to(device)
        self.eval()
        action, _ = self.policy.act(state, deterministic=deterministic)

        return action.detach().cpu().numpy()
    
    def eval(self):
        self.policy.eval()
        self.policy_old.eval()

    def save(self, folder_path):
        if folder_path is not None:
            folder_path = os.path.join(folder_path, 'ppo')
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)

            torch.save(self.policy.state_dict(), f'{folder_path}/policy.pkl')
            # torch.save(self.policy.state_dict(), f'{folder_path}/policy.pkl')
    
    def load(self, folder_path):
        if folder_path is not None:
            folder_path = os.path.join(folder_path, 'ppo')
            self.policy.load_state_dict(torch.load(f'{folder_path}/policy.pkl'))
