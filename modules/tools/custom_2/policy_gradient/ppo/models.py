import torch
import torch.nn as nn
from torch.distributions import Categorical
from torch.distributions import MultivariateNormal
import gym
import numpy as np
import os

# device = torch.device("cuda:2" if torch.cuda.is_available() else "cpu")

class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_actor, hidden_critic, device):
        super(ActorCritic, self).__init__()
        self.device = device

        #self.action_var = torch.full(size=(action_dim,), fill_value=0.5).to(device)
        #self.cov_mat = torch.diag(self.cov_var).to(device)

        # actor
        self.action_layer = nn.Sequential(
                            nn.Linear(state_dim, hidden_actor[0]),
                            #nn.ReLU(),
                            nn.Tanh(),
                            nn.Linear(hidden_actor[0], hidden_actor[1]),
                            #nn.ReLU(),
                            nn.Tanh(),
                            nn.Linear(hidden_actor[1], action_dim),
                            nn.Tanh()
                            )
        
        # critic
        self.value_layer =  nn.Sequential(
                            nn.Linear(state_dim, hidden_critic[0]),
                            #nn.ReLU(),
                            nn.Tanh(),
                            nn.Linear(hidden_critic[0], hidden_critic[1]),
                            #nn.ReLU(),
                            nn.Tanh(),
                            nn.Linear(hidden_critic[1], 1)
                            )
                            
        # print("ActorCritic device: ", self.device)
        # print(torch.zeros(action_dim, dtype=torch.float32, device=self.device))
        # print("Continue")
        self.logstd = nn.Parameter(
            torch.zeros(action_dim, dtype=torch.float32, device=self.device)
        )
                
    def forward(self):
        raise NotImplementedError

    def act(self, state, deterministic=False):

        state = torch.FloatTensor(state).to(self.device)
        action_mean = self.action_layer(state)
        scale_tril = torch.diag(torch.exp(self.logstd))
        dist = MultivariateNormal(
            action_mean,
            scale_tril=scale_tril,
        )

        action = dist.sample()

        return action.detach() if not deterministic else action_mean.detach(), dist.log_prob(action).detach()
        
    def evaluate(self, state, action):
        #state = torch.FloatTensor(state).to(device)
        #action = torch.FloatTensor(action).to(device)

        action_mean = self.action_layer(state)

        scale_tril = torch.diag(torch.exp(self.logstd))
        #print("action_mean.shape: ", action_mean.shape)
        
        #batch_scale_tril = scale_tril.repeat(batch_dim, 1, 1)
        #print("scale_trilscale_tril.shape: ", scale_tril.shape)
        dist = MultivariateNormal(
            action_mean,
            scale_tril=scale_tril,
        )
        
        action_logprobs = dist.log_prob(action)
        dist_entropy = dist.entropy()
        state_value = self.value_layer(state)
        
        return action_logprobs, torch.squeeze(state_value), dist_entropy.mean()