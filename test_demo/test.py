# python test.py --logdir='./logdir' --port=2021 --record=True

import os
import sys
sys.path.append("..")
import pathlib
import gym
import argparse
import numpy as np
import torch
from torchvision import transforms
from CarlaHighwayTestEnv import CarlaEnv, PlayGame


class CarlaWrapper:

  def __init__(self, mode='test', logdir='', port=2000, 
               scenario=None, test_scen=None, record=False):
    self.mode = mode
    logdir = pathlib.Path(logdir).expanduser()
    logdir = logdir / 'carla_log'
    self.scenario = scenario
    self.test_scen = test_scen
    world, client = PlayGame.setup_world(host='localhost', port=port,
                                         fixed_delta_seconds=0.05, reload=True)
    if world is None:
        raise ValueError("No Carla world found.")
    traffic_manager = client.get_trafficmanager(port+6000)
    if mode == 'train':
        self._env = CarlaEnv(world, traffic_manager, port)
    else:
        self._env = CarlaEnv(world, traffic_manager, port, 
                             client=client, logdir=logdir, record=record)

  @property
  def observation_space(self):
    return self._env.observation_space

  @property
  def action_space(self):
    return self._env.action_space

  def step(self, action):
    return self._env.step(action)

  def reset(self):
    if self.mode == 'train':
      obs = self._env.reset(scenario=6)  # The trainning scenario
    else:
      obs = self._env.reset(scenario=self.scenario, test_scen=self.test_scen)
    return obs

  def render(self, *args, **kwargs):
    pass


class Actor(object):

    def __init__(self):
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.trans = transforms.ToTensor()
        self.eval_net = torch.jit.load('d3qn.pt')

    def __call__(self, state):
        state = self.trans(state).unsqueeze(0).to(self.device)
        action_value = self.eval_net(state).cpu()
        action = torch.max(action_value, 1)[1].data.numpy()
        action = action[0]
        return action


def main():
    parser = argparse.ArgumentParser(description='Training parameters')
    parser.add_argument('--logdir', required=True, help='Directory for storing test results')
    parser.add_argument('--port', type=int, default=2000, help='CARLA client port')
    parser.add_argument('--runs', type=int, default=423, help='Test runs')
    parser.add_argument('--record', type=bool, default=False, help='Record test results')
    parser.add_argument('--scenario', type=int, help='Test a scenario class')
    parser.add_argument('--test_scen', type=int, help='Test a scenario')
    args = parser.parse_args()

    env = CarlaWrapper(logdir=args.logdir, port=args.port, 
                       scenario=args.scenario, test_scen=args.test_scen, record=args.record)
    model = Actor()

    # Test trained agent
    for i in range(args.runs):
        obs = env.reset()
        done = False
        while not done:
            action = model(obs)
            # print(action)
            obs, rewards, done, info = env.step(action)


if __name__ == '__main__':
    main()
