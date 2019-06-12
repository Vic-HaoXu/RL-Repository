from arguments import get_args
import math
import os
import gym
import gym.spaces
from gym import wrappers
import torch
import numpy as np

from normalized_actions import NormalizedActions

if __name__ == '__main__':
    
    args = get_args()

    # initialize environment
    env_name = args.env_name
    env = gym.make(env_name)

    # choose agent according to action space
    if type(env.action_space) != gym.spaces.discrete.Discrete:
        from reinforce_continuous import REINFORCE
        env = NormalizedActions(gym.make(env_name))
    else:
        from reinforce_discrete import REINFORCE

    if args.display:
        env = wrappers.Monitor(env, '/tmp/{}-experiment'.format(env_name), force=True)

    env.seed(args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    
    agent = REINFORCE(args.hidden_size, env.observation_space.shape[0], env.action_space)

    dir = 'ckpt_' + env_name
    if not os.path.exists(dir):
        os.mkdir(dir)

    for i_episode in range(args.num_episodes):
        env_reset = np.expand_dims(env.reset(), 0)
        state = torch.Tensor(env_reset)
        # print(state)
        entropies = []
        log_probs = []
        rewards = []
        for t in range(args.num_steps):

            action, log_prob, entropy = agent.select_action(state)
            action = action.cpu()

            next_state, reward, done, _ = env.step(action.numpy()[0])

            if args.render:
                env.render()

            entropies.append(entropy)
            log_probs.append(log_prob)
            rewards.append(reward)
            state = torch.Tensor([next_state])

            if done:
                break

        agent.update_parameters(rewards, log_probs, entropies, args.gamma)

        if i_episode % args.ckpt_freq == 0:
            torch.save(agent.model.state_dict(), os.path.join(dir, 'reinforce-' + str(i_episode) + '.pkl'))

        print('Episode: {}, reward: {}'.format(i_episode, np.sum(rewards)))


    env.close()
