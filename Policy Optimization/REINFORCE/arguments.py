import argparse

def get_args():
    parser = argparse.ArgumentParser(description='Pytorch REINFORCE example')
    parser.add_argument('--env_name', type=str, default='CartPole-v0')
    parser.add_argument('--gamma', type=float, default=0.99, help='discount factor for reward')
    parser.add_argument('--exploration_end', type=int, default=100, help='number of episode with noise')
    parser.add_argument('--seed', type=int, default=123, help='random seed')
    parser.add_argument('--num_steps', type=int, default=1000, help='max episode length')
    parser.add_argument('--num_episodes', type=int, default=2000, help='number of episodes')
    parser.add_argument('--hidden_size', type=int, default=128, help='number of episodes')
    parser.add_argument('--render', action='store_true', help='render the environment')
    parser.add_argument('--ckpt_freq', type=int, default=100, help='model saving frequency')
    parser.add_argument('--display', type=bool, default=False, help='display or not')
    args = parser.parse_args()

    return args
