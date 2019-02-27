import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)
import pybullet_envs
import gym
import argparse
import pybullet as p
import pybullet_envs
import time

def test(args):
    count = 0
    if args.render:
        env = gym.make(args.env, render=True)
    else:
        env = gym.make(args.env)
    env.reset()
    sample = env.action_space.sample()
    action = sample
    count = 0
    for i in range(args.steps):
        action = env.action_space.sample()
        action = [0]*len(action)
        obs, rewards, done, _ = env.step(action)
        # print("# ===============================================")
        # print("ob : ", obs)
        count += 1
        if done:
            print(count)
            count = 0
            env.reset()
            pass
        # time.sleep(env.env.timeStep)
        time.sleep(1./240.)

def main():
    import argparse
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env', help='environment ID', default='Draco-v0')
    parser.add_argument('--render', help='OpenGL Visualizer', type=bool, default=True)
    parser.add_argument('--steps', help='Number of steps', type=int, default=10000)

    args = parser.parse_args()
    test(args)

if __name__ == '__main__':
    main()
