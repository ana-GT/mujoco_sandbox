import gymnasium as gym

# Initialise the environment
env = gym.make("CartPole-v1", render_mode="human")

# Installed pygame and still crashes, with rgb mode does not crash

# Reset the environment to generate the first observation
observation, info = env.reset()

print(f"Starting observation: {observation}")

episode_over = False
total_reward = 0

while not episode_over:

  # Choose an action: 0=push cart left, 1 = push cart right
  action = env.action_space.sample()
  # Take action and see what happens
  observation, reward, terminated, truncated, info = env.step(action)

  total_reward += reward
  episode_over = terminated or truncated

print(f"Episode finished! Total reward: {total_reward}")
env.close()

print(f"Action space: {env.action_space}")
print(f"Sample action: {env.action_space.sample()}")

print(f"Observation space: {env.observation_space}")
print(f"Sample observation: {env.observation_space.sample()}")

#gym.pprint_registry()
