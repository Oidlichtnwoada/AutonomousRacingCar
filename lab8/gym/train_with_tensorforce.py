from __future__ import print_function

import os
import logging
import yaml
import numpy as np

import grpc
import racecar_simulator_pb2
import racecar_simulator_pb2_grpc

from tensorforce import Agent, Environment

import tensorflow as tf
import gym
from gym import spaces

class RacecarEnv(gym.Env):
  metadata = {'render.modes': ['']}

  def __init__(self, server_address):
    super(RacecarEnv, self).__init__()

    self.episode_history = []
    self.current_episode = []

    # Load pre-computed costmaps
    self.map_archive = np.load('maps/f1_aut.npz')
    self.distances_from_nearest_obstacle = self.map_archive['distances_from_nearest_obstacle']
    self.driven_distances = self.map_archive['driven_distances']
    self.normalized_driven_distances = self.driven_distances / np.amax(self.driven_distances.flatten())

    # Read map properties
    with open('maps/f1_aut.yaml') as file:
      self.map_properties = yaml.safe_load(file)

    self.map_origin = self.map_properties['origin'][0:2]
    self.map_resolution = self.map_properties['resolution']

    # Connect to server
    self.channel = grpc.insecure_channel(server_address)
    self.simulator = racecar_simulator_pb2_grpc.SimulatorStub(self.channel)
    self.simulation_tag = "tuw_arc_lab8" # any non-empty string will do
    self.simulation_scenario = "tuw_arc_lab8" # see "racecar_simulator.json" (simulator configuration)
    self.simulation_time_step = 1.0/20.0 # [s]
    self.constant_longitudinal_velocity = 1.0

    # Create simulation
    response = self.create_simulation(self.simulation_scenario)

    # Define action and observation space
    assert(len(response.simulation_state.vehicle_definitions) > 0)

    lidar0_parameters = response.simulation_state.vehicle_definitions[0].laser_range_scanner_parameters
    assert(getattr(lidar0_parameters,'number_of_rays', 0) > 0)

    self.obs_min_range = getattr(lidar0_parameters, 'mininum_range', 0.0)
    self.obs_max_range = getattr(lidar0_parameters, 'maximum_range', 0.0)
    self.obs_low = getattr(lidar0_parameters,'mininum_range', 0.0)
    self.obs_high = getattr(lidar0_parameters,'maximum_range', 0.0)
    self.obs_shape = (getattr(lidar0_parameters,'number_of_rays', 0), )

    self.observation_space = spaces.Box(low=self.obs_low,
                                        high=self.obs_high,
                                        shape=self.obs_shape,
                                        dtype=np.float32)

    vehicle0_constraints = response.simulation_state.vehicle_definitions[0].constraints
    self.action_space = spaces.Box(low=getattr(vehicle0_constraints,'minimum_steering_angle', 0.0),
                                    high=getattr(vehicle0_constraints,'maximum_steering_angle', 0.0),
                                    shape=(1,),
                                    dtype=np.float32)

    self.lap_counter = 0 # currently unused

    print("Action space: {}", self.action_space)
    print("Observation space: {}", self.observation_space)


  def __del__(self):
    np.savez_compressed("trained_models/episode_history.npz",
                        episode_history=np.asarray(self.episode_history))
    self.delete_simulation()
    self.channel.close()

  def create_simulation(self, scenario_name):
    request = racecar_simulator_pb2.CreateSimulationRequest()
    request.scenario_name = scenario_name
    request.simulation_tag = self.simulation_tag
    response = self.simulator.CreateSimulation(request)
    if response.WhichOneof('response') is 'error':
      raise RuntimeError('Failed to create simulation')
    return response

  def delete_simulation(self):
    request = racecar_simulator_pb2.DeleteSimulationRequest()
    request.simulation_tag = self.simulation_tag
    response = self.simulator.DeleteSimulation(request)
    if response.WhichOneof('response') is 'error':
      raise RuntimeError('Failed to delete simulation')
    return response

  def log_episode(self):
    self.episode_history.append(self.current_episode)
    self.current_episode = []
    if np.mod(len(self.episode_history), 100) == 0:
      np.savez_compressed("intermediate/episode_history_{}.npz".format(len(self.episode_history)),
                          episode_history=np.asarray(self.episode_history))

  def reset_simulation(self):
    self.log_episode()
    self.delete_simulation()
    response = self.create_simulation(self.simulation_scenario)
    return response

  def step(self, action):

    if self.use_discrete_actions:
      requested_steering_angle = self.discrete_steering_angles[action]
    else:
      requested_steering_angle = action[0]

    request = racecar_simulator_pb2.SimulationStepRequest()
    request.simulation_tag = self.simulation_tag
    request.time_step = self.simulation_time_step
    control_input = racecar_simulator_pb2.VehicleControlInput()
    control_input.legacy_control_input.longitudinal_velocity = self.constant_longitudinal_velocity
    control_input.legacy_control_input.steering_angle = requested_steering_angle
    request.vehicle_control_input.append(control_input)
    response = self.simulator.SimulationStep(request)

    if response.WhichOneof('response') is 'error':
      raise RuntimeError('Simulation step failed')

    #print(response)

    assert(len(response.simulation_state.vehicle_states) > 0)
    vehicle0_state = response.simulation_state.vehicle_states[0]
    #observation = np.asarray(vehicle0_state.laser_range_scan.ranges)
    observation = np.reshape(a=vehicle0_state.laser_range_scan.ranges, newshape=self.obs_shape)
    if self.use_cnn_policy:
      observation = ((observation - self.obs_min_range) / (self.obs_max_range - self.obs_min_range)) * 255

    world_x = vehicle0_state.pose.x
    world_y = vehicle0_state.pose.y
    map_x = (world_x - self.map_origin[0]) / self.map_resolution
    map_y = (world_y - self.map_origin[1]) / self.map_resolution

    self.current_episode.append(np.array((world_x, world_y, map_x, map_y)))

    normalized_driven_distance = self.normalized_driven_distances[int(map_y),int(map_x)]
    distance_from_obstacle = self.distances_from_nearest_obstacle[int(map_y),int(map_x)]
    driven_distance = self.driven_distances[int(map_y),int(map_x)]

    done = False # indicates whether the current episode should terminate
    reward = 0.0
    reward = reward + driven_distance + (distance_from_obstacle**2)

    if normalized_driven_distance > 0.98: # 98% is just a heuristic, there are better ways to check for lap completion
      print("Reached goal at t={:.4f}, d={:.4f}, r={:.4f}".format(
        response.simulation_state.simulation_time,
        normalized_driven_distance,
        reward))
      reward = +(1E6) # completion reward
      done = True

    # penalize collisions
    vehicle0_is_in_a_collision = getattr(vehicle0_state.collision, 'in_collision', False)
    if vehicle0_is_in_a_collision:
      print("Crash at t={:.4f}, d={:.4f}, r={:.4f}, pos=[{:.4f},{:.4f}]".format(
        response.simulation_state.simulation_time,
        normalized_driven_distance,
        reward,
        world_x,
        world_y))
      reward = reward-(1E3) # collision penalty
      done = True

    info = {} # diagnostic information (not used)
    return observation, reward, done, info

  def reset(self):
    response = self.reset_simulation()
    assert(len(response.simulation_state.vehicle_states) > 0)
    vehicle0_state = response.simulation_state.vehicle_states[0]
    observation = np.reshape(a=vehicle0_state.laser_range_scan.ranges, newshape=self.obs_shape)
    if self.use_cnn_policy:
      observation = ((observation - self.obs_min_range) / (self.obs_max_range - self.obs_min_range)) * 255
    self.lap_counter = 0 # reset lap counter
    return observation

  def render(self, mode=''):
    # Not implemented
    return None

  def close(self):
    # Nothing to do...
    return


def run():
    gym_env = RacecarEnv('localhost:50051')
    environment = Environment.create(environment=gym_env)

    exploration_sigma = float(np.deg2rad(1.0))

    # Create a PPO (Proximal Policy Optimization) agent
    agent = Agent.create(
      agent='ppo', environment=environment,
      seed=1234, # fixed seed for reproducibility
      max_episode_timesteps=10000,
      # use an automatically configured network (CNN with two hidden layers)
      network='auto',
      # optimization parameters
      batch_size=10, update_frequency=5, learning_rate=2.5e-4, subsampling_fraction=0.2,
      optimization_steps=5,
      # reward estimation
      likelihood_ratio_clipping=0.2, discount=0.99, estimate_terminal=False,
      # use an automatically configured critic network (CNN with two hidden layers)
      critic_network='auto',
      critic_optimizer=dict(optimizer='adam', learning_rate=2.5e-4),
      # no preprocessing
      preprocessing=None,
      # enable exploration (add small action noise)
      exploration=exploration_sigma, variable_noise=0.0,
      # regularization
      l2_regularization=0.0, entropy_regularization=0.01,
      # TensorFlow-related options
      name='agent', device=None, parallel_interactions=1, execution=None,
      summarizer=None, recorder=None,
      saver = dict(load=False, directory='intermediate', filename='trained_model', frequency=(60 * 10))
    )

    episode = 0
    try:
      while True:
        states = environment.reset()
        terminal = False

        while not terminal:
          actions = agent.act(states=states)
          states, terminal, reward = environment.execute(actions=actions)
          agent.observe(terminal=terminal, reward=reward)

        episode = episode + 1
        if np.mod(episode, 50)==0:
          print('[>] completed {} episodes'.format(episode))

    except (KeyboardInterrupt, SystemExit):
      print('[!] interrupted after {} episodes'.format(episode))
      agent.save(directory='trained_models', filename="model", format='tensorflow')

    agent.close()
    environment.close()

if __name__ == '__main__':
  verbose = False
  os.environ['TF_CPP_MIN_LOG_LEVEL'] = '0' if verbose else '3'  # 0: DEBUG, 1: INFO, 2: WARNING, 3: ERROR
  tf.get_logger().setLevel('DEBUG' if verbose else 'ERROR')  # {DEBUG, INFO, WARN, ERROR, FATAL}
  MEMORY_LIMIT = 1024 * 4 # Limit GPU memory usage to 6GB
  gpus = tf.config.experimental.list_physical_devices('GPU')
  if gpus:
      try:
          tf.config.experimental.set_virtual_device_configuration(gpus[0], [
              tf.config.experimental.VirtualDeviceConfiguration(memory_limit=MEMORY_LIMIT)])
      except RuntimeError as e:
          print(e)

  logging.basicConfig()
  run()