import gym
import numpy as np

from f110_gym.envs.f110_env import F110Env
#from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.common.policies import MlpPolicy
#from stable_baselines import DQN
from stable_baselines import PPO2
from stable_baselines.common.env_checker import check_env


class RacecarEnv(F110Env):
	def __init__(self):
		super().__init__()
		actions_low =  np.array([ 0.5, -0.4189 ], dtype=np.float32)
		actions_high = np.array([ 7.0, 0.4189 ], dtype=np.float32)
		self.action_space = gym.spaces.Box(actions_low, actions_high, dtype=np.float32)

		#observations_low =  np.array([  ], dtype=np.float32)
		#observations_high = np.array([  ], dtype=np.float32)
		self.observation_space = gym.spaces.Box(low=0, high=255, shape=(1080,), dtype=np.float32)
		# loading the map (uses the ROS convention with .yaml and an image file)
		map_path = '../maps/levine_blocked.yaml'
		map_img_ext = '.pgm' # png extension for example
		executable_dir = '/home/daniel/f1tenth_gym/build/'

		# loading physical parameters of the car
		# These could be identified on your own system
		mass= 3.47
		l_r = 0.17145
		I_z = 0.04712
		mu = 0.523
		h_cg = 0.074
		cs_f = 4.718
		cs_r = 5.4562

		self.init_map(map_path, map_img_ext, False, False)
		self.update_params(mu, h_cg, l_r, cs_f, cs_r, I_z, mass, executable_dir)

		# Initial state (for two cars)
		initial_x = [0.0]
		initial_y = [0.0]
		initial_theta = [0.0]
		lap_time = 0.0

		# Resetting the environment
		self.reset({'x': initial_x,
					'y': initial_y,
					'theta': initial_theta})

	def step(self, action):
        # can't step if params not set
        if not self.params_set:
            print('ERROR - Gym Env - Params not set, call update params before stepping.')
            sys.exit()
        # action is a list of steering angles + command velocities
        # also a ego car index
        # action should a DICT with {'ego_idx': int, 'speed':[], 'steer':[]}
        action = {'ego_idx': self.ego_idx, 'speed': [action[0]], 'steer': [action[1]]}

        step_request_proto = sim_requests_pb2.SimRequest()
        step_request_proto.type = 0
        step_request_proto.step_request.ego_idx = action['ego_idx']
        step_request_proto.step_request.requested_vel.extend(action['speed'])
        step_request_proto.step_request.requested_ang.extend(action['steer'])
        # serialization
        step_request_string = step_request_proto.SerializeToString()
        # send step request
        self.socket.send(step_request_string)
        # receive response from sim instance
        sim_response_string = self.socket.recv()
        # print('Gym env - Received response for step request.')
        # parse map response proto
        sim_response_proto = sim_requests_pb2.SimResponse()
        sim_response_proto.ParseFromString(sim_response_string)
        # get results
        # make sure we have the right type of response
        response_type = sim_response_proto.type
        # TODO: also check for stepping fail
        if not response_type == 0:
            print('Gym env - Wrong response type for stepping, exiting...')
            sys.exit()
        observations_proto = sim_response_proto.sim_obs
        # make sure the ego idx matches
        if not observations_proto.ego_idx == action['ego_idx']:
            print('Gym env - Ego index mismatch, exiting...')
            sys.exit()
        # get observations
        carobs_list = observations_proto.observations
        # construct observation dict
        # Observation DICT, assume indices consistent: {'ego_idx':int, 'scans':[[]], 'poses_x':[], 'poses_y':[], 'poses_theta':[], 'linear_vels_x':[], 'linear_vels_y':[], 'ang_vels_z':[], 'collisions':[], 'collision_angles':[]}
        obs = {'ego_idx': observations_proto.ego_idx, 'scans': [], 'poses_x': [], 'poses_y': [], 'poses_theta': [], 'linear_vels_x': [], 'linear_vels_y': [], 'ang_vels_z': [], 'collisions': [], 'collision_angles': [], 'lap_times': [], 'lap_counts': []}
        for car_obs in carobs_list:
            obs['scans'].append(car_obs.scan)
            obs['poses_x'].append(car_obs.pose_x)
            obs['poses_y'].append(car_obs.pose_y)
            if abs(car_obs.theta) < np.pi:
                obs['poses_theta'].append(car_obs.theta)
            else:
                obs['poses_theta'].append(-((2 * np.pi) - car_obs.theta))
            obs['linear_vels_x'].append(car_obs.linear_vel_x)
            obs['linear_vels_y'].append(car_obs.linear_vel_y)
            obs['ang_vels_z'].append(car_obs.ang_vel_z)
            obs['collisions'].append(car_obs.collision)
            obs['collision_angles'].append(car_obs.collision_angle)

        obs['lap_times'] = self.lap_times
        obs['lap_counts'] = self.lap_counts

        # TODO: do we need step reward?
        reward = self.timestep
        # update accumulated time in env
        self.current_time = self.current_time + self.timestep
        # TODO: donezo should be done in simulator? could be done here as well
        self._update_state(obs)
        if self.double_finish:
            done, temp = self._check_done()
            info = {'checkpoint_done': temp}
        else:
            done = self._check_done()
            info = {}

        return np.array(obs["scans"], dtype=np.float32), reward, done, info
        # TODO: return obs, reward, done, info
        #return obs, reward, done, info

    def reset(self, poses=None):

        self.current_time = 0.0
        self.in_collision = False
        self.collision_angles = None
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True]*self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))
        if poses:
            pose_x = poses['x']
            pose_y = poses['y']
            pose_theta = poses['theta']
            self.start_x = pose_x[0]
            self.start_y = pose_y[0]
            self.start_theta = pose_theta[0]
            self.start_xs = np.array(pose_x)
            self.start_ys = np.array(pose_y)
            self.start_thetas = np.array(pose_theta)
            self.start_rot = np.array([[np.cos(-self.start_theta), -np.sin(-self.start_theta)],
                                        [np.sin(-self.start_theta), np.cos(-self.start_theta)]])
            # create reset by pose proto
            reset_request_proto = sim_requests_pb2.SimRequest()
            reset_request_proto.type = 4
            reset_request_proto.reset_bypose_request.num_cars = self.num_agents
            reset_request_proto.reset_bypose_request.ego_idx = 0
            reset_request_proto.reset_bypose_request.car_x.extend(pose_x)
            reset_request_proto.reset_bypose_request.car_y.extend(pose_y)
            reset_request_proto.reset_bypose_request.car_theta.extend(pose_theta)
            reset_request_string = reset_request_proto.SerializeToString()
            self.socket.send(reset_request_string)
        else:
            # create reset proto
            self.start_x = 0.0
            self.start_y = 0.0
            self.start_theta = 0.0
            self.start_rot = np.array([[np.cos(-self.start_theta), -np.sin(-self.start_theta)],
                                        [np.sin(-self.start_theta), np.cos(-self.start_theta)]])
            reset_request_proto = sim_requests_pb2.SimRequest()
            reset_request_proto.type = 2
            reset_request_proto.reset_request.num_cars = self.num_agents
            reset_request_proto.reset_request.ego_idx = 0
            # serialize reset proto
            reset_request_string = reset_request_proto.SerializeToString()
            # send reset proto string
            self.socket.send(reset_request_string)
        # receive response from sim
        reset_response_string = self.socket.recv()
        reset_response_proto = sim_requests_pb2.SimResponse()
        reset_response_proto.ParseFromString(reset_response_string)
        if reset_response_proto.reset_resp.result:
            print('Gym env - Reset failed')
            # TODO: failure handling
            return None
        # TODO: return with gym convention, one step?
        vels = [0.0] * self.num_agents
        angs = [0.0] * self.num_agents
        #action = {'ego_idx': self.ego_idx, 'speed': vels, 'steer': angs}
        action = [vels[0], angs[0]]
        # print('Gym env - Reset done')
        #obs, reward, done, info = self.step(action)
        # print('Gym env - step done for reset')
        return self.step(action)[0]#obs, reward, done, info

# making the environment
racecar_env = RacecarEnv() #gym.make('f110_gym:f110-v0')
#check_env(racecar_env)

model = PPO2(MlpPolicy, racecar_env, verbose=1)
model.learn(total_timesteps=8000)
model.save("../params/deepqn_lab8")