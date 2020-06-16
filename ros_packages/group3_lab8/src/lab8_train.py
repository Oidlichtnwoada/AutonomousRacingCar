import math

import gym
import numpy as np

import sim_requests_pb2

from f110_gym.envs.f110_env import F110Env
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO2
#from stable_baselines.ddpg.policies import MlpPolicy
#from stable_baselines import DDPG
from stable_baselines.common.env_checker import check_env


class RacecarEnv(F110Env):
    def __init__(self):
        super().__init__()
        actions_low = np.array([1.0, -0.4189], dtype=np.float32)
        actions_high = np.array([1.0, 0.4189], dtype=np.float32)
        self.action_space = gym.spaces.Box(actions_low, actions_high, dtype=np.float32)
        self.lap_driven = False
        self.timeout = 1E18

        # observations_low =  np.array([  ], dtype=np.float32)
        # observations_high = np.array([  ], dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=0, high=255, shape=(54,), dtype=np.float32)
        # loading the map (uses the ROS convention with .yaml and an image file)
        map_path = '../maps/levine_blocked.yaml'
        map_img_ext = '.pgm'  # png extension for example
        executable_dir = '/home/daniel/f1tenth_gym/build/'

        # loading physical parameters of the car
        # These could be identified on your own system
        mass = 3.47
        l_r = 0.17145
        I_z = 0.04712
        mu = 0.523
        h_cg = 0.074
        cs_f = 4.718
        cs_r = 5.4562

        self.init_map(map_path, map_img_ext, False, False)
        self.update_params(mu, h_cg, l_r, cs_f, cs_r, I_z, mass, executable_dir, {'x': [0.480721921921],
                                                                                  'y': [-0.0170010328293],
                                                                                  'theta': [0.0172652381447]})

        # Initial state (for two cars)
        # initial_x = [0.0]
        # initial_y = [0.0]
        # initial_theta = [0.0]
        # lap_time = 0.0

        # Resetting the environment
        print(self.reset().shape)

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
        obs = {'ego_idx': observations_proto.ego_idx, 'scans': [], 'poses_x': [], 'poses_y': [], 'poses_theta': [], 'linear_vels_x': [], 'linear_vels_y': [], 'ang_vels_z': [], 'collisions': [],
               'collision_angles': [], 'lap_times': [], 'lap_counts': []}
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

        # reward = self.timestep
        red_scan = np.array(obs["scans"][-1][240:780], dtype=np.float32)[0::10]

        # TODO: do we need step reward?
        if self.in_collision:
            reward = 0
            #print("collision")
        elif self.lap_driven:
            #print("lap completed")
            reward = 1E9
            self.lap_driven = False
        else:
            #print("step")
            #reward = 1/((red_scan[0] - red_scan[-1])**4) 
            min_val = min(red_scan)
            if min_val < 0.5:
            	reward = -10000
            elif min_val < 0.7:
            	reward = -10
            elif min_val < 0.9:
                reward = 10
            else:
            	reward = 10000

        self.accumulated_rew += reward

        if done:
            print(self.accumulated_rew)

        return red_scan, reward, done, info

    # TODO: return obs, reward, done, info
    # return obs, reward, done, info

    def _check_done(self):
        """
        Check if the episode is done
        This is in terms of the ego car
        For our case, whether the car ends up close enough to the starting point
        And if accumulated time is over the timeout
        return true if done, false if not
        This assumes start is always (0, 0)

        """
        # TODO: start not always 0, 0
        dist_to_start = math.sqrt((self.x-self.start_x) ** 2 + (self.y-self.start_y) ** 2)
        left_t = 1.5
        right_t = 5
        timeout = self.current_time >= self.timeout
        if self.double_finish:
            poses_x = np.array(self.all_x) - self.start_xs
            poses_y = np.array(self.all_y) - self.start_ys
            delta_pt = np.dot(self.start_rot, np.stack((poses_x, poses_y), axis=0))
            temp_y = delta_pt[1, :]
            idx1 = temp_y > left_t
            idx2 = temp_y < -right_t
            temp_y[idx1] -= left_t
            temp_y[idx2] = -right_t - temp_y[idx2]
            temp_y[np.invert(np.logical_or(idx1, idx2))] = 0

            dist2 = delta_pt[0, :] ** 2 + temp_y ** 2
            closes = dist2 <= 0.1
            for i in range(self.num_agents):
                if closes[i] and not self.near_starts[i]:
                    self.near_starts[i] = True
                    self.toggle_list[i] += 1
                elif not closes[i] and self.near_starts[i]:
                    self.near_starts[i] = False
                    self.toggle_list[i] += 1
            done = (self.in_collision | (timeout) | np.all(self.toggle_list >= 4))

            for i in range(self.num_agents):
                self.lap_counts[i] = np.floor(self.toggle_list[i] / 2)
                if self.toggle_list[i] < 4:
                    self.lap_times[i] = self.current_time

            return done, self.toggle_list >= 4

        delta_pt = np.dot(self.start_rot, np.array([self.x - self.start_x, self.y - self.start_y]))
        if delta_pt[1] > left_t:  # left
            temp_y = delta_pt[1] - left_t
        elif delta_pt[1] < -right_t:  # right
            temp_y = -right_t - delta_pt[1]
        else:
            temp_y = 0
        dist2 = delta_pt[0] ** 2 + temp_y ** 2
        # close = dist2 <= 0.1
        close = dist_to_start <= 1
        if close and not self.near_start:
            self.near_start = True
            self.num_toggles += 1
            self.lap_driven = True
        elif not close and self.near_start:
            self.near_start = False
        done = (self.in_collision | timeout | (self.num_toggles >= 10))

        return done

    def reset(self):

        self.accumulated_rew = 0

        self.current_time = 0.0
        self.in_collision = False
        self.collision_angles = None
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True] * self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))
        if self.poses:
            pose_x = self.poses['x']
            pose_y = self.poses['y']
            pose_theta = self.poses['theta']
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
        # action = {'ego_idx': self.ego_idx, 'speed': vels, 'steer': angs}
        action = [vels[0], angs[0]]
        # print('Gym env - Reset done')
        # obs, reward, done, info = self.step(action)
        # print('Gym env - step done for reset')
        return self.step(action)[0]  # obs, reward, done, info

    def update_params(self, mu, h_cg, l_r, cs_f, cs_r, I_z, mass, exe_path, poses, double_finish=False):
        # if not self.sim_p is None:
        #     print('Gym env - Sim server exists, killing...')
        #     self.socket.send(b'dead')
        #     self.sim_p.kill()
        #     os.kill(self.sim_p.pid, signal.SIGINT)
        #     self.sim_p = None
        # print('in update params')

        self.params = [mu, h_cg, l_r, cs_f, cs_r, I_z, mass]
        self.params_set = True

        self.poses = poses

        if self.sim_p is None:
            # print('starting ex and setting map')
            self._start_executable(exe_path)
            self._set_map()
        self.double_finish = double_finish
        # print('before creating proto')

        # create update proto
        update_param_proto = sim_requests_pb2.SimRequest()
        update_param_proto.type = 3
        update_param_proto.update_request.mu = mu
        update_param_proto.update_request.h_cg = h_cg
        update_param_proto.update_request.l_r = l_r
        update_param_proto.update_request.cs_f = cs_f
        update_param_proto.update_request.cs_r = cs_r
        update_param_proto.update_request.I_z = I_z
        update_param_proto.update_request.mass = mass
        # serialize reset proto
        update_param_string = update_param_proto.SerializeToString()
        # print('proto serialized')
        # send update param request
        self.socket.send(update_param_string)
        # print('Gym env - Update param request sent.')
        # receive response
        update_response_string = self.socket.recv()
        update_response_proto = sim_requests_pb2.SimResponse()
        update_response_proto.ParseFromString(update_response_string)
        if update_response_proto.update_resp.result:
            print('Gym env - Update param failed')
            return None


# making the environment
racecar_env = RacecarEnv()  # gym.make('f110_gym:f110-v0')
# check_env(racecar_env)

model = PPO2(MlpPolicy, racecar_env, verbose=0)
model.learn(total_timesteps=150000) #, log_interval=8000)
model.save("../params/deepqn_lab8")