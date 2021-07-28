import json

def open_json(self, filename) -> list:
    '''
        Open the json file which stores the path
    '''
    # Opening JSON file
    f = open(filename, )

    # returns JSON object as
    # a dictionary
    data = json.load(f)

    # ic(type(data))
    # ic(type(data['trajectories'][0]['positions']))
    # ic(len(data['trajectories'][0]['positions']))

    self.trajectory = data['trajectories'][0]['positions']
    # ic(traj)
    # ic(traj[0])
    # ic(traj[-1])
    # ic(traj[:][:2])

    # Iterating through the json
    # list
    # for i in data['trajectories'][0]['positions']:
    # 	print(i)

    # Closing file
    f.close()

    self.num_path_points = len(self.trajectory)

    return self.trajectory


def get_x_y_traj(self, traj) -> list:
    '''
        Get the 2d trajectory which is generated by RRTstar in MoveIt
    '''
    length = len(self.trajectory)
    self.xy_traj = [[0, 0] for i in range(length)]
    for i in range(length):
        self.xy_traj[i] = traj[i][:2]

    return self.xy_traj


def check_arrive(self) -> bool:
    '''
        According to self.end_point_threshold=0.01 -> check the robot arrive at the target position or not
    '''
    curren_state = self.get_robot_current_state()
    curren_position = curren_state[0]
    cur_dist = self.compute_euclidean_distance(self.end_point, curren_position)

    if cur_dist < self.end_point_threshold:
        return True