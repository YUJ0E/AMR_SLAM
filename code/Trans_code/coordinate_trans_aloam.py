import numpy as np

def parse_tum_format(filename):
    poses = []
    with open(filename, 'r') as file:
        for line in file:
            if line.strip():  # Skip empty lines
                parts = line.strip().split()
                timestamp = float(parts[0])
                tx, ty, tz = map(float, parts[1:4])
                qx, qy, qz, qw = map(float, parts[4:])
                poses.append((timestamp, tx, ty, tz, qx, qy, qz, qw))
    return np.array(poses)

measured_poses = parse_tum_format('aloam_tum.txt')  # velo
true_poses = parse_tum_format('have_fun_tum.txt')  # camera


def velo_to_camera(camera_coords, rotation_matrix, translation_vector):
    return np.dot(rotation_matrix, camera_coords) + translation_vector


# Example coordinates in camera coordinate system (x, y, z)
velo_coords = measured_poses[:, 1:4]

# rotation matrix (3x3)
rotation_matrix = np.array([[0, -1, 0],
                            [0, 0, 1],
                            [1, 0, 0]])

# translation vector (3x1)
translation_vector = np.array([0, -0.08, 0.27])

camera_poses = []


for i in range(len(velo_coords)):
    if i == 0:
        camera_coords = np.array([0, 0, 0])
    else:
    # translation vector representing no translation
        camera_coords = velo_to_camera(velo_coords[i], rotation_matrix, translation_vector)
    camera_poses.append(camera_coords)


measured_camera_poses = np.hstack((true_poses[:, 0].reshape(-1, 1), camera_poses, true_poses[:, 4:8].reshape(-1, 4)))


np.savetxt('aloam_tum_2.txt', measured_camera_poses)

print(camera_poses[2000])
print(true_poses[2000, 1:4])
