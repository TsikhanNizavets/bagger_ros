import numpy as np
from scipy.spatial.transform import Rotation

def marker_corners(marker_size):
    half = marker_size / 2

    return np.array([
        [-half, half, 0],
        [half, half, 0],
        [half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float32)

def generate_cube_tags(cube_side_length, offset=0.0002, first_id=0):
    texture_size = cube_side_length / 5
    marker_size = 0.8 * texture_size

    local_marker_corners = marker_corners(marker_size)

    face_totations = [
        (np.pi / 2, 0, np.pi / 2),
        (np.pi / 2, 0, -np.pi / 2),
        (np.pi / 2, 0, np.pi),
        (np.pi / 2, 0, 0),
        (0, 0, 0),
        (0, np.pi, 0),
    ]

    tags = []

    for face_index, rpy in enumerate(face_totations):
        rotation = Rotation.from_euler('xyz', rpy)

        for row in range(2):
            for column in range(2):
                local_position = [
                    0.4 * cube_side_length * (2 * column - 1),
                    0.4 * cube_side_length * (1 - 2 * row),
                    cube_side_length / 2 + offset,
                ]

                position = rotation.apply(local_position)
                corners = rotation.apply(local_marker_corners) + position

                tags.append({
                    'id': first_id + 4 * face_index + 2 * row + column,
                    'position': position,
                    'rpy': rpy,
                    'texture_size': texture_size,
                    'corners': corners,
                })

    return tags

if __name__ == '__main__':
    tags = generate_cube_tags(0.1)

    print('Number of tags:', len(tags))
    print('First tag ID:', tags[0]['id'])
    print('First tag position:', tags[0]['position'])
    print('First tag corners:\n', tags[0]['corners'])
