from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import yaml

def load_cube_boards(path, dictionary):
    with Path(path).open() as file:
        descriptions = yaml.safe_load(file)

    boards = {}

    for description in descriptions:
        markers = description['markers']

        corners = np.array(
            [marker['corners'] for marker in markers],
            dtype=np.float32,
        )

        ids = np.array(
            [marker['id'] for marker in markers],
            dtype=np.int32,
        )

        board = cv2.aruco.Board_create(
            corners,
            dictionary,
            ids,
        )

        boards[description['name']] = board

    return boards

def main():
    package_directory = Path(
        get_package_share_directory('scratchpad')
    )
    path = package_directory / 'config' / 'cube_boards_3d.yaml'

    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )

    boards = load_cube_boards(path, dictionary)

    for name, board in boards.items():
        print(f'{name}: loaded {len(board.ids)} markers')

if __name__ == '__main__':
    main()
