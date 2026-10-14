import argparse
from pathlib import Path

import yaml

from bagger_perception.cube_geometry import generate_cube_tags
from bagger_perception.cube_config import load_cube_settings

def create_cube_board(cube_id, cube_side_length, offset):
    tags = generate_cube_tags(
        cube_side_length,
        offset=offset,
        first_id=(cube_id - 1) * 24,
    )

    markers = []

    for tag in tags:
        markers.append({
            'id': tag['id'],
            'corners': tag['corners'].round(9).tolist(),
        })

    return {
        'name': f'cube_{cube_id}',
        'markers': markers,
    }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('__file__').resolve().parents[1] / 'config' / 'cube_boards_3d.yaml')
    args = parser.parse_args()

    settings = load_cube_settings()

    boards = [
        create_cube_board(
            cube_id=1,
            cube_side_length=settings['cube_side_length'],
            offset=settings['offset'],
        ),
    ]

    with args.output.open('w') as file:
        yaml.safe_dump(boards, file, sort_keys=False)

    print(f'Saved {len(boards)} cube boards to {args.output}')

if __name__ == '__main__':
    main()
