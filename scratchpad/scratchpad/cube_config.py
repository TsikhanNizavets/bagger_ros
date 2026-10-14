from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

def load_cube_settings():
    package_directory = Path(
        get_package_share_directory('scratchpad')
    )
    config_path = package_directory / 'config' / 'cube_settings.yaml'

    with config_path.open() as file:
        return yaml.safe_load(file)
