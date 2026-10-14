from pathlib import Path

import cv2


dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)

output_directory = Path(__file__).resolve().parents[1] / 'textures'
output_directory.mkdir(parents=True, exist_ok=True)

number_of_tags = 24
marker_pixels = 800
margin_pixels = 100

for tag_id in range(number_of_tags):
    marker = cv2.aruco.drawMarker(
        dictionary,
        tag_id,
        marker_pixels,
    )

    texture = cv2.copyMakeBorder(
        marker,
        margin_pixels, margin_pixels,
        margin_pixels, margin_pixels,
        cv2.BORDER_CONSTANT,
        value=255,
    )

    filename = output_directory / f'tag36_11_{tag_id:05d}.png'

    if not cv2.imwrite(str(filename), texture):
        raise RuntimeError(f'Could not save {filename}')

print(f'Generated {number_of_tags} textures in {output_directory}')