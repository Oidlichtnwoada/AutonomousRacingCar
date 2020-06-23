# ---------------------------------------------------------------------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@gmail.com)
# SPDX-License-Identifier: GPL-3.0-or-later
# ---------------------------------------------------------------------------------------------------------------------

import os
import argparse
import yaml
import numpy as np
import cmapy
from skimage import io, morphology, img_as_ubyte
from scipy import ndimage

class MapPreprocessor:
    def __init__(self,
                 input_yaml_path,
                 output_npz_path):

        with open(input_yaml_path) as file:
            self.map_properties = yaml.safe_load(file)

        self.output_npz_path = output_npz_path
        self.input_directory = os.path.dirname(input_yaml_path)
        image_filename = os.path.join(self.input_directory, self.map_properties['image'])
        self.image = io.imread(image_filename, as_gray=True)
        self.image_center = (self.image.shape + np.array(self.map_properties['origin'])[0:2] /
                             self.map_properties['resolution']).astype(np.int)
        self.binary_image = self.image > (255 * self.map_properties['occupied_thresh'])

    def run(self):

        driven_distances = np.zeros_like(self.binary_image, np.float)

        mask = np.zeros_like(self.binary_image, np.bool)
        finish_line = np.zeros_like(self.binary_image, np.bool)
        mask[self.image_center[0],self.image_center[1]] = True

        def block_finish_line(row_start, col, row_step, blocked):
            row = row_start
            while True:
                if self.binary_image[row, col]:
                    self.binary_image[row, col] = False
                    blocked[row, col] = True
                    row = row + row_step
                else:
                    break

        block_finish_line(self.image_center[0], self.image_center[1]-1, 1, finish_line)
        block_finish_line(self.image_center[0]-1, self.image_center[1]-1, -1, finish_line)

        print("Please wait...")

        distance = 0.0
        while True:
            distance = distance + 1.0
            dilated_mask = morphology.binary_dilation(mask, selem=morphology.selem.square(width=3, dtype=np.bool))
            new_pixels = np.logical_and(self.binary_image, np.logical_xor(mask, dilated_mask))
            x,y = np.nonzero(new_pixels)
            if len(x) == 0:
                break
            driven_distances[x,y] = distance
            mask = np.logical_or(mask, new_pixels)
            if np.mod(distance,200) == 0:
                print("distance: {}".format(distance))

        driven_distances = driven_distances * float(self.map_properties['resolution'])
        normalized_driven_distances = driven_distances / np.amax(driven_distances.flatten())
        drivable_area = np.logical_or(mask, finish_line)

        distances_from_nearest_obstacle = ndimage.distance_transform_edt(drivable_area, return_distances=True, return_indices=False)
        distances_from_nearest_obstacle = distances_from_nearest_obstacle.astype(float) * float(self.map_properties['resolution'])
        normalized_distances_from_nearest_obstacle = distances_from_nearest_obstacle / np.amax(distances_from_nearest_obstacle.flatten())

        np.savez_compressed(self.output_npz_path,
                            driven_distances=driven_distances,
                            normalized_driven_distances=normalized_driven_distances,
                            drivable_area=drivable_area,
                            distances_from_nearest_obstacle=distances_from_nearest_obstacle)

        filename_root, _ = os.path.splitext(self.output_npz_path)

        io.imsave(filename_root + '.driven_distances.png',
                  img_as_ubyte(normalized_driven_distances))

        io.imsave(filename_root + '.drivable_area.png',
                  img_as_ubyte(drivable_area))

        io.imsave(filename_root + '.driven_distances.colorized.png',
                  cmapy.colorize(img_as_ubyte(normalized_driven_distances), 'cividis', rgb_order=True))

        io.imsave(filename_root + '.distances_from_nearest_obstacle.png',
                  img_as_ubyte(normalized_distances_from_nearest_obstacle))

        io.imsave(filename_root + '.distances_from_nearest_obstacle.colorized.png',
                  cmapy.colorize(img_as_ubyte(normalized_distances_from_nearest_obstacle), 'cividis', rgb_order=True))

# ======================================================================================================================
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--input', help='filename of the map description (.yaml)')
    parser.add_argument('--output', help='filename of the output archive (.npz)')
    args = parser.parse_args()
    app = MapPreprocessor(input_yaml_path=args.input,
                          output_npz_path=args.output)
    app.run()
