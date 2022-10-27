import cv2
import numpy as np
import argparse

'''
Author: Johann Erhard

Create a path trajectory based on map image. Select origin and known location to 
calibrate all following mouse clicks for the correct map coordinates. The selected path
is then printed in map coordinates.
'''


click_counter = 0
origin = np.array([0.0, 0.0], dtype=float)
scale = np.array([0.0, 0.0], dtype=float)


def click_event(event, x, y, flags, params):
    '''
    Captures mouse click event on the image
    '''

    global click_counter, origin, scale, map

    if event == cv2.EVENT_LBUTTONDOWN:

        location = np.array([x, y], dtype=float)

        # Initialize origin on first click
        if click_counter == 0:

            origin = np.copy(location)
            map = cv2.circle(map, (x, y), radius=5,
                             color=(0, 255, 0), thickness=3)
            cv2.imshow('map', map)

            print('Select known location')

        # Calculate map scale on second click
        if click_counter == 1:

            map = cv2.circle(map, (x, y), radius=5,
                             color=(0, 255, 0), thickness=3)
            cv2.imshow('map', map)

            reference_location = input('Input known coordinates \'<x>, <y>\': ')
            reference_str = reference_location.split(',')
            reference = np.array([float(reference_str[0]), float(reference_str[1])], dtype=float)

            delta = location - origin
            scale = np.divide(reference, delta)

            print('Select path')

        # Calculate path trajectory from third click onwards
        if click_counter > 1:

            map = cv2.circle(map, (x, y), radius=5,
                             color=(255, 0, 0), thickness=3)
            cv2.imshow('map', map)

            path_location = np.multiply((location - origin), scale)
            print(f'[{path_location[0]}, {path_location[1]}]')

        click_counter += 1


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-map', type=str, required=True, help='Path to map image')
    args = parser.parse_args()

    map = cv2.imread(args.map)

    cv2.imshow('map', map)

    print('Select origin')

    cv2.setMouseCallback('map', click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
