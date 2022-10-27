# LIDAR_simulation:
This folder holds the files used to visualize the different approaches that are implemented in the [LaserChecker.py](../../lane_planner/scripts/LaserChecker.py).
It mainly focuses on the transformations required to implement the zone filter.

## Curved_Zones
To Jupyter Notebooks were developed to visualize the calculation of the front zones used by the LaserChecker.py. It visualizes how the zones in the front of the car are calculated.

* Jupyter Notebook: [Curved_Zones.ipynb](Curved_Zones.ipynb)
* Jupyter Notebook: [Curved_Zones.ipynb](Square_Zones.ipynb)

## Zone_Analysis
This notebook can be used for visualization and verification of the zone mapping. It reads in an example file that can be exported by the LaserChecker.py if the debug option is turned on in its config file. 
It then uses the actual code from the node to analyze it. The results are then visualized using matplotlib. The user can change the mask and the shape of the front zones.
* Jupyter Notebook: [Zone_Analysis.ipynb](Zone_Analysis.ipynb)

## Lidar
This script was developed as a zone filter that used the beams returned from the laser scanner instead of the coordinates of the detected points. This led to a more complicated simulation that can be seen visualized in this script:
* Matlab: [lidar.m](lidar.m)

The method was not implemented in the final [LaserChecker.py](../../lane_planner/scripts/LaserChecker.py).
