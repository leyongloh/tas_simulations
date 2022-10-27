# Highway Generator

Scripts used to generate Gazebo world.

## Highway Parameters

For the highway dimensions, we adapted the scheme RQ29,5 from : [Richtlinien für die Anlage von Straßen – Querschnitt](https://de.wikipedia.org/wiki/Richtlinien_f%C3%BCr_die_Anlage_von_Stra%C3%9Fen_%E2%80%93_Querschnitt). 
As we want to experiment with overtaking in the oncoming lane in case of fully blocked lanes, we excluded the safety space between the opposite roadways. By comparing the width of the tas_car with the width of a real car, we determined a scale factor of 0.25.

![Highway ground image](img/highway.png)

## Usage

The world generation is split up into two scripts: `generate_highway.m` and `gazebo_spawn_objects.py`.

### Generate Highway


- Open the file `generate_highway.m` in MATLAB2021b.

- Set the lane parameters for your desired highway configuration.

- Use the plot option, to check if all lanes are set correctly.

- If true, use the `export_figure` option to generate the highway image file, also enable the `plot_grid` option.

- Copy the exported figure to `/project_files/lane_planner_environment/gazebo/models/highway/materials/textures`

- Launch one of our demos and check if the gazebo grid is aligned with the image's grid, if not adapt the scale of the highway object in the world file.

- If successfull, redo the above steps without the grid.

- For the Gazebo objects use `generate_highway.m`, set object parameters and check them with enabled `plot_obj` option.

- Please make sure that the model names in the export part fit the gazebo object name.

- If the objects positions are as desired, run script again with enabled `export_obj`.

- If successfull, resume with the below step.

### Spawn Gazebo objects

- To spawn your defined objects in the Gazebo world, launch a demo with the object-free gazebo world.

- launch when demo is loaded:

```shell
rosrun highway_generator spawn_gazebo_objects.py
```
- If all objects are spawned correctly, delete all unwanted objects (including the TAS car).

- Save the Gazebo world to your desired directory and adapt launch files accordingly.

