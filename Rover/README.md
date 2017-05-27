# Search and Sample Return

Program a space rover to search for environmental samples in a simulated environment using computer vision techniques, including the three main steps in the robotics process: perception, decision making, and actuation. This project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html).

<img src="https://github.com/LuLi0077/Robotics/blob/master/Rover/images/rover.gif" width="425" height="300">  |  <img src="https://github.com/LuLi0077/Robotics/blob/master/Rover/images/rover-autonomous.gif" width="425" height="300">
:-------------------------:|:-------------------------:


* `Rover_test.ipynb`: test functions for performing the various steps of this project and visualize outputs
* `perception.py`: process and map images
* `decision.py`: determine throttle, brake and steer commands 
* `drive_rover.py`: the main script for autonomous navigation and mapping 
* `supporting_functions.py`: `update_rover() - RoverState()` object gets updated with each new batch of telemetry. The `create_output_images()` function compares Rover.worldmap with the ground truth map and gets converted, along with Rover.vision_image, into base64 strings to send back to the rover.

Screen resolution: 800 x 600
Graphics Quality: Fast






