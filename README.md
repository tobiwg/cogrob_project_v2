# cogrob_project

## Table of Contents

- [Description](#description)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Description

### Alligator Egg Collection Robot
In commercial alligator farms, the harvesting of alligator eggs is a costly and dangerous operation. In existing solutions, human operators are typically suspended in the air and descended into the desired location using helicopters. The present project aims to develop a cognitive robot capable of harvesting alligator eggs.

This repository contains code to control a robot designed to collect alligator eggs. The robot is equipped with a custom end effector designed to gently pick up and transport alligator eggs without damaging them.

The code in this repository is written in Python and runs on the Robot Operating System (ROS) framework. It includes scripts to control the robot's movement, operate the end effector, and perform image recognition to detect the location of alligator eggs.

Please note that the collection of alligator eggs is illegal in many areas and can have serious consequences for both the individual and the environment. The use of this code for illegal or unethical purposes is strictly prohibited. The owners and contributors of this repository assume no liability for any damages or legal repercussions resulting from the use of this code.

This code was developed as part of a research project aimed at studying the impact of alligator egg collection on alligator populations. The robot is intended to provide a safer and more efficient method of collecting alligator eggs for research purposes, without putting human researchers or alligator populations at risk.

If you have any questions or concerns regarding the use of this code, please refer to the warning message in the repository or contact the repository owners for more information.
## Installation

To use this package, you must first install ROS1 (noetic) and the MoveIt framework. Refer to the [MoveIt installation guide](https://moveit.ros.org/install/) for instructions.

After installation, download this package and place the `startup` folder under `ws_moveit/src`. 

Then, take the two files in the `replace` folder and place them in `ws_moveit/src/panda_moveit_config/launch`, replacing the original files.

Next, navigate to the `ws_moveit/src/startup/scripts` folder and run the following three commands:
```
chmod +x vision.py
chmod +x move_group_python_interface.py
chmod +x display_images_test.py
```
Return to the `ws_moveit` folder and run the following command:
```
source devel/setup.bash
```

Finally, run the following command to launch the package:
```
rosrun startup start.launch
```
## Usage

### WARNING: Alligator Egg Collection Robot

This repository contains code to control a robot for the purpose of picking up alligator eggs. Please be aware that the collection of alligator eggs is illegal in many areas and can have serious consequences for both the individual and the environment.

The use of this code for illegal or unethical purposes is strictly prohibited. The owners and contributors of this repository assume no liability for any damages or legal repercussions resulting from the use of this code.

By accessing and using the code in this repository, you acknowledge that you are solely responsible for ensuring that your actions comply with all applicable laws and regulations.

If you have any questions or concerns regarding the use of this code, please contact the repository owners or seek legal advice before proceeding.


## Contributing

We welcome contributions from anyone who is interested in improving this project. Here are some ways you can contribute:

Report bugs and issues: If you encounter any bugs or issues while using this code, please let us know by opening an issue on GitHub. Please include as much detail as possible, including the steps to reproduce the issue and any error messages you received.

Suggest new features: If you have an idea for a new feature that would improve this project, feel free to open an issue on GitHub and let us know. We will consider all suggestions, but please keep in mind that not all features may be feasible or aligned with the goals of the project.

Submit pull requests: If you would like to contribute code to this project, you can do so by submitting a pull request on GitHub. Before submitting a pull request, please make sure that your code is properly tested and documented, and that it adheres to the project's coding standards.

Provide feedback: If you have any other feedback or suggestions for how we can improve this project, please let us know by opening an issue on GitHub. We value feedback from our users and are always looking for ways to make this project better.

## License

Mention the license under which the project is released. If there is no license yet, indicate that it is currently under development.
