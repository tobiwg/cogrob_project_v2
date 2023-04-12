# cogrob_project


This is a readme file for the project named "Project Name". 

## Table of Contents

- [Description](#description)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Description

Provide a brief description of the project here.

## Installation

To use this package, you must first install ROS1 (noetic) and the MoveIt framework. Refer to the [MoveIt installation guide](https://moveit.ros.org/install/) for instructions.

After installation, download this package and place the startup package under `ws_moveit/src`. Then, take the two files in the `replace` folder and place them in `ws_moveit/src/panda_moveit_config/launch`, replacing the original files.

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

Explain how to use the project after installation. Provide clear instructions and examples if necessary.

## Contributing

Explain how others can contribute to the project. Provide guidelines for contributing, such as style guides, code reviews, and pull request procedures.

## License

Mention the license under which the project is released. If there is no license yet, indicate that it is currently under development.
