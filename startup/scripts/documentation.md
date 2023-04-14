##add_egg.py
The script uses the MoveIt! Python API to interface with MoveIt! and adds an "egg" object to the planning scene.

The MoveGroupPythonInterfaceTutorial class is defined and the necessary modules and packages are imported. In the constructor, the moveit_commander and rospy nodes are initialized, and instances of RobotCommander, PlanningSceneInterface, and MoveGroupCommander are created. The wait_for_state_update and add_egg methods are defined in the class.

The wait_for_state_update method ensures that the collision updates are received. It waits until the changes are reflected in the get_attached_objects() and get_known_object_names() lists.

The add_egg method adds an object to the planning scene. It creates a CollisionObject message and sets its properties such as its position, size, and name. It adds the object to the planning scene using the add_object() method of the PlanningSceneInterface class. It then waits for the changes to be reflected in the planning scene using the wait_for_state_update() method. If the changes are not made within a specified timeout, the method returns False.
