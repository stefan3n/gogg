# Code organization

To improve modularity and maintainability, we have divided the codebase into **three** distinct packages based on their functionality:

* **[JetsonControl](JetsonControl/JetsonControl.md)**: Contains the code that will run on the Jetson device, handling higher-level control and coordination between the modules.
* **[ArmControl](ArmControl/ArmControl.md)**: Contains the code that will be uploaded to the Arduino responsible for controlling the robotic arm's logic.
* **[CarControl](CarControl/CarControl.md)**: Contains the code that will be uploaded to the Arduino responsible for managing the car's movement.
