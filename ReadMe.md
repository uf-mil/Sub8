Estimate Drag Coefficients
==========================

This program calculates the linear drag coeffiecnt in all six directions that the sub can move.

# Usage

run

	roslaunch sub8_launch dynamics_simulator.launch

In a new window, run

	 rosnode kill /rise_6dof
to kill the controller.

run

	rosrun sub8_system_id  estimate_drag_coefficients.py

The program should then start applying forces to the sub and calculating the drag using its max velocity.