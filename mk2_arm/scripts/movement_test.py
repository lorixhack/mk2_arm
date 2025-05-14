#!/usr/bin/env python3

import time

# ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanningComponent
)

from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint

from geometry_msgs.msg import PoseStamped

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

def create_pose_and_constraints(pose_goal):
    constraints = Constraints()
    pos_constraint = PositionConstraint()
    pos_constraint.header.frame_id = "base_link"
    pos_constraint.link_name = "roll_6"
    pos_constraint.constraint_region.primitives.append(
        SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.03, 0.03, 0.03])  # Box di ±0.1m
    )
    pos_constraint.constraint_region.primitive_poses.append(pose_goal)
    constraints.position_constraints.append(pos_constraint)

    # Tolleranza di orientamento (±0.087 rad ≈ 5°)
    orient_constraint = OrientationConstraint()
    orient_constraint.header.frame_id = "base_link"
    orient_constraint.link_name = "roll_6"
    orient_constraint.orientation = pose_goal.orientation
    orient_constraint.absolute_x_axis_tolerance = 0.03
    orient_constraint.absolute_y_axis_tolerance = 0.03
    orient_constraint.absolute_z_axis_tolerance = 0.03
    constraints.orientation_constraints.append(orient_constraint)
    return constraints


def main():
    # MoveItPy setup
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")
    
    # instantiate MoveItPy instance
    mk2 = MoveItPy(node_name = "moveit_py")
    mk2_arm = mk2.get_planning_component("mk2_arm")
    logger.info("MoveItPy instance created")
    mk2_arm.set_start_state_to_current_state()
    state = mk2_arm.get_start_state()
    start_pose = state.get_pose("roll_6")

    while True:
        time.sleep(10)

        # Set the pose with RobotState object
        # robot_model = mk2.get_robot_model()
        # robot_state = RobotState(robot_model)

        # robot_state.set_to_random_positions()


        # mk2_arm.set_goal_state(robot_state=robot_state)

        # mk2_arm.set_start_state_to_current_state()

        # logger.info("Set goal state to the initialized robot state")
        # mk2_arm.set_goal_state(robot_state=robot_state)
        
        # plan_and_execute(mk2, mk2_arm, logger, sleep_time=0.1)

        ## Use PoseStamped() message

        # Test to get Pose()
        # logger.info(f"Positions: x={pose_goal.pose.position.x} y={pose_goal.pose.position.y} z={pose_goal.pose.position.z}")
        # logger.info(f"Orientations: x={pose_goal.pose.orientation.x} y={pose_goal.pose.orientation.y} z={pose_goal.pose.orientation.z} w={pose_goal.pose.orientation.w}")


        # x-axis
        mk2_arm.set_start_state_to_current_state()
        state = mk2_arm.get_start_state()
        pose_goal = start_pose
        pose_goal.position.x = 0.12
        constraints = create_pose_and_constraints(pose_goal)
    
    
        mk2_arm.set_goal_state(motion_plan_constraints=[constraints])

        plan_and_execute(mk2, mk2_arm, logger, sleep_time=3.0)

        mk2_arm.set_start_state_to_current_state()
        state = mk2_arm.get_start_state()
        pose_goal = start_pose
        pose_goal.position.x = -0.12

        constraints = create_pose_and_constraints(pose_goal)

        mk2_arm.set_goal_state(motion_plan_constraints=[constraints])

        plan_and_execute(mk2, mk2_arm, logger, sleep_time=3.0)

        # Reset
        mk2_arm.set_start_state_to_current_state()
        constraints = create_pose_and_constraints(start_pose)
        mk2_arm.set_goal_state(motion_plan_constraints=[constraints])
        plan_and_execute(mk2, mk2_arm, logger, sleep_time=3.0)

        #y-axis
        mk2_arm.set_start_state_to_current_state()
        state = mk2_arm.get_start_state()
        pose_goal = start_pose
        pose_goal.position.y= 0.1
        constraints = create_pose_and_constraints(pose_goal)

    
        mk2_arm.set_goal_state(motion_plan_constraints=[constraints])

        plan_and_execute(mk2, mk2_arm, logger, sleep_time=3.0)

        mk2_arm.set_start_state_to_current_state()
        state = mk2_arm.get_start_state()
        pose_goal = start_pose
        pose_goal.position.y = -0.1

        constraints = create_pose_and_constraints(pose_goal)

        mk2_arm.set_goal_state(motion_plan_constraints=[constraints])

        plan_and_execute(mk2, mk2_arm, logger, sleep_time=3.0)

        # Reset
        mk2_arm.set_start_state_to_current_state()
        constraints = create_pose_and_constraints(start_pose)
        mk2_arm.set_goal_state(motion_plan_constraints=[constraints])
        plan_and_execute(mk2, mk2_arm, logger, sleep_time=3.0)

        #z-axis
        mk2_arm.set_start_state_to_current_state()
        state = mk2_arm.get_start_state()
        pose_goal = start_pose
        pose_goal.position.z= 0.4
        constraints = create_pose_and_constraints(pose_goal)
    
        mk2_arm.set_goal_state(motion_plan_constraints=[constraints])

        plan_and_execute(mk2, mk2_arm, logger, sleep_time=3.0)

        mk2_arm.set_start_state_to_current_state()
        state = mk2_arm.get_start_state()
        pose_goal = start_pose
        pose_goal.position.z = -0.4

        constraints = create_pose_and_constraints(pose_goal)

        mk2_arm.set_goal_state(motion_plan_constraints=[constraints])

        plan_and_execute(mk2, mk2_arm, logger, sleep_time=3.0)


if __name__ == "__main__":
    main() 

