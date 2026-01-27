#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive, Plane
import tf_transformations
import copy
import math
import threading
import time

class SortingNode(Node):
    def __init__(self):
        super().__init__('sorting_node')
        
        # Subscriptions
        self.subscription = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',
            self.object_callback,
            10)
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10)
        self.current_joints = {}

        # Publishers
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

        # Action Client
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        # Gripper Action Client
        from control_msgs.action import GripperCommand
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        
        self.group_name = "arm"
        self.object_pose = None
        self.is_moving = False

        # Add Floor collision
        threading.Thread(target=self.add_floor_collision).start()

        self.get_logger().info('Sorting Node Started (Static Camera). Waiting for object...')
        
        # DEBUG: Check where the robot thinks it is
        # This tells us if TF is correct (Z should be > 0.8)
        try:
            current_pose = self._action_client._node.get_clock().now() # Just placeholder
            # We can't easily get pose from ActionClient alone without the whole MoveGroupWrapper
            # But we can assume if the node started, we can query TF? 
            # Easier: Just wait for the first pick_and_place to log "Current Pose"?
            pass
        except:
             pass

    def add_floor_collision(self):
        # Floor
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "floor"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [3.0, 3.0, 0.05]
        pose = Pose()
        pose.position.x = 0.0; pose.position.y = 0.0
        pose.position.z = -0.025
        pose.orientation.w = 1.0
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        
        # Table
        # NOTE: We keep the collision table slightly lower (0.75m top) than real visual table (0.8m)
        # to prevent "Touching" errors during grasping.
        co_table = CollisionObject()
        co_table.header.frame_id = "world"
        co_table.id = "table"
        box_t = SolidPrimitive()
        box_t.type = SolidPrimitive.BOX
        box_t.dimensions = [1.0, 0.8, 0.75] # Collision height reduced
        pose_t = Pose()
        pose_t.position.x = 0.0; pose_t.position.y = 0.0
        pose_t.position.z = 0.375 # Center
        pose_t.orientation.w = 1.0
        co_table.primitives.append(box_t)
        co_table.primitive_poses.append(pose_t)
        co_table.operation = CollisionObject.ADD

        # Publish in a loop to ensure RViz receives it
        from visualization_msgs.msg import Marker, MarkerArray
        marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        while rclpy.ok():
            # Collision Objects - RE-ENABLED
            self.collision_pub.publish(co)
            self.collision_pub.publish(co_table)
            
            # Visual Markers (Colored) - MATCH GAZEBO EXACTLY
            ma = MarkerArray()
            
            # Table (Brown) - Visual tweak: Increase scale slightly if user sees a gap
            # Gazebo is 0.8m. Let's make it 0.85 to SWALLOW the visual gap.
            m_table = Marker()
            m_table.header.frame_id = "world"
            m_table.id = 1
            m_table.type = Marker.CUBE
            m_table.action = Marker.ADD
            m_table.pose.position.x = 0.0; m_table.pose.position.y = 0.0; m_table.pose.position.z = 0.425 # Center so Bottom is 0
            m_table.pose.orientation.w = 1.0
            m_table.scale.x = 1.0; m_table.scale.y = 0.8; m_table.scale.z = 0.85 
            m_table.color.r = 0.6; m_table.color.g = 0.4; m_table.color.b = 0.2; m_table.color.a = 1.0
            ma.markers.append(m_table)
            
            # Bin (Red Translucent) - -0.3, 0.3, 0.85
            m_bin = Marker()
            m_bin.header.frame_id = "world"
            m_bin.id = 3
            m_bin.type = Marker.CUBE
            m_bin.action = Marker.ADD
            m_bin.pose.position.x = -0.3; m_bin.pose.position.y = 0.3; m_bin.pose.position.z = 0.85
            m_bin.pose.orientation.w = 1.0
            m_bin.scale.x = 0.2; m_bin.scale.y = 0.2; m_bin.scale.z = 0.1
            m_bin.color.r = 1.0; m_bin.color.g = 0.0; m_bin.color.b = 0.0; m_bin.color.a = 0.8
            ma.markers.append(m_bin)
            
            marker_pub.publish(ma)
            time.sleep(2.0)

    def operate_gripper(self, width):
        from control_msgs.action import GripperCommand
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = 100.0
        future = self._gripper_client.send_goal_async(goal)
        try:
            handle = future.result()
            if not handle or not handle.accepted: return False
            handle.get_result_async().result() # Wait
        except Exception as e:
            self.get_logger().error(f"Gripper error: {e}")
            return False
        return True

    def joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

    def object_callback(self, msg):
        if self.is_moving:
            return
            
        self.object_pose = msg
        self.get_logger().info(f'Object received at: {msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}')
        
        if self.object_pose:
            self.is_moving = True
            threading.Thread(target=self.pick_and_place).start()

    def create_pose_goal(self, pose, frame_id="world"):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        pcm = PositionConstraint()
        pcm.header.frame_id = frame_id
        pcm.link_name = "tool0"
        pcm.target_point_offset.x = 0.0; pcm.target_point_offset.y = 0.0; pcm.target_point_offset.z = 0.0
        pcm.weight = 1.0
        box = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        # Relax position box significantly to ensure it finds A solution
        primitive.dimensions = [0.2, 0.2, 0.2] 
        box.primitives.append(primitive)
        p = Pose(); p.position = pose.position; p.orientation.w = 1.0
        box.primitive_poses.append(p)
        pcm.constraint_region = box
        
        
        ocm = OrientationConstraint()
        ocm.header.frame_id = frame_id
        ocm.link_name = "tool0"
        ocm.orientation = pose.orientation
        # Relaxed Tolerances to help finding a valid IK solution
        ocm.absolute_x_axis_tolerance = 1.0 # ~60 degrees freedom
        ocm.absolute_y_axis_tolerance = 1.0
        ocm.absolute_z_axis_tolerance = 3.14 # Z rotation doesn't matter (Spin is fine)
        ocm.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pcm)
        constraints.orientation_constraints.append(ocm)
        goal_msg.request.goal_constraints = [constraints]
        return goal_msg

    def execute_goal(self, goal_msg):
        self.get_logger().info("Sending Goal...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action Server not available")
            return False
            
        future = self._action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance (since we're in a thread, we can sleep)
        while not future.done():
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal Rejected by Server")
            return False
            
        self.get_logger().info("Goal Accepted. Executing...")
        result_future = goal_handle.get_result_async()
        
        # Wait for execution result
        while not result_future.done():
            time.sleep(0.1)
            
        res = result_future.result().result
        if res.error_code.val == 1:
            return True
        if res.error_code.val == 1:
            return True
        else:
            # Error Mapping
            errors = {
                -1: "FAILURE",
                -2: "PLANNING_FAILED (Collision or Out of Reach)",
                -3: "INVALID_MOTION_PLAN",
                -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                -5: "CONTROL_FAILED",
                -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                -7: "TIMED_OUT",
                -16: "INVALID_GOAL_CONSTRAINTS (Orientation Violation)",
                -21: "NO_IK_SOLUTION",
                -26: "INVALID_GOAL_CONSTRAINTS (Legacy)"
            }
            err_str = errors.get(res.error_code.val, "UNKNOWN_ERROR")
            self.get_logger().error(f"Move Failed: {res.error_code.val} -> {err_str}")
            return False

    def pick_and_place(self):
        try:
            target_pose = self.object_pose.pose
            
            # DEBUG: Print Current Pose to verify TF
            # Since we don't have the full MoveGroupCommander here (using ActionClient directly),
            # we rely on TF or just trial. 
            # But wait! We can't easily get current pose without a subscriber or TF listener.
            # Let's just create a goal relative to the Base Frame? 
            # NO, let's Stick to World.
            
            # Gripper Offset (Tool0 to Fingertip Center)
            # Robotiq 2F-140 is roughly 20cm? Let's assume 0.16m conservatively.
            GRIPPER_OFFSET = 0.16 
            
            # Target Z is ~0.81 (Lego Height)
            # Flange Z should be 0.81 + GRIPPER_OFFSET = 0.97
            
            grasp_z = target_pose.position.z + GRIPPER_OFFSET
            pre_grasp_z = grasp_z + 0.10

            # ULTIMATE FIX: Plan in 'base_link' frame.
            # This ignores whether the robot thinks it is on the floor or table.
            # We just tell it: "Go to X,Y relative to your base, and Z just above your base."
            
            # Target Z relative to Base:
            # Lego is on the same surface as base (Table).
            # To ensure we don't "Air Grab", aim LOWER.
            # 5mm above table surface.
            LEGO_Z_BASE = 0.005
            
            grasp_z = LEGO_Z_BASE + GRIPPER_OFFSET
            pre_grasp_z = grasp_z + 0.10
            
            self.get_logger().info(f"Planning Pick relative to BASE_LINK at Z: {grasp_z:.3f}")

            # 0. Open Gripper
            self.operate_gripper(0.0)
            
            # Orientation: Down with pi/2 Z rotation
            q = tf_transformations.quaternion_from_euler(3.14159, 0, 1.5708) 
            
            # Pre-Grasp
            pre_grasp = copy.deepcopy(target_pose)
            
            # OVERRIDE FRAME TO BASE_LINK
            # ULTIMATE FIX: Plan in 'base_link' frame.
            # We assume World X/Y align with Base X/Y (since Yaw=0)
            # So we use the same X/Y from detection.
            # But we force Z to be relative to base.
            
            # Pre-Grasp
            pre_grasp = copy.deepcopy(target_pose)
            pre_grasp.position.z = pre_grasp_z
            pre_grasp.orientation.x = q[0]; pre_grasp.orientation.y = q[1]; pre_grasp.orientation.z = q[2]; pre_grasp.orientation.w = q[3]

            self.get_logger().info("Moving to Pre-Grasp")
            # PASS base_link frame!
            if not self.execute_goal(self.create_pose_goal(pre_grasp, "base_link")): 
                self.is_moving = False; return

            # Grasp
            grasp = copy.deepcopy(pre_grasp)
            grasp.position.z = grasp_z
            self.get_logger().info("Moving to Grasp")
            if not self.execute_goal(self.create_pose_goal(grasp, "base_link")): 
                self.is_moving = False; return

            # Close
            self.get_logger().info("Closing Gripper...")
            self.operate_gripper(0.8) # TIGHTER GRIP
            time.sleep(2.0) 

            # Lift
            self.get_logger().info("Lifting")
            if not self.execute_goal(self.create_pose_goal(pre_grasp, "base_link")): 
                self.is_moving = False; return

            # Place (On Table at X=0.3, Y=0.0)
            # Place logic: X=0.3 is relative to Base now.
            place = copy.deepcopy(pre_grasp)
            place.position.x = 0.3; place.position.y = 0.0 
            place.position.z = LEGO_Z_BASE + GRIPPER_OFFSET + 0.05 # Drop slightly above table
            
            self.get_logger().info("Moving to Place")
            if not self.execute_goal(self.create_pose_goal(place, "base_link")): 
                self.is_moving = False; return
            
            # Open
            self.operate_gripper(0.0)
            time.sleep(1.0)
            
            # Go Home (Up)
            self.get_logger().info("Going Home")
            home_pose = copy.deepcopy(place)
            home_pose.position.z = 0.3 # 30cm above base
            if not self.execute_goal(self.create_pose_goal(home_pose, "base_link")): 
                 self.is_moving = False; return
            
            self.get_logger().info("SEQUENCE COMPLETE")

        except Exception as e:
            self.get_logger().error(f"Sequence Error: {e}")
        finally:
            self.is_moving = False

def main(args=None):
    rclpy.init(args=args)
    node = SortingNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
