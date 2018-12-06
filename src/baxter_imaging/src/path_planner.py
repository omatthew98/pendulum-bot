import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class PathPlanner(object):

    def __init__(self, group_name):

        rospy.on_shutdown(self.shutdown)
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject)
        self._group = moveit_commander.MoveGroupCommander(group_name)
        self._group.set_planning_time(5)
        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

        rospy.sleep(0.5)

    def shutdown(self):

        self._group = None
    
    def plan_to_pose(self, target, orientation_constraints):

        self._group.set_pose_target(target)
        self._group.set_start_state_to_current_state()

        constraints = Constraints()
        constraints.orientation_constraints = orientation_constraints
        self._group.set_path_constraints(constraints)

        plan = self._group.plan()

        return plan

    def execute_plan(self, plan):

        return self._group.execute(plan, True)

    def add_box_obstacle(self, size, name, pose):

        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size
        
        co.primitives = [box]
        co.primitive_poses = [pose.pose]

        self._planning_scene_publisher.publish(co)

        