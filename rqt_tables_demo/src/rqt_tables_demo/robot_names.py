"""Names that belong to one robot of a multi-robot sim: the robot namespace chooses the pose selector,
gripper services, action servers, map frame and Gazebo model. No ROS imports: unit-testable."""

DEFAULT_ROBOT = 'mobipick'   # robot 1: its pose selector runs in the global namespace, its map frame is 'map'
SELECTED_ROBOT_TOPIC = '/selected_robot'   # std_msgs/String, latched: the robot chosen in the sim GUI's one selector


def normalize(namespace):
    return (namespace or '').strip('/') or DEFAULT_ROBOT


def prefix(namespace):
    return '/' + normalize(namespace)


def map_frame(namespace):
    namespace = normalize(namespace)
    return 'map' if namespace == DEFAULT_ROBOT else namespace + '/map'


def pose_selector_services(namespace):
    base = '' if normalize(namespace) == DEFAULT_ROBOT else prefix(namespace)
    return {
        'activate': base + '/pick_pose_selector_node/pose_selector_activate',
        'class_query': base + '/pick_pose_selector_node/pose_selector_class_query',
        'get_all': base + '/pick_pose_selector_node/pose_selector_get_all',
        'clear': base + '/pick_pose_selector_node/pose_selector_clear',
    }


def gripper_services(namespace):
    return {
        'open': prefix(namespace) + '/pose_teacher/open_gripper',
        'close': prefix(namespace) + '/pose_teacher/close_gripper',
    }


def action_server(namespace, name):
    return prefix(namespace) + '/' + name
