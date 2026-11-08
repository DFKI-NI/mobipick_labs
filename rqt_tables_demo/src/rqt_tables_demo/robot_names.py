"""Names that belong to one robot of a multi-robot sim: the robot namespace chooses the pose selector,
gripper services, action servers, map frame and Gazebo model. No ROS imports: unit-testable."""
import re

DEFAULT_ROBOT = 'mobipick'   # robot 1: its pose selector runs in the global namespace, its map frame is 'map'
SELECTED_ROBOT_TOPIC = '/selected_robot'   # std_msgs/String, latched: the robot every GUI of the sim acts on
REQUEST_TOPIC = '/selected_robot/request'   # std_msgs/String: a switch asks the selected-robot owner node for a robot


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


_NAV_STATUS = re.compile(r'^/([^/]+)/move_base/status$')


def _number(namespace):
    match = re.search(r'(\d+)$', namespace)
    return int(match.group(1)) if match else 1


def robots_from_topics(topic_names, current=DEFAULT_ROBOT):
    """robot namespaces with a running move_base (from the published topic names), natural order, plus `current`
    (the same list as the sim window's robot selector)"""
    robots = {normalize(current)}
    for topic in topic_names:
        match = _NAV_STATUS.match(topic)
        if match:
            robots.add(match.group(1))
    return sorted(robots, key=lambda robot: (_number(robot), robot))


def owner_running(system_state):
    """True when a node listens on the request topic (`system_state` as rosgraph.Master.getSystemState returns it):
    then the sim's selected-robot owner node is the only writer of /selected_robot and a switch asks it"""
    _publishers, subscribers, _services = system_state
    return any(topic == REQUEST_TOPIC and nodes for topic, nodes in subscribers)


def to_follow(current, heard):
    """the robot to switch to when /selected_robot carries `heard`, None when it is the robot already chosen. Without
    the owner node every GUI that publishes the topic adopts a choice made elsewhere and publishes it again, so the
    latched message of every publisher names the same robot and a window opened later reads that robot from all."""
    heard = normalize(heard)
    return None if heard == normalize(current) else heard
