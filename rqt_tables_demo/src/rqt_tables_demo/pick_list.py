"""What the pick list of the panel shows, and the hint when the pose selector gives nothing. No ROS imports:
unit-testable."""


def pick_items(detected_names, enable_id, objects_of_interest):
    """class_id names of the perceived objects when ids are enabled, else the classes of interest"""
    return list(detected_names) if enable_id else list(objects_of_interest)


def empty_hint(robot_ns):
    robot = (robot_ns or '').strip('/') or 'mobipick'
    return (f'the pose selector of robot {robot} holds no objects: perceive with this robot first (its pose '
            'selector must be active while it looks at the table); ids are listed only for perceived objects')


def unreachable_hint(robot_ns, error):
    robot = (robot_ns or '').strip('/') or 'mobipick'
    return f'cannot read the pose selector of robot {robot}: {error}'
