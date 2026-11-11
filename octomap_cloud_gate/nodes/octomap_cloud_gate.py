#!/usr/bin/env python3
'''
Forward a depth point cloud only while the arm is at rest.

The wrist camera's depth cloud shows the world a little later than its stamp (the render runs in the sensor thread), so
while the arm moves, slivers of the arm itself pass MoveIt's self-filter, which masks the robot as it was AT THE STAMP,
and end up as phantom voxels in the octomap. This node sits between the camera and MoveIt's PointCloudOctomapUpdater and
drops every cloud that was taken, or arrives, less than ~rest_time seconds after the arm last moved. The octomap keeps
the map of the last rest for collision checking. Perception only, nothing is sent to the robot.

  rosrun octomap_cloud_gate octomap_cloud_gate.py   (see launch/octomap_cloud_gate.launch)

Parameters: ~cloud_in, ~cloud_out (topics), ~joint_states (topic), ~joint_suffixes (the arm joints, matched with
endswith so a tf prefix does not matter), ~rest_speed (rad/s, 0.05), ~rest_time (s, 0.3), ~stale_time (s, 1.0: joint
states older than this count as moving, so the gate fails closed).

Every 30 s it logs how many clouds passed and were dropped and the clouds' lag (arrival minus stamp, median and
max over those 30 s): while the base drives, a lag L shifts the whole cloud by speed * L, which the gate does not
catch (it watches the arm only).
'''
import rospy
from sensor_msgs.msg import JointState, PointCloud2

ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
              'wrist_3_joint']


class RestDetector:
    '''Tracks the last time (seconds) the arm joints moved faster than rest_speed. Speeds come from the message velocity
    when it has one per joint, else from the position difference to the previous message.'''

    def __init__(self, joint_suffixes=ARM_JOINTS, rest_speed=0.05, rest_time=0.3, stale_time=1.0):
        self.joint_suffixes = tuple(joint_suffixes)
        self.rest_speed = rest_speed
        self.rest_time = rest_time
        self.stale_time = stale_time
        self.last_moving = None   # stamp of the newest joint state that moved; None = never seen a state
        self.last_state = None    # stamp of the newest joint state
        self._previous = None     # (stamp, {joint: position})

    def update(self, stamp, names, positions, velocities=()):
        index = [i for i, name in enumerate(names) if name.endswith(self.joint_suffixes)]
        if not index:
            return
        if self._previous is not None and stamp < self._previous[0]:   # a time jump back (sim reset): start over
            self._previous = None
            self.last_moving = None
        speed = 0.0
        if len(velocities) == len(names):
            speed = max(abs(velocities[i]) for i in index)
        elif self._previous is not None and stamp > self._previous[0]:
            before = self._previous[1]
            speed = max((abs(positions[i] - before[names[i]]) for i in index if names[i] in before), default=0.0) \
                / (stamp - self._previous[0])
        self._previous = (stamp, {names[i]: positions[i] for i in index})
        self.last_state = stamp
        if self.last_moving is None:
            self.last_moving = stamp if speed > self.rest_speed else stamp - self.rest_time - 1.0  # unknown: assume rest
        elif speed > self.rest_speed:
            self.last_moving = stamp

    def at_rest(self, cloud_stamp, now):
        '''True when the arm has been still for rest_time before the cloud was taken and before now.'''
        if self.last_state is None or now - self.last_state > self.stale_time:
            return False
        if cloud_stamp <= 0.0:
            cloud_stamp = now
        return cloud_stamp - self.last_moving >= self.rest_time and now - self.last_moving >= self.rest_time


def lag_summary(lags):
    '''"lag median X ms, max Y ms" of cloud lags in seconds, or "no clouds"'''
    if not lags:
        return 'no clouds'
    ordered = sorted(lags)
    return f'lag median {1000 * ordered[len(ordered) // 2]:.0f} ms, max {1000 * ordered[-1]:.0f} ms'


def main():
    rospy.init_node('octomap_cloud_gate')
    detector = RestDetector(rospy.get_param('~joint_suffixes', ARM_JOINTS), rospy.get_param('~rest_speed', 0.05),
                            rospy.get_param('~rest_time', 0.3), rospy.get_param('~stale_time', 1.0))
    pub = rospy.Publisher(rospy.get_param('~cloud_out', 'eef_main_cam/depth_registered/points_at_rest'), PointCloud2,
                          queue_size=1)
    counts = {'passed': 0, 'dropped': 0}
    lags = []
    last_log = [rospy.Time.now().to_sec()]

    def on_joints(msg):
        detector.update(msg.header.stamp.to_sec(), msg.name, msg.position, msg.velocity)

    def on_cloud(msg):
        now, stamp = rospy.Time.now().to_sec(), msg.header.stamp.to_sec()
        if stamp > 0.0:
            lags.append(now - stamp)
        if detector.at_rest(stamp, now):
            counts['passed'] += 1
            pub.publish(msg)
        else:
            counts['dropped'] += 1
        if now - last_log[0] >= 30.0 or now < last_log[0]:
            rospy.loginfo(f'octomap_cloud_gate: {counts["passed"]} clouds passed, {counts["dropped"]} dropped (arm '
                          f'moving, or no fresh joint states); {lag_summary(lags)}')
            last_log[0] = now
            del lags[:]

    rospy.Subscriber(rospy.get_param('~joint_states', 'joint_states'), JointState, on_joints, queue_size=5)
    rospy.Subscriber(rospy.get_param('~cloud_in', 'eef_main_cam/depth_registered/points'), PointCloud2, on_cloud,
                     queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
