# Copyright (c) 2026, DFKI GmbH and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

'''Every robot of the sim runs the same robot_stack_sim.launch, so a flag added to demo_sim.launch reaches robot 1 and
robots 2 and 3 alike: the include declares and passes on every bringup argument demo_sim has, and each of the three
includes in demo_sim passes the same set. No ROS needed (XML only).'''

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

LAUNCH = Path(__file__).resolve().parents[1] / 'launch'
STACK = 'includes/robot_stack_sim.launch'
PER_ROBOT = {'namespace', 'map_frame', 'map_ns', 'pose_selector_ns', 'offset_x'}


def declared(root):
    return {a.get('name'): a.get('default') for a in root.findall('arg')}


def passed(include):
    return {a.get('name') for a in include.findall('arg')}


class TestRobotStackSimArgs(unittest.TestCase):
    def setUp(self):
        self.demo = ET.parse(LAUNCH / 'demo_sim.launch').getroot()
        self.stack = ET.parse(LAUNCH / STACK).getroot()
        self.includes = [i for i in self.demo.findall('include') if i.get('file', '').endswith(STACK)]

    def test_robot_1_2_and_3_use_the_stack(self):
        self.assertEqual([i.get('if') is None for i in self.includes], [True, False, False])

    def test_every_include_passes_the_same_flags(self):
        sets = [passed(i) - PER_ROBOT for i in self.includes]
        self.assertEqual(sets[0], sets[1])
        self.assertEqual(sets[0], sets[2])

    def test_the_stack_declares_what_demo_sim_passes(self):
        self.assertLessEqual(passed(self.includes[0]), set(declared(self.stack)))

    def test_every_shared_flag_of_demo_sim_reaches_the_stack(self):
        # a flag added to demo_sim.launch and to the bringup but not passed on would silently stay at its default
        bringup = ET.parse(LAUNCH / 'bringup.launch').getroot()
        shared = set(declared(self.demo)) & set(declared(bringup))
        shared -= {'mtc_cable_constraints'}  # reaches the stack as mtc_cable_plan_check
        self.assertEqual(shared - passed(self.includes[0]), set())

    def test_the_bringup_gets_every_flag_of_the_stack(self):
        bringup = [i for i in self.stack.findall('include') if i.get('file', '').endswith('/bringup.launch')][0]
        flags = set(declared(self.stack)) - PER_ROBOT - {'use_ur_polished_kinematics'}
        self.assertLessEqual(flags, passed(bringup))

    def test_the_defaults_of_demo_sim_and_the_stack_agree(self):
        demo, stack = declared(self.demo), declared(self.stack)
        for name in passed(self.includes[0]) - PER_ROBOT:
            if (
                name == 'mtc_cable_plan_check'
            ):  # demo_sim defaults it to mtc_cable_constraints, the stack gets it passed
                continue
            self.assertEqual(demo.get(name), stack.get(name), name)


if __name__ == '__main__':
    unittest.main()
