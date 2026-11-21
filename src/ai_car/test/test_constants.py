"""Validate the shared robot specification without ROS or hardware imports."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from ai_car import constants


class TestSharedConstants(unittest.TestCase):
    def test_original_geometry_and_command_limits(self):
        self.assertEqual((constants.WHEEL_RADIUS, constants.WHEEL_SEPARATION), (0.033, 0.16))
        self.assertEqual((constants.JOINT_MIN_DEG, constants.JOINT_MAX_DEG), (0, 90))
        self.assertEqual((constants.TOOL_MIN_DEG, constants.TOOL_MAX_DEG), (0, 45))

    def test_home_pose_is_within_command_limits(self):
        self.assertEqual(constants.HOME_POS, (0, 0))
        self.assertEqual(constants.HOME_TOOL, 0)
        self.assertTrue(all(constants.JOINT_MIN_DEG <= angle <= constants.JOINT_MAX_DEG
                            for angle in constants.HOME_POS))
        self.assertLessEqual(constants.TOOL_MIN_DEG, constants.HOME_TOOL)
        self.assertLessEqual(constants.HOME_TOOL, constants.TOOL_MAX_DEG)

    def test_identifiers_match_robot_description(self):
        urdf = Path(__file__).resolve().parents[1] / 'urdf' / 'ai_car.urdf'
        root = ET.parse(urdf).getroot()
        joints = {joint.attrib['name'] for joint in root.findall('joint')}
        links = {link.attrib['name'] for link in root.findall('link')}
        self.assertEqual(constants.ARM_NAMES, ('arm_joint_1', 'arm_joint_2', 'tool_joint'))
        self.assertTrue(set(constants.ARM_NAMES) <= joints)
        self.assertEqual(constants.IMU_FRAME_ID, 'imu_link')
        self.assertIn(constants.IMU_FRAME_ID, links)


if __name__ == '__main__':
    unittest.main()
