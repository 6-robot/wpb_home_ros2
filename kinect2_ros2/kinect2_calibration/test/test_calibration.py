"""Offline checks for the migrated calibration executable (no Kinect needed)."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import cv2
import numpy as np
from ament_index_python.packages import get_package_prefix


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = str(Path(get_package_prefix('kinect2_calibration')) /
                         'lib/kinect2_calibration/kinect2_calibration_node')
        cls.environment = dict(os.environ, ROS_DOMAIN_ID='173', ROS_LOCALHOST_ONLY='1',
                               QT_QPA_PLATFORM='offscreen', OMP_NUM_THREADS='2')

    def run_node(self, *args):
        return subprocess.run([self.binary, *map(str, args)], env=self.environment,
                              capture_output=True, text=True, timeout=45)

    def test_help_and_invalid_inputs(self):
        self.assertEqual(self.run_node('--help').returncode, 0)
        self.assertNotEqual(self.run_node('calibrate', '-path', '/missing/calibration').returncode, 0)
        self.assertNotEqual(self.run_node('chess0x7x0.03', 'calibrate').returncode, 0)
        with tempfile.TemporaryDirectory() as directory:
            for mode in ['color', 'ir', 'sync', 'depth']:
                result = self.run_node('calibrate', mode, '-path', directory)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertFalse(list(Path(directory).glob('calib_*.yaml')))

    def test_shutdown_before_first_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            process = subprocess.Popen([self.binary, 'record', '-path', directory],
                                       env=self.environment, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            try:
                time.sleep(1.5)
                self.assertIsNone(process.poll())
                process.send_signal(signal.SIGINT)
                _, error = process.communicate(timeout=8)
                self.assertEqual(process.returncode, 0, error)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def test_known_intrinsics_and_bad_pattern(self):
        expected = np.array([[1050., 0., 960.], [0., 1040., 540.], [0., 0., 1.]])
        board = np.zeros((35, 3), np.float32)
        board[:, :2] = np.mgrid[0:5, 0:7].T.reshape(-1, 2) * .03
        rng = np.random.default_rng(7)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            for index in range(20):
                rotation = rng.uniform(-.4, .4, (3, 1))
                translation = np.array([rng.uniform(-.2, .1), rng.uniform(-.2, .1),
                                        rng.uniform(.6, 1.5)]).reshape(3, 1)
                corners, _ = cv2.projectPoints(board, rotation, translation, expected,
                                               np.zeros(5))
                cv2.imwrite(str(path / f'{index:04d}_color.png'), np.zeros((1, 1), np.uint8))
                storage = cv2.FileStorage(str(path / f'{index:04d}_color_points.yaml'),
                                         cv2.FILE_STORAGE_WRITE)
                storage.startWriteStruct('points', cv2.FileNode_SEQ | cv2.FileNode_FLOW)
                for value in corners.ravel():
                    storage.write('', float(value))
                storage.endWriteStruct()
                storage.release()
            result = self.run_node('chess5x7x0.03', 'calibrate', 'color', '-path', path)
            self.assertEqual(result.returncode, 0, result.stderr)
            storage = cv2.FileStorage(str(path / 'calib_color.yaml'), cv2.FILE_STORAGE_READ)
            actual = storage.getNode('cameraMatrix').mat()
            storage.release()
            np.testing.assert_allclose(actual, expected, atol=.1, rtol=0)
            # A different board must fail before replacing the valid result.
            saved = (path / 'calib_color.yaml').read_bytes()
            result = self.run_node('chess6x7x0.03', 'calibrate', 'color', '-path', path)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual((path / 'calib_color.yaml').read_bytes(), saved)
            (path / '0000_color_points.yaml').write_text('invalid: [yaml')
            result = self.run_node('chess5x7x0.03', 'calibrate', 'color', '-path', path)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual((path / 'calib_color.yaml').read_bytes(), saved)

    def test_stereo_and_depth_offset(self):
        color_k = np.array([[1050., 0., 960.], [0., 1040., 540.], [0., 0., 1.]])
        ir_k = np.array([[365., 0., 256.], [0., 365., 212.], [0., 0., 1.]])
        board = np.zeros((35, 3), np.float32)
        board[:, :2] = np.mgrid[0:5, 0:7].T.reshape(-1, 2) * .03
        offset = np.array([[.05], [0.], [0.]])
        rng = np.random.default_rng(19)

        def save_points(path, points):
            storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
            storage.startWriteStruct('points', cv2.FileNode_SEQ | cv2.FileNode_FLOW)
            for value in points.ravel():
                storage.write('', float(value))
            storage.endWriteStruct()
            storage.release()

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            for name, matrix in [('color', color_k), ('ir', ir_k)]:
                storage = cv2.FileStorage(str(path / f'calib_{name}.yaml'), cv2.FILE_STORAGE_WRITE)
                storage.write('cameraMatrix', matrix)
                storage.write('distortionCoefficients', np.zeros((1, 5)))
                storage.release()
            for i in range(15):
                rotation = rng.uniform(-.2, .2, (3, 1))
                translation = np.array([-.06, -.08, rng.uniform(.7, 1.4)]).reshape(3, 1)
                for name, matrix, t in [('color', color_k, translation + offset),
                                         ('ir', ir_k, translation)]:
                    points, _ = cv2.projectPoints(board, rotation, t, matrix, np.zeros(5))
                    save_points(path / f'{i:04d}_sync_{name}_points.yaml', points)
                cv2.imwrite(str(path / f'{i:04d}_sync_color.png'), np.zeros((1, 1), np.uint8))
            result = self.run_node('chess5x7x0.03', 'calibrate', 'sync', '-path', path)
            self.assertEqual(result.returncode, 0, result.stderr)
            storage = cv2.FileStorage(str(path / 'calib_pose.yaml'), cv2.FILE_STORAGE_READ)
            np.testing.assert_allclose(storage.getNode('rotation').mat(), np.eye(3), atol=1e-4)
            np.testing.assert_allclose(storage.getNode('translation').mat(), offset, atol=1e-4)
            storage.release()
            # Depth mode reads IR captures; use fronto-parallel planes with a known +20 mm error.
            for i, distance in enumerate([.8, 1., 1.2]):
                points, _ = cv2.projectPoints(board, np.zeros((3, 1)),
                    np.array([-.06, -.08, distance]), ir_k, np.zeros(5))
                save_points(path / f'd{i}_ir_points.yaml', points)
                cv2.imwrite(str(path / f'd{i}_grey_ir.png'), np.zeros((424, 512), np.uint8))
                cv2.imwrite(str(path / f'd{i}_depth.png'),
                            np.full((424, 512), round(distance * 1000) + 20, np.uint16))
            # Remove the synthetic stereo filenames: those captures have no depth images.
            for file in path.glob('*_sync_color.png'):
                file.unlink()
            result = self.run_node('chess5x7x0.03', 'calibrate', 'depth', '-path', path)
            self.assertEqual(result.returncode, 0, result.stderr)
            storage = cv2.FileStorage(str(path / 'calib_depth.yaml'), cv2.FILE_STORAGE_READ)
            self.assertAlmostEqual(storage.getNode('depthShift').real(), -20., delta=.2)
            storage.release()

    def test_pose_converter(self):
        converter = Path(self.binary).with_name('convert_calib_pose_to_urdf_format.py')
        with tempfile.TemporaryDirectory() as directory:
            pose = Path(directory) / 'calib_pose.yaml'
            storage = cv2.FileStorage(str(pose), cv2.FILE_STORAGE_WRITE)
            storage.write('rotation', np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]))
            storage.write('translation', np.array([[.05], [.01], [0.]]))
            storage.release()
            result = subprocess.run([str(converter), '-f', str(pose)], capture_output=True,
                                    text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            import xml.etree.ElementTree as ET
            origin = ET.fromstring(result.stdout).find('origin')
            np.testing.assert_allclose(np.fromstring(origin.get('xyz'), sep=' '), [.05, .01, 0.])
            np.testing.assert_allclose(np.fromstring(origin.get('rpy'), sep=' '), [0., 0., np.pi/2])


if __name__ == '__main__':
    unittest.main()
