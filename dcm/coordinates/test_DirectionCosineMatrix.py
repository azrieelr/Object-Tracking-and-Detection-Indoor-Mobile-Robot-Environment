import unittest
import numpy as np

from coordinates import DirectionCosineMatrix

class TestDirectionCosineMatrix(unittest.TestCase):
    def test_init(self):
        dcm = DirectionCosineMatrix()
        np.testing.assert_array_equal(dcm.dcm, np.eye(3))
        self.assertEqual(dcm.rotationList, [])

        dcm = DirectionCosineMatrix(np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]))
        np.testing.assert_array_equal(dcm.dcm, np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]))
        self.assertEqual(dcm.rotationList, [])

        with self.assertRaises(ValueError):
            DirectionCosineMatrix(np.array([[1, 2], [3, 4]]))

    def test_matmul(self):
        dcm1 = DirectionCosineMatrix()
        dcm2 = DirectionCosineMatrix()
        dcm_result = dcm1 @ dcm2
        np.testing.assert_array_equal(dcm_result.dcm, np.eye(3))
        self.assertEqual(dcm_result.rotationList, [])

        with self.assertRaises(ValueError):
            dcm1 @ "invalid_type"

    def test_array(self):
        dcm = DirectionCosineMatrix()
        np.testing.assert_array_equal(np.array(dcm), np.eye(3))

    def test_compileRotMatrix(self):
        dcm = DirectionCosineMatrix()
        dcm.compileRotMatrix('x', np.radians(45))
        expected = np.array([[1., 0., 0.],
                             [0., np.cos(np.radians(45)), -np.sin(np.radians(45))],
                             [0., np.sin(np.radians(45)), np.cos(np.radians(45))]])
        np.testing.assert_array_almost_equal(dcm.dcm, expected)

        dcm = DirectionCosineMatrix()
        dcm.compileRotMatrix('y', np.radians(45))
        expected = np.array([[np.cos(np.radians(45)), 0., np.sin(np.radians(45))],
                            [0., 1., 0.],
                            [-np.sin(np.radians(45)), 0., np.cos(np.radians(45))]])
        np.testing.assert_array_almost_equal(dcm.dcm, expected)

        dcm = DirectionCosineMatrix()
        dcm.compileRotMatrix('z', np.radians(45))
        expected = np.array([[np.cos(np.radians(45)), -np.sin(np.radians(45)), 0.],
                            [np.sin(np.radians(45)), np.cos(np.radians(45)), 0.],
                            [0., 0., 1.]])
        np.testing.assert_array_almost_equal(dcm.dcm, expected)

        dcm = DirectionCosineMatrix()
        dcm.compileRotMatrix('x', np.radians(45), name="rotation1")
        self.assertEqual(dcm.rotationList, [{'axis': 'x', 'radians': np.radians(45), 'name': "rotation1"}])
        expected = np.array([[1., 0., 0.],
                             [0., np.cos(np.radians(45)), -np.sin(np.radians(45))],
                             [0., np.sin(np.radians(45)), np.cos(np.radians(45))]])
        np.testing.assert_array_almost_equal(dcm.dcm, expected)

        dcm.compileRotMatrix('y', np.radians(45), name="rotation2")
        self.assertEqual(dcm.rotationList, [{'axis': 'x', 'radians': np.radians(45), 'name': "rotation1"}, {'axis': 'y', 'radians': np.radians(45), 'name': "rotation2"}])
        expected2 = np.array([[np.cos(np.radians(45)), 0., np.sin(np.radians(45))],
                            [0., 1., 0.],
                            [-np.sin(np.radians(45)), 0., np.cos(np.radians(45))]])
        np.testing.assert_array_almost_equal(dcm.dcm, expected @ expected2)

        with self.assertRaises(ValueError):
            dcm.compileRotMatrix('x', np.radians(45), 45)

        with self.assertRaises(ValueError):
            dcm.compileRotMatrix('x')

    def test_history(self):
        dcm = DirectionCosineMatrix()
        dcm.compileRotMatrix('x', np.radians(45), name="rotation1")
        self.assertEqual(dcm.history(), [{'axis': 'x', 'radians': np.radians(45), 'name': "rotation1"}])

if __name__ == '__main__':
    unittest.main()
