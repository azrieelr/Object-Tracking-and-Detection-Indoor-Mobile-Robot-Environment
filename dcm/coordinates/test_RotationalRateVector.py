import unittest
import numpy as np
from coordinates import DirectionCosineMatrix, RotationalRateVector

class TestRotationalRateVector(unittest.TestCase):
    def setUp(self):
        self.dcm = DirectionCosineMatrix() # Initialize this with appropriate values
        self.rotVec1 = RotationalRateVector(np.array([[1], [2], [3]]), self.dcm, 'object1', 'refFrame1', 'proj1')
        self.rotVec2 = RotationalRateVector(np.array([[4], [5], [6]]), self.dcm, 'object1', 'refFrame1', 'proj1')
        self.rotVec3 = RotationalRateVector(np.array([[1], [1], [1]]), self.dcm, 'object2', 'refFrame2', 'proj2')

    def test_initialization(self):
        self.assertTrue(np.array_equal(self.rotVec1.rotRateVec, self.dcm @ np.array([[1], [2], [3]])))
        self.assertEqual(self.rotVec1.name['of'], 'object1')
        self.assertEqual(self.rotVec1.name['wrt'], 'refFrame1')
        self.assertEqual(self.rotVec1.name['projection'], 'proj1')

    def test_setProjection(self):
        new_dcm = DirectionCosineMatrix()
        new_dcm.compileRotMatrix('x', np.radians(45))
        self.rotVec1.setProjection(new_dcm, 'proj2')
        self.assertEqual(self.rotVec1.name['projection'], 'proj2')
        self.assertEqual(self.rotVec1.dcmProjection, new_dcm)
        self.assertTrue(np.array_equal(self.rotVec1.rotRateVec, new_dcm @ np.array([[1], [2], [3]])))

    def test_addition(self):
        addedRotVec = self.rotVec1 + self.rotVec2
        self.assertTrue(np.array_equal(addedRotVec.rotRateVec, np.array([[5], [7], [9]])))
        with self.assertRaises(ValueError):
            _ = self.rotVec1 + self.rotVec3

    def test_multiplication_and_division(self):
        multipliedRotVec = self.rotVec1 * 3
        self.assertTrue(np.array_equal(multipliedRotVec.rotRateVec, np.array([[3], [6], [9]])))
        with self.assertRaises(ValueError):
            _ = self.rotVec1 * self.rotVec2

        dividedRotVec = self.rotVec1 / 2
        self.assertTrue(np.array_equal(dividedRotVec.rotRateVec, np.array([[0.5], [1], [1.5]])))
        with self.assertRaises(ValueError):
            _ = self.rotVec1 / self.rotVec2

if __name__ == "__main__":
    unittest.main()
