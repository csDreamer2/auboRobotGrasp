import math
import numpy as np

def rpy2R(rpy): # [r,p,y] 单位rad
        rot_x = np.array([[1, 0, 0],
                          [0, math.cos(rpy[0]), -math.sin(rpy[0])],
                          [0, math.sin(rpy[0]), math.cos(rpy[0])]])
        rot_y = np.array([[math.cos(rpy[1]), 0, math.sin(rpy[1])],
                          [0, 1, 0],
                          [-math.sin(rpy[1]), 0, math.cos(rpy[1])]])
        rot_z = np.array([[math.cos(rpy[2]), -math.sin(rpy[2]), 0],
                          [math.sin(rpy[2]), math.cos(rpy[2]), 0],
                          [0, 0, 1]])
        R = np.dot(rot_z, np.dot(rot_y, rot_x))
        return R

def R2rpy(R):
    # assert (isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

def getRpy(rpy):
    for i in range(3):
        if rpy[i] > np.pi:
            rpy[i] -= 2 * np.pi
        elif rpy[i] < -np.pi:
            rpy[i] += 2 * np.pi
    return rpy      

#测试
def test_rpy2R():
    # Test case 1: rpy = [0, 0, 0]
    rpy = [0, 0, 0]
    expected_result = np.array([[1, 0, 0],
                                [0, 1, 0],
                                [0, 0, 1]])
    result = rpy2R(rpy)
    assert np.allclose(result, expected_result), f"Test case 1 failed. Expected: {expected_result}, Got: {result}"

    # Test case 2: rpy = [0.5, 0.5, 0.5]
    rpy = [0.5, 0.5, 0.5]
    expected_result = np.array([[0.85355339, -0.14644661, 0.5],
                                [0.5, 0.85355339, -0.14644661],
                                [-0.14644661, 0.5, 0.85355339]])
    result = rpy2R(rpy)
    assert np.allclose(result, expected_result), f"Test case 2 failed. Expected: {expected_result}, Got: {result}"

    # Test case 3: rpy = [-0.3, 0.7, -0.2]
    rpy = [-0.3, 0.7, -0.2]
    expected_result = np.array([[0.93675213, -0.27516334, -0.21835066],
                                [0.28962948, 0.95642505, 0.03673918],
                                [0.19866933, -0.0978434, 0.97517033]])
    result = rpy2R(rpy)
    assert np.allclose(result, expected_result), f"Test case 3 failed. Expected: {expected_result}, Got: {result}"

    print("All test cases passed!")