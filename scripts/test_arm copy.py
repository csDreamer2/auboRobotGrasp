import numpy as np
import math

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

test_rpy2R()