import numpy as np
import time
import icp
import matplotlib.pyplot as plt

# Constants
N = 100                                    # number of random points in the dataset
num_tests = 100                             # number of test iterations
dim = 2                                     # number of dimensions of the points
noise_sigma = .01                           # standard deviation error to be added
translation = 1.                            # max translation of the test set
rotation = .5                               # max rotation (radians) of the test set


def test_icp():
    # Generate a random dataset
    A = np.random.rand(N, dim)

    total_time = 0

    for i in range(num_tests):

        B = np.copy(A)

        # Translate
        t = np.random.rand(dim)*translation
        B += t

        # Rotate
        #R = rotation_matrix(np.random.rand(dim), np.random.rand() * rotation)
        #B = np.dot(R, B.T).T

        # Add noise
        B += np.random.randn(N, dim) * noise_sigma

        # Shuffle to disrupt correspondence
        np.random.shuffle(B)

        # Run ICP
        start = time.time()
        T, distances, iterations, r, t2 = icp.icp(B, A, max_iterations=20, tolerance=0.000001)
        total_time += time.time() - start

        # Make C a homogeneous representation of B
        C = np.ones((N, dim + 1))
        C[:,0:dim] = np.copy(B)

        # Transform C
        C = np.dot(T, C.T).T

    print('icp time: {:.3}'.format(total_time/num_tests))
    print(r)
    print('\n')
    print(t2)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter([x[0] for x in A], [y[1] for y in A], c='red')
    ax.scatter([x[0] for x in B], [y[1] for y in B], c='blue')
    ax.scatter([x[0] for x in C], [y[1] for y in C], c='green')
    plt.show()

    return


if __name__ == "__main__":
    test_icp()
