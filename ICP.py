<<<<<<< HEAD
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors


def icp(a, b, init_pose=(0, 0, 0), no_iterations=13):
    src = np.array([a.T], copy=True).astype(np.float32)
    dst = np.array([b.T], copy=True).astype(np.float32)

    # 齐次变换矩阵(旋转+平移)
    Tr = np.array([[np.cos(init_pose[2]), -np.sin(init_pose[2]), init_pose[0]],
                   [np.sin(init_pose[2]), np.cos(init_pose[2]), init_pose[1]],
                   [0,                    0,                   1]])

    src = cv2.transform(src, Tr[0:2])

    for i in range(no_iterations):
        # 找到src中每个点在dst中的最近邻点
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(dst[0])
        # 最近邻点的距离, 临近点索引
        distances, indices = nbrs.kneighbors(src[0])

        # 计算从src到dst的最优仿射变换矩阵T
        T, _ = cv2.estimateAffinePartial2D(src[0], dst[0][indices.T[0]])

        if T is None:
            print("Transformation matrix not found")
            break

        src = cv2.transform(src, T)

        Tr = np.dot(Tr, np.vstack((T, [0, 0, 1])))
    return Tr[0:2]


ang = np.linspace(-np.pi/2, np.pi/2, 320)
a = np.array([ang, np.sin(ang)])
th = np.pi/2
rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
b = np.dot(rot, a) + np.array([[0.2], [0.3]])

M2 = icp(a, b, [0.1,  0.33, np.pi/2.2], 30)

src = np.array([a.T]).astype(np.float32)
res = cv2.transform(src, M2)
plt.figure()
plt.plot(b[0], b[1])
plt.plot(res[0].T[0], res[0].T[1], 'r.')
plt.plot(a[0], a[1])
plt.show()
=======
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors


def icp(a, b, init_pose=(0, 0, 0), no_iterations=13):
    src = np.array([a.T], copy=True).astype(np.float32)
    dst = np.array([b.T], copy=True).astype(np.float32)

    # 齐次变换矩阵(旋转+平移)
    Tr = np.array([[np.cos(init_pose[2]), -np.sin(init_pose[2]), init_pose[0]],
                   [np.sin(init_pose[2]), np.cos(init_pose[2]), init_pose[1]],
                   [0,                    0,                   1]])

    src = cv2.transform(src, Tr[0:2])

    for i in range(no_iterations):
        # 找到src中每个点在dst中的最近邻点
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(dst[0])
        # 最近邻点的距离, 临近点索引
        distances, indices = nbrs.kneighbors(src[0])

        # 计算从src到dst的最优仿射变换矩阵T
        T, _ = cv2.estimateAffinePartial2D(src[0], dst[0][indices.T[0]])

        if T is None:
            print("Transformation matrix not found")
            break

        src = cv2.transform(src, T)

        Tr = np.dot(Tr, np.vstack((T, [0, 0, 1])))
    return Tr[0:2]


ang = np.linspace(-np.pi/2, np.pi/2, 320)
a = np.array([ang, np.sin(ang)])
th = np.pi/2
rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
b = np.dot(rot, a) + np.array([[0.2], [0.3]])

M2 = icp(a, b, [0.1,  0.33, np.pi/2.2], 30)

src = np.array([a.T]).astype(np.float32)
res = cv2.transform(src, M2)
plt.figure()
plt.plot(b[0], b[1])
plt.plot(res[0].T[0], res[0].T[1], 'r.')
plt.plot(a[0], a[1])
plt.show()
>>>>>>> 20f8e51d558ac903605a46ec3db2554ae42e5131
