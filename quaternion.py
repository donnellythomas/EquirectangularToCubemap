def quaternion_mult(q, r):
    return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]


def point_rotation_by_quaternion(point, q):
    r = [0]+point
    print("r:", r)
    q_conj = [q[0], -1*q[1], -1*q[2], -1*q[3]]
    q1 = quaternion_mult(q, r)
    print(q1)
    q2 = quaternion_mult(q1, q_conj)
    print(q2)
    return q2[1:]


# print(point_rotation_by_quaternion([1, 0, 0], [
#       0.7071203316249954, 0.0, 0.7071203316249954, 0.0]))
q = [0, 0.707, 0, -0.707]
r = [0.707, 0, -0.707, 0]
print(r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0])
