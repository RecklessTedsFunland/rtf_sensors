from math import sqrt, sin, acos
from squaternion import Quaternion


# initialized = False

def norm(v):
    im = 1.0/sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    return (v[0]*im, v[1]*im, v[2]*im,)

def update(a, w, dt, q=None):
    na = norm(a)
    # if not initialized:
    if q is None:
        ax, ay, az = na
        if az >= 0.0:
            n = sqrt((az+1)*0.5)
            inn = 1.0/(2.0*n)
            q = Quaternion(n, -ay*inn, ax*inn, 0)
        else:
            n = (1-az)*0.5)
            inn = 1.0/(2.0*n)
            q = Quaternion(-ay*inn, n, 0, ax*inn)
        # initalized = True
        return q

    # predict quaternion from gyro readings
    qp = q + 0.5*q*Quaternion(0, *w)*dt
    qp = qp.normalize

    # calculate quaternion from acceleration
    a = .9
    # q = qp*((1.0-a)*Quaternion() + scale(a, quatAcc(na, qp)))
    q = qp*scale(a, quatAcc(na, qp))
    q = q.normalize
    return q

def quatAcc(na, qp):
    gx,gy,gz = rotate(na, qp.conjugate) # rotate into World frame
    n = sqrt(0.5*(gz+1))
    qacc = Quaternion(n, -gy/(2*n), gx/(2*n), 0.0)
    return qacc

def scale(s, q):
    q0,q1,q2,q3 = q
    if q0 < 0.0:
        # Slerp (Spherical linear interpolation)
        angle = acos(q0)
        A = sin(angle*(1.0 - s))/sin(angle)
        B = sin(angle * s)/sin(angle)
        q0 = A + B * q0
        q1 = B * q1
        q2 = B * q2
        q3 = B * q3
    else:
        # Lerp (Linear interpolation)
        q0 = (1.0 - s) + s * q0
        q1 = s * q1
        q2 = s * q2
        q3 = s * q3

    return Quaternion(q0,q1,q2,q3).normalized

def rotate(v, q):
    q0,q1,q2,q3 = q
    x,y,z = v
    vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z
    vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z
    vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z
    return (x,y,z,)
