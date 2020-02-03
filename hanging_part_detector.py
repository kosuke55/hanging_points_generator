import numpy as np
from math import cos, pi, sin
import pybullet
import pybullet_data
import time


def rpy2quaternion(rpy):
    yaw, pitch, roll = rpy
    cr, cp, cy = cos(roll / 2.), cos(pitch / 2.), cos(yaw / 2.)
    sr, sp, sy = sin(roll / 2.), sin(pitch / 2.), sin(yaw / 2.)
    return np.array([
        -cr * sp * sy + cp * cy * sr,
        cr * cy * sp + sr * cp * sy,
        cr * cp * sy - sr * cy * sp,
        cr * cp * cy + sr * sp * sy])


enable_changeview_with_key = False
cyaw = 0
cpitch = 0
cdist = 0.5
cx = 0
cz = 0
cy = 0


def change_view_with_key():
    global cyaw, cpitch, cdist, cx, cy, cz
    keys = pybullet.getKeyboardEvents()
    if keys.get(100):  # d
        cyaw += 1
    if keys.get(97):  # a
        cyaw -= 1
    if keys.get(99):  # c
        cpitch += 1
    if keys.get(102):  # f
        cpitch -= 1
    if keys.get(122):  # z
        cdist += .01
    if keys.get(120):  # x
        cdist -= .01
    if keys.get(106):  # j
        cx -= .01
    if keys.get(107):  # k
        cx += .01
    if keys.get(104):  # h
        cy -= .01
    if keys.get(108):  # l
        cy += .01
    if keys.get(105):  # i
        cz += .01
    if keys.get(109):  # m
        cz -= .01
    pybullet.resetDebugVisualizerCamera(
        cameraDistance=cdist,
        cameraYaw=cyaw,
        cameraPitch=cpitch,
        cameraTargetPosition=[
            cx,
            cy,
            cz])


# or p.DIRECT for non-graphical version
physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
gravity = -10
pybullet.setGravity(0, 0, gravity)
planeId = pybullet.loadURDF("plane.urdf")

StartPos = [0, 0, 0]
StartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])
objectId = pybullet.loadURDF("mug/base.urdf", StartPos, StartOrientation)

bar = pybullet.createCollisionShape(
    pybullet.GEOM_CYLINDER,
    radius=0.005,
    height=1)

barid = pybullet.createMultiBody(
    baseMass=0.,
    baseCollisionShapeIndex=bar,
    basePosition=[0, 0, 1],
    baseOrientation=pybullet.getQuaternionFromEuler(
        [0, 1.5707963267948966, 0]))

start = time.time()
reset_pose_time = 3
loop_num = int(1e10)
find_count = 0
collision_check = True

for i in range(loop_num):

    if collision_check:
        pybullet.setGravity(0, 0, 0)
        pybullet.stepSimulation()
        contact_point = pybullet.getContactPoints(objectId, barid)
    elapsed = time.time() - start

    if contact_point:
        x = (np.random.rand() - 0.5) * 0.1
        y = (np.random.rand() - 0.5) * 0.1
        z = 1 + (np.random.rand() - 0.5) * 0.1
        roll = np.random.rand() * pi
        pitch = np.random.rand() * pi
        yaw = np.random.rand() * pi
        pybullet.setGravity(0, 0, 0)
        pybullet.resetBasePositionAndOrientation(
            objectId,
            [x, y, z],
            rpy2quaternion([roll, pitch, yaw]))
        start = time.time()
        collision_check = True
    else:
        contact_point = False
        pybullet.setGravity(0, 0, gravity)
        collision_check = False
    pybullet.stepSimulation()
    # time.sleep(1. / 240.)
    time.sleep(1. / 1e10)
    pos, orn = pybullet.getBasePositionAndOrientation(objectId)

    if enable_changeview_with_key:
        change_view_with_key()

    if (elapsed > reset_pose_time or pos[2] < 0.1):
        x = (np.random.rand() - 0.5) * 0.1
        y = (np.random.rand() - 0.5) * 0.1
        z = 1 + (np.random.rand() - 0.5) * 0.1
        roll = np.random.rand() * pi
        pitch = np.random.rand() * pi
        yaw = np.random.rand() * pi
        pybullet.setGravity(0, 0, 0)
        pybullet.resetBasePositionAndOrientation(
            objectId,
            [x, y, z],
            rpy2quaternion([roll, pitch, yaw]))
        start = time.time()
        collision_check = True
    elif (elapsed > reset_pose_time - 0.1 and pos[2] > 0.1):
        print("Find the hanging part ({})".format(find_count))
        find_count += 1
        x = (np.random.rand() - 0.5) * 0.1
        y = (np.random.rand() - 0.5) * 0.1
        z = 1 + (np.random.rand() - 0.5) * 0.1
        roll = np.random.rand() * pi
        pitch = np.random.rand() * pi
        yaw = np.random.rand() * pi
        pybullet.setGravity(0, 0, 0)
        pybullet.resetBasePositionAndOrientation(
            objectId,
            [x, y, z],
            rpy2quaternion([roll, pitch, yaw]))
        start = time.time()
        collision_check = True

pybullet.disconnect()
