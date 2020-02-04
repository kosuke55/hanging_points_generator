import os
import pickle
import time
import skrobot

current_dir = os.path.dirname(os.path.abspath(__file__))

f = open('mug_contact_points.pickle', 'rb')
contact_points_list = pickle.load(f)

print(os.path.join(current_dir, "./urdf/mug/base.urdf"))
obj_model = skrobot.models.urdf.RobotModelFromURDF(
    urdf_file=os.path.join(current_dir, "./urdf/mug/base.urdf"))
viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(obj_model)
viewer.show()

time.sleep(0.5)
center = [-0.023543, -0.007383, 0.041545]
for i, cp in enumerate(contact_points_list):
    print(cp + center)
    contact_point_sphere = skrobot.models.Sphere(0.001, color=[255, 0, 0])
    contact_point_sphere.newcoords(
        skrobot.coordinates.Coordinates(
            pos=(cp + center)))
    viewer.add(contact_point_sphere)
    time.sleep(0.5)
