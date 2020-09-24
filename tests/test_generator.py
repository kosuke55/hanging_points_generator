import itertools
import json
import os
import os.path as osp
import unittest

from hanging_points_generator.generator_utils \
    import check_contact_points
from hanging_points_generator.generator_utils \
    import filter_contact_points
from hanging_points_generator.generator_utils \
    import set_contact_points_urdf_path


class TestGenerator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):

        cls.current_dir = os.path.dirname(os.path.abspath(__file__))
        cls.contact_points_file = osp.join(
            cls.current_dir, '../urdf/scissors/contact_points.json')
        cls.urdf_file = osp.join(
            cls.current_dir, '../urdf/scissors/base.urdf')
        set_contact_points_urdf_path(cls.contact_points_file)

    def test_check_contact_points(self):
        for cluster_min_points in [0, -1, 2]:
            args_list = list(itertools.product([True, False], repeat=5))
            check_contact_points(
                self.contact_points_file,
                self.urdf_file,
                cluster_min_points=cluster_min_points,
                use_filter_penetration=args_list[0],
                inf_penetration_check=args_list[1],
                align=args_list[2],
                average=args_list[3],
                average_pos=args_list[4],
                _test=True)

    def test_filter_contact_points(self):
        contact_points_dict = json.load(
            open(self.contact_points_file, 'r'))
        filter_contact_points(contact_points_dict)
