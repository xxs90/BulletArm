import sys
sys.path.append('..')

import pybullet as pb
import numpy as np
import os

import bulletarm
from bulletarm.pybullet.objects.pybullet_object import PybulletObject
from bulletarm.pybullet.utils import constants


# def getPlateRadius(model_id):
#   if model_id == 1:
#     return 0.18
#   else:
#     raise NotImplementedError
#
# def getZOffset(model_id):
#   if model_id == 1:
#     return 0.01
#   else:
#     raise NotImplementedError
#
# PLACE_RY_OFFSET = {
#   0: np.deg2rad(80),
#   1: np.deg2rad(55),
#   2: np.deg2rad(60),
#   3: np.deg2rad(60),
#   4: np.deg2rad(60),
#   5: np.deg2rad(60),
#   6: np.deg2rad(60),
#   7: np.deg2rad(60),
#   8: np.deg2rad(70),
#   9: np.deg2rad(60),
#   10: np.deg2rad(60),
#   11: np.deg2rad(60),
# }
#
# PLACE_Z_OFFSET = {
#   0: 0.06,
#   1: 0.04,
#   2: 0.04,
#   3: 0.04,
#   4: 0.04,
#   5: 0.04,
#   6: 0.04,
#   7: 0.05,
#   8: 0.06,
#   9: 0.04,
#   10: 0.05,
#   11: 0.05,
# }

class KitchenPlate(PybulletObject):
  def __init__(self, pos, rot, scale):
    self.scale = scale
    root_dir = os.path.dirname(bulletarm.__file__)
    urdf_filepath = os.path.join(root_dir, constants.OBJECTS_PATH, 'kitchen_set/lobster_plate/lobster_plate.urdf')
    object_id = pb.loadURDF(urdf_filepath, basePosition=pos, baseOrientation=rot, globalScaling=scale)

    super(KitchenPlate, self).__init__(constants.KITCHEN_LOBSTER_PLATE, object_id)

  def getLeftPos(self):
    return pb.getLinkState(self.object_id, 0)

  def getRightPos(self):
    return pb.getLinkState(self.object_id, 1)

