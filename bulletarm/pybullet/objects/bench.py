import sys
sys.path.append('..')

import pybullet as pb
import numpy as np
import os

import bulletarm
from bulletarm.pybullet.objects.pybullet_object import PybulletObject
from bulletarm.pybullet.utils import constants

class Bench(PybulletObject):
  def __init__(self, pos, rot, scale):
    self.scale = scale
    root_dir = os.path.dirname(bulletarm.__file__)
    urdf_filepath = os.path.join(root_dir, constants.OBJECTS_PATH, 'shoe_pack/bench/bench.urdf')
    pos = (pos[0], pos[1], 0)
    object_id = pb.loadURDF(urdf_filepath, basePosition=pos, baseOrientation=rot, globalScaling=scale)
    super(Bench, self).__init__(constants.BENCH, object_id)

  def getLeftPose(self):
    return pb.getLinkState(self.object_id, 0)

  def getRightPose(self):
    return pb.getLinkState(self.object_id, 1)