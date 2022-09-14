import pybullet as pb
import numpy as np

from bulletarm.envs.close_loop_envs.close_loop_env import CloseLoopEnv
from bulletarm.pybullet.utils import constants
from bulletarm.planners.close_loop_kitchen_setting_planner import CloseLoopKitchenSettingPlanner
from bulletarm.pybullet.utils.constants import NoValidPositionException
from bulletarm.pybullet.objects.kitchen_plate import KitchenPlate
from bulletarm.pybullet.objects.kitchen_fork import KitchenFork
from bulletarm.pybullet.objects.kitchen_knife import KitchenKnife


class CloseLoopKitchenSettingEnv(CloseLoopEnv):
    '''Close loop shoe packing task.

  The robot needs to pick up the shoes and place on the shoe rack in pairs.

  Args:
    config (dict): Initialization arguments for the env
  '''

    def __init__(self, config):
      super().__init__(config)

    def reset(self):
      while True:
        self.resetPybulletWorkspace()
        try:
          self._generateShapes(constants.KITCHEN_LOBSTER_PLATE, 1, scale=0.8, random_orientation=self.random_orientation)
          # self._generateShapes(constants.KITCHEN_LOBSTER_PLATE, 1, scale=1, rot=[(0, 0, 0, 1)])
          self._generateShapes(constants.KITCHEN_FORK, 1, scale=0.85, random_orientation=self.random_orientation)
          # self._generateShapes(constants.KITCHEN_FORK, 1, scale=1, rot=[(0, 0, 0, 1)])
          self._generateShapes(constants.KITCHEN_KNIFE, 1, scale=0.95, random_orientation=self.random_orientation)
          # self._generateShapes(constants.KITCHEN_KNIFE, 1, scale=1, rot=[(0, 0, 0, 1)])

        except NoValidPositionException as e:
          continue
        else:
          break
      return self._getObservation()

    def _checkTermination(self):
      # lobster plate size is (x = 0.160, y = 0.160, z = 0.034)
      # knife size is (x = 0.108, y = 0.030, z = 0.022)
      # fork size is (x = 0.109, y = 0.020, z = 0.021)

      fork_pos = self.objects[1].getPosition()
      knife_pos = KitchenKnife.getGraspPose(self.objects[2])[0]

      plate_pos_left = KitchenPlate.getLeftPos(self.objects[0])[0]
      plate_pos_right = KitchenPlate.getRightPos(self.objects[0])[0]

      if not (self._checkObjUpright(self.objects[1]) and self._checkObjUpright(self.objects[2])):
        return False

      finish = 0
      if np.linalg.norm(np.array(fork_pos[:2]) - np.array(plate_pos_left[:2])) < 0.03 \
              and np.linalg.norm(np.array(fork_pos[2]) - np.array(plate_pos_left[2])) < 0.01:
        finish += 1
      if np.linalg.norm(np.array(knife_pos[:2]) - np.array(plate_pos_right[:2])) < 0.03 \
              and np.linalg.norm(np.array(knife_pos[2]) - np.array(plate_pos_right[2])) < 0.01:
        finish += 1

      return finish == 2


    def isSimValid(self):
      for obj in self.objects:
        p = obj.getPosition()

        if self._isObjectHeld(obj):
          continue
        if not self.workspace[0][0] - 0.05 < p[0] < self.workspace[0][1] + 0.05 and \
          self.workspace[1][0] - 0.05 < p[1] < self.workspace[1][1] + 0.05 and \
          self.workspace[2][0] < p[2] < self.workspace[2][1]:
          return False
        return True


def createCloseLoopKitchenSettingEnv(config):
  return CloseLoopKitchenSettingEnv(config)


# if __name__ == '__main__':
#   env = CloseLoopKitchenSettingEnv({'seed': 0, 'render': True})
#   planner = CloseLoopKitchenSettingPlanner(env, {})
#   env.reset()
#   # count = 0
#   while True:
#     action = planner.getNextAction()
#     (state, obs, in_hands), reward, done = env.step(action)
#     import time
#     # time.sleep(0.1)
#     if done:
#   #     # count += 1
#       env.reset()
#   #     # if count == 6:
#   #       # print(count)