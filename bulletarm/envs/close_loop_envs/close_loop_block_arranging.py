import pybullet as pb
import numpy as np

from bulletarm.pybullet.utils import constants
from bulletarm.envs.close_loop_envs.close_loop_env import CloseLoopEnv
from bulletarm.pybullet.utils import transformations
from bulletarm.planners.close_loop_house_building_1_planner import CloseLoopHouseBuilding1Planner
from bulletarm.planners.close_loop_block_arranging_planner import CloseLoopBlockArrangingPlanner
from bulletarm.pybullet.utils.constants import NoValidPositionException

class CloseLoopBlockArrangingEnv(CloseLoopEnv):
  '''Open loop block arranging task.

  The robot needs to stack all N cubic blocks. The number of blocks N is set by the config.

  Args:
    config (dict): Intialization arguments for the env
  '''
  def __init__(self, config):
    # env specific parameters
    super().__init__(config)
    self.goal_pos_cube = self.workspace.mean(1)[:2]
    self.goal_pos_tri = self.workspace.mean(1)[:2]
    self.arrange_stack = False
    self.arrange_sort = True


  def reset(self):

    while True:
      self.resetPybulletWorkspace()
      if self.arrange_stack:
        self.robot.moveTo([self.workspace[0].mean(), self.workspace[1].mean(), 0.2], transformations.quaternion_from_euler(0, 0, 0))
      try:
        if self.arrange_stack:
          self._generateShapes(constants.TRIANGLE, 1, random_orientation=self.random_orientation)
          self._generateShapes(constants.CUBE, 1, random_orientation=self.random_orientation)
          # generate red blocks
          pb.changeVisualShape(3, -1, rgbaColor=[1, 0, 0, 1])
          pb.changeVisualShape(4, -1, rgbaColor=[1, 0, 0, 1])
        if self.arrange_sort:
          self._generateShapes(constants.CUBE_BIG, 1, scale=1, random_orientation=self.random_orientation)
          self._generateShapes(constants.TRIANGLE_BIG, 1, scale=1, random_orientation=self.random_orientation)
          # generate yellow blocks
          pb.changeVisualShape(3, -1, rgbaColor=[1, 1, 0, 1])
          pb.changeVisualShape(4, -1, rgbaColor=[1, 1, 0, 1])

          # left side and right side sorting task
          self.goal_pos_cube = [0.45, -0.09]
          self.goal_pos_tri = [0.45, 0.09]
      except NoValidPositionException as e:
        continue
      else:
        break

    return self._getObservation()


  def _checkTermination(self):
    ''''''
    if self.arrange_stack:
      obj_pos_cube = self.objects[0].getPosition()[:2]
      obj_pos_tri = self.objects[1].getPosition()[:2]
      sort_cube = abs(obj_pos_cube[1] - self.goal_pos_cube[1]) < 0.02
      sort_tri = abs(obj_pos_tri[1] - self.goal_pos_tri[1]) < 0.02
      return sort_cube and sort_tri
    if self.arrange_sort:
      blocks = list(filter(lambda x: self.object_types[x] == constants.CUBE, self.objects))
      triangles = list(filter(lambda x: self.object_types[x] == constants.TRIANGLE, self.objects))
      return not self._isHolding() and self._checkStack(blocks + triangles) and self._checkObjUpright(triangles[0]) and self._isObjOnTop(triangles[0])


  def isSimValid(self):
    for obj in self.objects:
      p = obj.getPosition()
      if self._isObjectHeld(obj):
        continue
      if not self.workspace[0][0]-0.05 < p[0] < self.workspace[0][1]+0.05 and \
          self.workspace[1][0]-0.05 < p[1] < self.workspace[1][1]+0.05 and \
          self.workspace[2][0] < p[2] < self.workspace[2][1]:
        return False
    return True


def createCloseLoopBlockArrangingEnv(config):
  return CloseLoopBlockArrangingEnv(config)


if __name__ == '__main__':
  env = CloseLoopBlockArrangingEnv({'seed': 0, 'render': True})

  if env.arrange_stack:
    planner = CloseLoopHouseBuilding1Planner(env, {})
  if env.arrange_sort:
    planner = CloseLoopBlockArrangingPlanner(env, {})
  else:
    print('no task entered')
  env.reset()

  while True:
    action = planner.getNextAction()
    (state, obs, in_hands), reward, done = env.step(action)

    if done:
      env.reset()