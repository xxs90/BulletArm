from bulletarm.envs.base_env import BaseEnv
from bulletarm.pybullet.utils import constants
from bulletarm.pybullet.utils.constants import NoValidPositionException
from bulletarm.planners.house_building_1_planner import HouseBuilding1Planner
from bulletarm.planners.block_stacking_planner import BlockStackingPlanner
from bulletarm.envs.block_structure_envs.block_stacking_env import BlockStackingEnv

class BlockArrangingEnv(BaseEnv):
  '''Open loop block arranging task.

  The robot needs to stack all N cubic blocks. The number of blocks N is set by the config.

  Args:
    config (dict): Intialization arguments for the env
  '''
  def __init__(self, config):
    # env specific parameters
    if 'object_scale_range' not in config:
      config['object_scale_range'] = [0.6, 0.6]
    if 'num_objects' not in config:
      config['num_objects'] = 4
    if 'max_steps' not in config:
      config['max_steps'] = 10
    if 'arrange_small' not in config:
      config['arrange_small'] = True
    if 'arrange_big' not in config:
      config['arrange_big'] = True
    super(BlockArrangingEnv, self).__init__(config)

  def resetSmall(self):
    ''''''
    while True:
      self.resetPybulletWorkspace()

      try:
        self._generateShapes(constants.TRIANGLE, 1, random_orientation=self.random_orientation)
        self._generateShapes(constants.CUBE, 1, random_orientation=self.random_orientation)

      except NoValidPositionException as e:
        continue
      else:
        break
    return self._getObservation()

  def resetBig(self):
    ''''''
    if self.object_type == 'cube':
      object_type = constants.CUBE
    elif self.object_type == 'cylinder':
      object_type = constants.CYLINDER
    else:
      raise ValueError('Invalid object type specified. Must be \'cube\' or \'cylinder\'')

    while True:
      self.resetPybulletWorkspace()

      try:
        self._generateShapes(object_type, self.num_obj, random_orientation=self.random_orientation)

      except NoValidPositionException as e:
        continue
      else:
        break
    return self._getObservation()

  # def _checkTerminationSmall(self):
  #   ''''''
  #   blocks = list(filter(lambda x: self.object_types[x] == constants.CUBE, self.objects))
  #   triangles = list(filter(lambda x: self.object_types[x] == constants.TRIANGLE, self.objects))
  #   return self._checkStack(blocks + triangles) and self._checkObjUpright(triangles[0])

  def _checkTermination(self):
    ''''''
    return self._checkStack()

  # def isSimValidSmall(self):
  #   triangles = list(filter(lambda x: self.object_types[x] == constants.TRIANGLE, self.objects))
  #   return self._checkObjUpright(triangles[0]) and super(BlockArrangingEnv, self).isSimValid()

  # def isSimValidBig(self):
  #   triangles = list(filter(lambda x: self.object_types[x] == constants.TRIANGLE, self.objects))
  #   return self._checkObjUpright(triangles[0]) and super(BlockArrangingEnv, self).isSimValid()

def createBlockArrangingEnv(config):
  return BlockArrangingEnv(config)



if __name__ == '__main__':
  smallFlag = True
  env = BlockArrangingEnv({'seed': 0, 'render': True})

  planner = HouseBuilding1Planner(env, {})
  env.resetSmall()

  while True:
    action = planner.getNextAction()
    (state, obs, in_hands), reward, done = env.step(action)

    if done:
      if smallFlag == True:
        env.resetBig()
        planner = BlockStackingPlanner(env, {})
      else:
        env.resetSmall()
        planner = HouseBuilding1Planner(env, {})
      smallFlag = not smallFlag