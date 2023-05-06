import pybullet as pb
import numpy as np

from bulletarm.pybullet.utils import constants
from bulletarm.envs.close_loop_envs.close_loop_env import CloseLoopEnv
from bulletarm.pybullet.utils import transformations
# from bulletarm.planners.close_loop_house_building_1_planner import CloseLoopHouseBuilding1Planner
from bulletarm.planners.close_loop_block_arranging_planner import CloseLoopBlockArrangingPlanner
from bulletarm.pybullet.utils.constants import NoValidPositionException

class CloseLoopBlockColorSortEnv(CloseLoopEnv):
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
    # self.workspace_size = self.workspace[0,1]-self.workspace[0,0]


  def createMat(self):
    ws_visual = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[self.workspace_size/2, self.workspace_size/4, 0.001], rgbaColor=[0.2, 0.2, 0.2, 1])
    ws_id1 = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=ws_visual,
                                basePosition=[self.workspace[0].mean(), self.workspace[1][0]+self.workspace_size/4, 0],
                                baseOrientation=[0, 0, 0, 1])
    ws_id2 = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=ws_visual,
                                basePosition=[self.workspace[0].mean(), self.workspace[1][1]-self.workspace_size/4, 0],
                                baseOrientation=[0, 0, 0, 1])
    self.ws_id = [ws_id1, ws_id2]

  def randSampleGray(self, bin_num=10):
    
    bins = np.arange(0,255,255/bin_num)

    left_num = [0,2,4,6,8]
    right_num = [1,3,5,7,9]
    left_idx = np.random.choice(left_num)
    right_idx = np.random.choice(right_num)
    # np.digitize(mylist,bins)
    left_gray = bins[left_idx]/255
    right_gray = bins[right_idx]/255
    left_rgba = [left_gray, left_gray, left_gray, 1]
    right_rgba = [right_gray, right_gray, right_gray, 1]
    return left_rgba, right_rgba

  def reset(self):

    while True:
      self.resetPybulletWorkspace()
      try:
        handle_cube = self._generateShapes(constants.CUBE_BIG, 1, scale=1, random_orientation=self.random_orientation)
        handle_triangle = self._generateShapes(constants.TRIANGLE_BIG, 1, scale=1, random_orientation=self.random_orientation)
        # generate yellow blocks
        pb.changeVisualShape(handle_cube[0].object_id, -1, rgbaColor=[1, 1, 0, 1])
        pb.changeVisualShape(handle_triangle[0].object_id, -1, rgbaColor=[1, 1, 0, 1])

        # change workspace color
        left_rgba, right_rgba = self.randSampleGray()
        print(left_rgba, right_rgba)
        pb.changeVisualShape(self.ws_id[0], -1, rgbaColor=left_rgba)
        pb.changeVisualShape(self.ws_id[1], -1, rgbaColor=right_rgba)

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
    obj_pos_cube = self.objects[0].getPosition()[:2]
    obj_pos_tri = self.objects[1].getPosition()[:2]
    sort_cube = abs(obj_pos_cube[1] - self.goal_pos_cube[1]) < 0.02
    sort_tri = abs(obj_pos_tri[1] - self.goal_pos_tri[1]) < 0.02
    return sort_cube and sort_tri


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


def createCloseLoopBlockColorSortEnv(config):
  return CloseLoopBlockColorSortEnv(config)


if __name__ == '__main__':
  env = CloseLoopBlockColorSortEnv({'seed': 0, 'render': True, 'workspace_option': 'custom'})


  planner = CloseLoopBlockArrangingPlanner(env, {})
  env.reset()

  while True:
    action = planner.getNextAction()
    (state, obs, in_hands), reward, done = env.step(action)

    if done:
      env.reset()