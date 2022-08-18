import pybullet as pb
import numpy as np

from bulletarm.envs.close_loop_envs.close_loop_env import CloseLoopEnv
from bulletarm.pybullet.utils import constants
from bulletarm.planners.close_loop_block_in_bowl_planner import CloseLoopBlockInBowlPlanner
from bulletarm.pybullet.utils.constants import NoValidPositionException
from bulletarm.pybullet.equipments.tray import Tray


class CloseLoopShoePack(CloseLoopEnv):
    '''Close loop shoe packing task.

  The robot needs to pick up the shoes and place on the shoe rack in pairs.

  Args:
    config (dict): Initialization arguments for the env
  '''

    def __init__(self, config):
        if 'num_objects' not in config:
            config['num_objects'] = 2
        super().__init__(config)

    def reset(self):
        while True:
            self.resetPybulletWorkspace()
            try:
                self._generateShapes(constants.SHOE_LEFT, 1, scale=0.5, random_orientation=self.random_orientation)
                self._generateShapes(constants.SHOE_RIGHT, 1, scale=0.5, random_orientation=self.random_orientation)
                self._generateShapes(constants.SHOE_RACK, 1, scale=0.5, random_orientation=self.random_orientation)
                # self._generateShapes(constants.CUBE, 1, random_orientation=self.random_orientation, pos=[(0, 0, 0)])
                # self._generateShapes(constants.CUBE, 1, random_orientation=self.random_orientation, pos=[(1, 1, 0)])
            except NoValidPositionException as e:
                continue
            else:
                break
        return self._getObservation()

    def _checkTermination(self):
        # check if bowl is upright
        if not self._checkObjUpright(self.objects[1]):
            return False
        # check if bowl and block is touching each other
        if not self.objects[0].isTouching(self.objects[1]):
            return False
        block_pos = self.objects[0].getPosition()[:2]
        bowl_pos = self.objects[1].getPosition()[:2]
        return np.linalg.norm(np.array(block_pos) - np.array(bowl_pos)) < 0.03

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


def createCloseLoopShoePackEnv(config):
    return CloseLoopShoePack(config)


if __name__ == '__main__':
    # from bulletarm.planners.close_loop_block_in_bowl_planner import CloseLoopBlockInBowlPlanner

    env = CloseLoopShoePack({'seed': 0, 'workspace': np.array([[0.3, 0.7], [-0.5, 0.5], [0, 1]]), 'render': True})
    # planner = CloseLoopBlockInBowlPlanner(env, {})
    env.reset()
    # while True:
    #     action = planner.getNextAction()
    #     (state, obs, in_hands), reward, done = env.step(action)
    #     if done:
    #         env.reset()
