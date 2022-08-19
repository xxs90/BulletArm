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
                self._generateShapes(constants.SHOE_LEFT, 1, scale=0.45, random_orientation=self.random_orientation)
                self._generateShapes(constants.SHOE_RIGHT, 1, scale=0.45, random_orientation=self.random_orientation)
                self._generateShapes(constants.SHOE_RACK, 1, scale=0.4,
                                     random_orientation=self.random_orientation)
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
        # if not self.objects[0].isTouching(self.objects[1]):
        #   return False

        # getAABB in a list of vec3, aabbMin(x, y, z) && aabbMax(x, y, z)
        # shoe_left: [(0.45362735534399207, 0.07344489751894974, -0.003995024502013195),
        #             (0.5688021905454737, 0.26293470104143674, 0.07533000901011326)]
        # shoe_right: [(0.438390291309848, -0.1566459698801372, -0.004242508927139052),
        #              (0.5799988931674707, 0.033824630488839075, 0.07599488523946032)]
        # shoe_rack: [(0.22149888464697737, -0.4151326071068406, -0.003207306106470992),
        #             (0.6068047642808458, -0.24561970277595596, 0.17528780100153887)]
        # the shoe size is (x = 0.12, y = 0.19, z = 0.08)
        # the shoe rack size is (x = 0.8, y = 0.65, z = 0.17)
        print('shoe_left: ', self.objects[0].getBoundingBox())
        print('shoe_right: ', self.objects[1].getBoundingBox())
        print('shoe_rack: ', self.objects[2].getBoundingBox())

        shoe_left_pos = self.objects[0].getPosition()[:2]
        shoe_right_pos = self.objects[1].getPosition()[:2]
        shoe_rack_pos = self.objects[2].getPosition()[:2]
        # return np.linalg.norm(np.array(shoe_left_pos) - np.array(shoe_rack_pos)) < 0.03 and \
        #        np.linalg.norm(np.array(shoe_right_pos) - np.array(shoe_rack_pos)) < 0.03
        return False

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
    from bulletarm.planners.close_loop_shoe_pack_planner import CloseLoopShoePackPlanner

    env = CloseLoopShoePack({'seed': 0, 'workspace': np.array([[0.2, 0.8], [-0.3, 0.3], [0, 1]]), 'render': True})
    planner = CloseLoopShoePackPlanner(env, {})
    env.reset()
    while True:
        action = planner.getNextAction()
        (state, obs, in_hands), reward, done = env.step(action)
        if done:
            env.reset()
