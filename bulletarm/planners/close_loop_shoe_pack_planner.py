import numpy as np
from bulletarm.planners.close_loop_planner import CloseLoopPlanner
from bulletarm.pybullet.utils import transformations
from bulletarm.pybullet.objects.shoe_rack import ShoeRack

class CloseLoopShoePackPlanner(CloseLoopPlanner):
  def __init__(self, env, config):
    super().__init__(env, config)

    self.current_target = None
    self.stage = 0

    self.pre_grasp_pos_left = self.env.workspace.mean(1)
    self.grasp_pos_left = self.env.workspace.mean(1)
    self.post_grasp_pos_left = self.env.workspace.mean(1)
    self.release_pos_left = self.env.workspace.mean(1)
    self.pick_rot_left = 0

    self.pre_grasp_pos_right = self.env.workspace.mean(1)
    self.grasp_pos_right = self.env.workspace.mean(1)
    self.post_grasp_pos_right = self.env.workspace.mean(1)
    self.release_pos_right = self.env.workspace.mean(1)
    self.pick_rot_right = 0

    self.place_rot = 0

  def getNextActionToCurrentTarget(self):
    x, y, z, r = self.getActionByGoalPose(self.current_target[0], self.current_target[1])
    if np.all(np.abs([x, y, z]) < self.dpos) and (not self.random_orientation or np.abs(r) < self.drot):
      p = self.current_target[3]
      self.current_target = None
    else:
      p = self.current_target[2]
    return self.env._encodeAction(p, x, y, z, r)

  def setWaypoints(self):
    shoe_left_pos = self.env.objects[1].getPosition()
    shoe_right_pos = self.env.objects[2].getPosition()
    shoe_left_rot = transformations.euler_from_quaternion(self.env.objects[1].getRotation())[2]
    shoe_right_rot = transformations.euler_from_quaternion(self.env.objects[2].getRotation())[2]

    shoe_rack_pos_left = ShoeRack.getLeftPose(self.env.objects[0])[0]
    shoe_rack_pos_right = ShoeRack.getRightPose(self.env.objects[0])[0]
    shoe_rack_rot = transformations.euler_from_quaternion(self.env.objects[0].getRotation())[2]

    self.pick_rot_left = shoe_left_rot
    self.pick_rot_right = shoe_right_rot
    self.place_rot = shoe_rack_rot - np.pi / 2
    # print('L: ', self.pick_rot_left)
    # print('R: ', self.pick_rot_right)
    # print('P: ', self.place_rot)
    # print('----------------------------------------')

    # if self.pick_rot_left >= np.pi:
    #   print('----------------------')
    # elif self.pick_rot_right >= np.pi:
    #   print('~~~~~~~~~~~~~~~~~~~~~~')
    # elif self.place_rot >= np.pi:
    #   print('!!!!!!!!!!!!!!!!!!!!!!')

    self.pre_grasp_pos_left = [shoe_left_pos[0], shoe_left_pos[1], shoe_left_pos[2] + 0.1]
    self.grasp_pos_left = [shoe_left_pos[0], shoe_left_pos[1], shoe_left_pos[2] + 0.005]
    self.post_grasp_pos_left = [shoe_left_pos[0], shoe_left_pos[1], shoe_left_pos[2] + 0.1]
    self.release_pos_left = [shoe_rack_pos_left[0], shoe_rack_pos_left[1], shoe_rack_pos_left[2] + 0.05]

    self.pre_grasp_pos_right = [shoe_right_pos[0], shoe_right_pos[1], shoe_right_pos[2] + 0.1]
    self.grasp_pos_right = [shoe_right_pos[0], shoe_right_pos[1], shoe_right_pos[2] + 0.005]
    self.post_grasp_pos_right = [shoe_right_pos[0], shoe_right_pos[1], shoe_right_pos[2] + 0.1]
    self.release_pos_right = [shoe_rack_pos_right[0], shoe_rack_pos_right[1], shoe_rack_pos_right[2] + 0.05]

  def setNewTarget(self):
    if self.stage == 0:   #arrive left shoe
      self.setWaypoints()
      self.current_target = (self.pre_grasp_pos_left, self.pick_rot_left, 1, 1)
      self.stage = 1
    elif self.stage == 1:   #grasp left shoe
      self.current_target = (self.grasp_pos_left, self.pick_rot_left, 1, 0)
      self.stage = 2
    elif self.stage == 2:   #lift left shoe
      self.current_target = (self.post_grasp_pos_left, self.pick_rot_left, 0, 0)
      self.stage = 3
    elif self.stage == 3:   #release left shoe
      self.current_target = (self.release_pos_left, self.place_rot, 0, 1)
      self.stage = 4
    elif self.stage == 4:   #arrive right shoe
      self.current_target = (self.pre_grasp_pos_right, self.pick_rot_right, 1, 1)
      self.stage = 5
    elif self.stage == 5:   #grasp right shoe
      self.current_target = (self.grasp_pos_right, self.pick_rot_right, 1, 0)
      self.stage = 6
    elif self.stage == 6:   #lift right shoe
      self.current_target = (self.post_grasp_pos_right, self.pick_rot_right, 0, 0)
      self.stage = 7
    elif self.stage == 7:   #release right shoe
      self.current_target = (self.release_pos_right, self.place_rot, 0, 1)
      self.stage = 0

  def getNextAction(self):
    if self.env.current_episode_steps == 1:
      self.current_target = None
      self.stage = 0
    if self.current_target is not None:
      return self.getNextActionToCurrentTarget()
    else:
      self.setNewTarget()
      return self.getNextActionToCurrentTarget()

  def getStepsLeft(self):
    return 100
