import mujoco
import numpy as np
import cv2
import os

#os.environ["MUJOCO_GL"] = "egl"

def render(renderer, model, data, height=300, camera=-1):
  
  mujoco.mj_forward(model, data)
    
  renderer.update_scene(data, camera)
  # Show image
  img = renderer.render()
  img_cv = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
  cv2.imshow("Cam feed", img_cv)
  cv2.waitKey()
      

# Camera options
def create_camera() : 
  cam = mujoco.MjvCamera()
  mujoco.mjv_defaultCamera(cam)
  
  cam.elevation = -10
  cam.lookat = [0, 0, 1]
  cam.distance = 4
  cam.azimuth = 135

  return cam

# Arena
arena_xml = """
<mujoco>
  <visual>
    <headlight diffuse=".5 .5 .5" specular="1 1 1"/>
    <global offwidth="2048" offheight="1536"/>
    <quality shadowsize="8192"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="1 1 1" rgb2="1 1 1" width="10" height="10"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="1 1 1" rgb2="1 1 1" markrgb="0 0 0" width="400" height="400"/>
    <material name="groundplane" texture="groundplane" texrepeat="45 45" reflectance="0"/>
  </asset>

  <worldbody>
    <geom name="floor" size="10 10 0.1" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
"""

arena_spec = mujoco.MjSpec.from_string(arena_xml)
arena_model = arena_spec.compile()
arena_data = mujoco.MjData(arena_model)

# Get info from humanoid
human_file = "/home/ana/Software/mujoco/model/humanoid/humanoid100.xml"
human_spec = mujoco.MjSpec.from_file(human_file)
human_model = human_spec.compile()
human_data = mujoco.MjData(human_model)

# Another humanoid
human_2_spec = mujoco.MjSpec.from_file(human_file)
attachment_frame = human_spec.worldbody.add_frame(pos=[0, -1, 2])
attachment_frame.attach_body(human_2_spec.body('humanoid_torso'), 'a', 'b')

# Recompile
new_model, new_data = human_spec.recompile(human_model, human_data)


cam = create_camera()
renderer = mujoco.Renderer(human_model, 480, 640)

render(renderer, new_model, new_data, camera=cam)

#render(renderer, arena_model, arena_data, camera=cam)



