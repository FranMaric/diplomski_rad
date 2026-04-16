from openpi_client import image_tools
from openpi_client import websocket_client_policy
import cv2

# Outside of episode loop, initialize the policy client.
# Point to the host and port of the policy server (localhost and 8000 are the defaults).
client = websocket_client_policy.WebsocketClientPolicy(host="161.53.68.175", port=8000)

img = cv2.imread("top_view.png")


# Inside the episode loop, construct the observation.
# Resize images on the client side to minimize bandwidth / latency. Always return images in uint8 format.
# We provide utilities for resizing images + uint8 conversion so you match the training routines.
# The typical resize_size for pre-trained pi0 models is 224.
# Note that the proprioceptive `state` can be passed unnormalized, normalization will be handled on the server side.
observation = {
    "observation/exterior_image_1_left": image_tools.convert_to_uint8(
        image_tools.resize_with_pad(img, 224, 224)
    ),
    "observation/wrist_image_left": image_tools.convert_to_uint8(
        image_tools.resize_with_pad(img, 224, 224)
    ),
    "observation/joint_position": [-1.059033810400126, 0.9073768402902865, 0.2589026809603644, -0.8333167082836516, -0.0538165064950784, 3.1349696121613184, 1.9074394658406573],
    "observation/gripper_position": 0.0,
    "prompt": "Pick up the red pepper.",
}

# Call the policy server with the current observation.
# This returns an action chunk of shape (action_horizon, action_dim).
# Note that you typically only need to call the policy every N steps and execute steps
# from the predicted action chunk open-loop in the remaining steps.
action_chunk = client.infer(observation)["actions"]

print(action_chunk)
