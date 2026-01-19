import torch
import os

model_path = '/home/ever/ros2_ws/src/go2_ros_gazebo/checkpoints/model_7850.pt'
print(f"Inspecting: {model_path}")

try:
    data = torch.load(model_path, map_location='cpu')
    print(f"Type: {type(data)}")
    if isinstance(data, dict):
        print(f"Keys: {data.keys()}")
        if 'model_state_dict' in data:
             print("Found 'model_state_dict'. Inspecting first few keys...")
             for k in list(data['model_state_dict'].keys())[:5]:
                 print(f"  {k}: {data['model_state_dict'][k].shape}")
    else:
        print("Data is not a dict.")
except Exception as e:
    print(f"Error loading with torch.load: {e}")
