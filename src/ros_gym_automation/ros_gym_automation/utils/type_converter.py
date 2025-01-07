#!/usr/bin/env python3
from typing import Any, Dict, List, Tuple, Union
import numpy as np
from gymnasium.spaces import Box, Discrete, MultiBinary, MultiDiscrete
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool
from geometry_msgs.msg import Vector3


class TypeConverter:
    """Utility class for converting between Gym and ROS2 types."""
    
    @staticmethod
    def gym_space_to_ros_type(space: Any) -> str:
        """Convert Gym space to corresponding ROS message type."""
        if isinstance(space, Box):
            return "Float32MultiArray"
        elif isinstance(space, Discrete):
            return "Int32"
        elif isinstance(space, MultiBinary):
            return "Bool"
        elif isinstance(space, MultiDiscrete):
            return "Int32MultiArray"
        else:
            raise ValueError(f"Unsupported space type: {type(space)}")
    
    @staticmethod
    def numpy_to_ros_msg(data: np.ndarray, msg_type: Any) -> Any:
        """Convert numpy array to ROS message."""
        if isinstance(msg_type, Float32MultiArray):
            msg = Float32MultiArray()
            msg.data = data.flatten().tolist()
            return msg
        elif isinstance(msg_type, Vector3):
            msg = Vector3()
            msg.x = float(data[0])
            msg.y = float(data[1])
            msg.z = float(data[2])
            return msg
        # Add more conversions as needed
        raise ValueError(f"Unsupported message type: {type(msg_type)}")
    
    @staticmethod
    def ros_msg_to_numpy(msg: Any) -> np.ndarray:
        """Convert ROS message to numpy array."""
        if isinstance(msg, Float32MultiArray):
            return np.array(msg.data)
        elif isinstance(msg, Vector3):
            return np.array([msg.x, msg.y, msg.z])
        # Add more conversions as needed
        raise ValueError(f"Unsupported message type: {type(msg)}") 