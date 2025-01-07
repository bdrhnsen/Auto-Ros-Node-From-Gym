#!/usr/bin/env python3
from typing import Dict, Any
from ..generators.env_parser import SpaceSpec


class ROSMsgGenerator:
    """Generator for ROS message type definitions based on environment specs."""
    
    @staticmethod
    def space_to_msg_type(space_spec: SpaceSpec) -> str:
        """Convert a space specification to appropriate ROS message type."""
        if space_spec.space_type == 'Box':
            if len(space_spec.shape) == 1:
                return 'Float32MultiArray'
            elif len(space_spec.shape) == 2:
                return 'Float32MultiArray'  # Could use sensor_msgs/Image for images
            else:
                raise ValueError(f"Unsupported Box space shape: {space_spec.shape}")
                
        elif space_spec.space_type == 'Discrete':
            return 'Int32'
            
        elif space_spec.space_type == 'MultiBinary':
            return 'BoolMultiArray'
            
        elif space_spec.space_type == 'MultiDiscrete':
            return 'Int32MultiArray'
            
        else:
            raise ValueError(f"Unsupported space type: {space_spec.space_type}")
    
    @staticmethod
    def generate_conversion_code(space_spec: SpaceSpec, direction: str) -> str:
        """Generate code for converting between Gym and ROS types.
        
        Args:
            space_spec: The space specification
            direction: Either 'to_ros' or 'from_ros'
            
        Returns:
            String containing Python code for conversion
        """
        if direction == 'to_ros':
            if space_spec.space_type == 'Box':
                return """
                msg = Float32MultiArray()
                msg.data = observation.flatten().tolist()
                return msg
                """
            elif space_spec.space_type == 'Discrete':
                return """
                return Int32(data=observation)
                """
        elif direction == 'from_ros':
            if space_spec.space_type == 'Box':
                return """
                return np.array(msg.data).reshape(self.env_wrapper.observation_space.shape)
                """
            elif space_spec.space_type == 'Discrete':
                return """
                return msg.data
                """
        
        raise ValueError(f"Unsupported conversion: {space_spec.space_type} {direction}") 