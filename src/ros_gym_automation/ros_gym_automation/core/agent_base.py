#!/usr/bin/env python3
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

class AgentBase(ABC):
    """Base class for reinforcement learning agents."""
    
    def __init__(self, config: Optional[Dict] = None):
        """Initialize the agent.
        
        Args:
            config: Optional configuration dictionary
        """
        self.config = config or {}
        self.model = None
        
    @abstractmethod
    def predict(self, observation: Any) -> Any:
        """Generate action based on observation.
        
        Args:
            observation: The current observation from the environment
            
        Returns:
            The action to take
        """
        pass
        
    @abstractmethod
    def load_model(self, path: str) -> None:
        """Load a pre-trained model.
        
        Args:
            path: Path to the model file
        """
        pass
        
    @abstractmethod
    def save_model(self, path: str) -> None:
        """Save the current model.
        
        Args:
            path: Path where to save the model
        """
        pass
        
    def update(self, observation: Any, action: Any, reward: float, 
               next_observation: Any, done: bool) -> Dict:
        """Update the agent's policy (for online learning).
        
        Args:
            observation: The observation that led to the action
            action: The action taken
            reward: The reward received
            next_observation: The resulting observation
            done: Whether the episode ended
            
        Returns:
            Dictionary containing training metrics
        """
        # Default implementation for non-learning agents
        return {} 