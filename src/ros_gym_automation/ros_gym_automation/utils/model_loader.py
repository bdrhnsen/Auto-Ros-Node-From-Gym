#!/usr/bin/env python3
from typing import Any, Dict, Optional, Type
import os
import json
import importlib
from pathlib import Path
from ..core.agent_base import AgentBase


class ModelLoader:
    """Utility for loading RL models and configurations."""
    
    @staticmethod
    def load_model(model_path: str, agent_class: Type[AgentBase],
                  config_path: Optional[str] = None) -> AgentBase:
        """Load a model and its configuration.
        
        Args:
            model_path: Path to the saved model
            agent_class: Class of the agent to instantiate
            config_path: Optional path to configuration file
            
        Returns:
            Instantiated agent with loaded model
        """
        # Load configuration if provided
        config = {}
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = json.load(f)
                
        # Create agent instance
        agent = agent_class(config=config)
        
        # Load the model
        agent.load_model(model_path)
        
        return agent
        
    @staticmethod
    def load_agent_class(agent_type: str) -> Type[AgentBase]:
        """Dynamically load an agent class.
        
        Args:
            agent_type: Fully qualified class name (e.g., 'package.module.Class')
            
        Returns:
            Agent class
        """
        try:
            module_path, class_name = agent_type.rsplit('.', 1)
            module = importlib.import_module(module_path)
            return getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            raise ImportError(f"Could not load agent class {agent_type}: {e}") 