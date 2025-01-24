from dataclasses import dataclass
from pathlib import Path
import json
from typing import Dict, Any, Callable, List
from datetime import datetime

from loguru import logger
from nicegui import ui

from robot_mindset.utils.config_data import ConfigData

# ---------------------------------------------------------------------
# UI Element Creation Functions
# ---------------------------------------------------------------------
class WeldSeamSelector:
    def __init__(self, data: ConfigData):
        self.data = data
        
        self.create()
        
    def create(self):
        ui.label("Test")
    
# ---------------------------------------------------------------------
# Assemble the UI using the dataclass
# ---------------------------------------------------------------------
def content(data: ConfigData) -> None:
        
    WeldSeamSelector(data)
    
