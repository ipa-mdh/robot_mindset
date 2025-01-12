from loguru import logger
import json

from nicegui import ui, app

from gui import theme
from gui.loguru_sink import LoguruSink
from gui.message import message

# Build the UI
def content() -> None:
    with ui.timeline(side='right').classes('w-2/3 max-w-[400px]'):
        ui.timeline_entry('Auswahl von Einheiten hinzugefügt.',
                        title='Release 0.4.1',
                        subtitle='16.9.2024',
                        icon='rocket')
        ui.timeline_entry('Max started to implement the Robot Mindest web interface..',
                        title='Initial commit',
                        subtitle='12.1.2025')