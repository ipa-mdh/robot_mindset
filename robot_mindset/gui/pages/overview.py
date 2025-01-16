from pathlib import Path
from loguru import logger
import json

from nicegui import ui, app

from robot_mindset.gui import theme
from robot_mindset.gui.loguru_sink import LoguruSink
from robot_mindset.gui.message import message

cards_data = {}

# Load data from JSON file
def load_config(share_dir):
    with open(f'{share_dir}/config/overview.json', 'r') as file:
        global cards_data
        cards_data = json.load(file)

    # with open('robot_mindset/robot_mindset/config/overview.json', 'r') as file:
    #     global cards_data
    #     cards_data = json.load(file)

def create_card(label, icon, link, share_dir):
    icon = Path(share_dir) / icon
    with ui.card().classes('w-full cursor-pointer') as card:
        ui.label(label).classes('w-full text-center')
        ui.image(icon).classes('w-full')
        card.on('click', lambda: ui.navigate.to(link))

# Build the UI
def content(share_dir) -> None:
    load_config(share_dir)
    
    num_elements = len(cards_data)
    
    # Dynamically adjust columns based on the number of elements
    columns = 2
    if num_elements <= 4:
        columns = 2
    elif num_elements <= 9:
        columns = 3
    else:
        columns = 4
        
    with ui.grid(columns=columns).classes('gap-4'):
        for card in cards_data:
            create_card(card['label'], card['icon'], card['link'], share_dir)
            # with ui.card().classes('w-32 cursor-pointer') as card:
            #     ui.label('Spark Overview').classes('w-full text-center')
            #     card.on('click', lambda: ui.navigate.to('/spark-overview')  )  # Change the link to your target URL
            #     ui.image('image/settings.svg').classes('w-full')  # Replace with your image URL
