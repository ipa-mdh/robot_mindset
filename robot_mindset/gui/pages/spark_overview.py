from loguru import logger
import json

from nicegui import ui, app

from robot_mindset.gui import theme
from robot_mindset.gui.loguru_sink import LoguruSink
from robot_mindset.gui.message import message

cards_data = {}

# Load data from JSON file
def load_config(share_dir):
    with open(share_dir / 'config/modules.json', 'r') as file:
        global cards_data
        cards_data = json.load(file)

# Save changes to JSON file
def save_changes():
    with open('config/modules_out.json', 'w') as file:
        global cards_data
        json.dump(cards_data, file, indent=4)
    ui.notify('Changes saved successfully!')

# Reload data and rebuild the UI
def reload_data(share_dir):
    load_config(share_dir)
    ui.notify('Data reloaded successfully!')

def create_card(card):
    def update_card_data(data, card=card):
        card["parameters"] = data.content['json']
    ui.add_css("""
        .my-json-editor {
            /* define a custom theme color */
            --jse-theme-color: #000000;
            --jse-theme-color-highlight: #687177;
        }
    """)
    editor = ui.json_editor({'content': {'json': card['parameters']},
                                'mode': "tree"},
                            on_change=update_card_data) \
                                .classes('w-full my-json-editor')

# Build the UI
def content(share_dir) -> None:
    load_config(share_dir)
    with ui.grid(columns=2).classes('w-full gap-4'):
        for card in cards_data:
            with ui.card().classes('w-full'):
                ui.label(f"Name: {card['name']}")
                ui.label(f"Package: {card['package']}")
                with ui.dialog() as dialog, ui.card():
                    create_card(card)
                    ui.button("Close", on_click=dialog.close).classes('ml-auto')
                
                with ui.expansion(f"Parameters").classes('w-full'):
                    create_card(card)
                ui.button(icon='settings', on_click=dialog.open).classes('ml-auto')

    # Add a save button
    ui.button("Save Changes", on_click=save_changes)

    # Add a button to load the data
    ui.button("Load Data", on_click=load_config)