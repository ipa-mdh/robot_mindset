from pathlib import Path
import json
from typing import Dict, Any, Callable, List
from datetime import datetime

from loguru import logger
from nicegui import ui

from robot_mindset.utils.config_data import ConfigData

# A callback used to update the grid; it is set later by create_card_grid
update_grid_callback: Callable[[List[Dict]], None] = None

def upload_parameters(set_spark_parameters_fnc, node_name: str, parameters: Dict) -> None:
    if callable(set_spark_parameters_fnc):
        set_spark_parameters_fnc(node_name, 'parameters_file', parameters)
    else:
        logger.warning('No setter function provided')

# ---------------------------------------------------------------------
# UI Element Creation Functions
# ---------------------------------------------------------------------
def create_spark_list(data: ConfigData) -> None:
    list_of_sparks = data.spark_list
    # Ensure the config is loaded before creating the list
    # load_config(data.share_dir)
    
    table: ui.Table = None  # type: ignore

    def update_rows() -> None:
        if table:
            rows = []
            for idx, spark_name in enumerate(list_of_sparks):
                buffer = {
                    'node_name': spark_name,
                    'date': datetime.now().strftime('%c'),
                    'id': idx
                }
                rows.append(buffer)
            table.rows = rows
    
    # Attach the update_rows callback to the data if needed elsewhere.
    data.spark_list_callback = update_rows  # type: ignore

    columns = [
        {'name': 'node_name', 'label': 'Node', 'field': 'node_name',
         'align': 'left', 'sortable': True, 'filterable': True, 'visible': True},
        {'name': 'date', 'label': 'Date', 'field': 'date', 'align': 'left',
         'sortable': True, 'filterable': False, 'visible': False,
         'classes': 'hidden', 'headerClasses': 'hidden'}
    ]
    
    with ui.row().classes('w-full gap-4'):
        search_input = ui.input('Search by name').props('clearable').style('flex: 1;')
        with ui.button(icon='view_column'):
            with ui.menu(), ui.column().classes('gap-0 p-2'):
                for column in columns:
                    ui.switch(
                        column['label'],
                        value=column['visible'],
                        on_change=lambda e, column=column: toggle(column, e.value)
                    )
    
    table = ui.table(
        columns=columns,
        rows=[],
        selection='multiple',
        pagination=10
    ).classes('w-full').props('virtual-scroll')
    
    def toggle(column: Dict, visible: bool) -> None:
        column['classes'] = '' if visible else 'hidden'
        column['headerClasses'] = '' if visible else 'hidden'
        table.update()
    
    search_input.bind_value(table, 'filter')
    table.on_select(update_grid)
    
    ui.checkbox('auto refresh').bind_value(data, 'use_heardbeat')

def create_json_editor(card: Dict, style="max-height: 400px; overflow-y: auto;") -> None:
    def update_card_data(editor_data) -> None:
        # Assumes that the json_editor returns a dict with key 'json' inside 'content'
        card["parameters"] = editor_data.content['json']
    
    def get_json_editor_properties(card):
        return {'content': {'json': card.get('parameters', {})},
         'mode': "tree"}
    
    ui.json_editor(
        get_json_editor_properties(card),
        on_change=update_card_data,
    ).classes('w-full h-max-300 my-json-editor').style(style)

def update_grid(selected: Any) -> None:
    if callable(update_grid_callback):
        update_grid_callback(selected.selection)
    # ui.notify(selected.selection)

def create_card(card: Dict, data: ConfigData) -> None:
    with ui.card().classes('w-full col-span-1'):
        ui.label(f"Name: {card['name']}")
        ui.label(f"Package: {card['package']}")
        with ui.dialog() as dialog, ui.card():
            create_json_editor(card, style="max-height: 1200px; overflow-y: auto;")
            ui.button("Close", on_click=dialog.close).classes('ml-auto')
        with ui.expansion("Parameters").classes('w-full'):
            create_json_editor(card)
        with ui.row().classes('absolute top-0 right-0 m-2'):
            def on_click_save() -> None:
                upload_parameters(data.set_spark_parameters_fnc, card['name'], card['parameters'])
                ui.notify(f"Uploaded {card['name']} parameters", type='positive', position='top')
                
            ui.button(icon="cloud_upload",
                      on_click=on_click_save)
            ui.button(icon='settings', on_click=dialog.open)

def create_card_grid(data: ConfigData) -> None:
    global update_grid_callback
    
    grid = ui.grid(columns=2).classes('w-full gap-4')

    def update_grid(selected: Any) -> None:
        with grid: 
            grid.clear()
            if selected:
                for entry in selected:
                    name = entry.get('node_name')
                    spark_list = data.spark_list
                    if name in spark_list:
                        spark = spark_list[name]
                        if spark.get('parameters'):
                            params = json.loads(
                                spark['parameters']
                                .replace("False", "false")
                                .replace("True", "true")
                                .replace("'", '"')
                            )
                        else:
                            ui.notify(f'No parameters for {name} found', type='negative', position='top')
                            params = {}
                        create_card({
                            'name': spark['node_name'],
                            'package': spark['node_name'],
                            'parameters': params
                        }, data)
    
    update_grid_callback = update_grid  # type: ignore

# ---------------------------------------------------------------------
# Assemble the UI using the dataclass
# ---------------------------------------------------------------------
def content(data: ConfigData) -> None:
    with ui.splitter(value=20, limits=(5, 80)).classes('w-full') as splitter:
        with splitter.before:
            with ui.column().classes('w-full').style('padding: 15px;'):
                create_spark_list(data)
        with splitter.after:
            with ui.column().classes('w-full').style('padding: 15px;'):
                create_card_grid(data)
