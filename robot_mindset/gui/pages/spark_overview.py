from dataclasses import dataclass
from pathlib import Path
import json
from typing import Dict, Any, Callable, List
from datetime import datetime

from loguru import logger
from nicegui import ui

from robot_mindset.utils.config_data import ConfigData

@dataclass
class CardData:
    name: str
    package: str
    parameters: Dict

# ---------------------------------------------------------------------
# UI Element Creation Functions
# ---------------------------------------------------------------------
class SparkList:
    def __init__(self, data: ConfigData, on_select=None):
        self.data = data
        self.on_select = on_select
        self.table = None
        
        self.table = self.create()
    
    def create(self):
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
        table.on_select(self.on_select)
        
        ui.checkbox('auto refresh').bind_value(self.data, 'use_heardbeat')
        
        ui.button("test", on_click=self.update)
        
        return table
        
    def update(self) -> None:
        if self.table:
            rows = []
            for idx, spark_name in enumerate(self.data.spark_list):
                buffer = {
                    'node_name': spark_name,
                    'date': datetime.now().strftime('%c'),
                    'id': idx
                }
                rows.append(buffer)
            self.table.rows = rows

def create_json_editor(json_dict: Dict,
                       style="max-height: 400px; overflow-y: auto;",
                       classes='w-full my-json-editor',
                       on_change=None) -> None:
    def update_card_data(editor_data) -> None:
        if callable(on_change):
            on_change(editor_data.content['json'])
    
    editor = ui.json_editor(
        {'content': {'json': json_dict}, 'mode': "tree"},
        on_change=update_card_data,
    ).classes(classes).style(style)
    
    return editor

class SparkCard:
    """
    A card element for a Spark node.
    """
    def __init__(self, data: CardData, on_change=None, on_save=None):
        self.data = data
        self.on_change = on_change
        self.on_save = on_save
        
        self.card_update = self.create(data)
        
    def create(self, data: CardData) -> None:
        # def create_card(card: Dict, data: ConfigData) -> None:
        with ui.card().classes('w-full col-span-1'):
            ui.label(f"Name: {data.name}")
            ui.label(f"Package: {data.package}")
            
            def update_editor_dialog(editor_data: Dict) -> None:
                data.parameters = editor_data
                editor_dialog.properties['content']['json'] = editor_data
                editor_dialog.update()
                
            def update_editor_expansion(editor_data: Dict) -> None:
                data.parameters = editor_data
                editor_expansion.properties['content']['json'] = editor_data
                editor_expansion.update()
                
            def update_editor() -> None:
                update_editor_dialog(data.parameters)
                update_editor_expansion(data.parameters)
            
            with ui.dialog() as dialog, ui.card():
                editor_dialog = create_json_editor(data.parameters,
                                                   style="max-height: 1200px; overflow-y: auto;",
                                                   on_change=update_editor_expansion)
                ui.button("Close", on_click=dialog.close).classes('ml-auto')
                
            with ui.expansion("Parameters").classes('w-full'):
                editor_expansion = create_json_editor(data.parameters,
                                                      on_change=update_editor_dialog)
                
            with ui.row().classes('absolute top-0 right-0 m-2'):
                def on_click_save() -> None:
                    update_editor()
                    if callable(self.on_save):
                        self.on_save(data.name, data.parameters)
                        
                    ui.notify(f"Uploaded {data.name} parameters", type='positive', position='top')
                
                def buffer():
                    update_editor_expansion(data.parameters)
                    dialog.open()
                
                ui.button(icon="cloud_upload",
                        on_click=on_click_save)
                ui.button(icon='settings', on_click=buffer)
            # ui.button("Update", on_click=update_editor)
            
        return update_editor

    def update(self) -> None:
        self.card_update()

class SparkCardGrid:
    """
    A grid of SparkCard elements.
    """
    def __init__(self, data: ConfigData):
        self.data = data
        
        self.grid = self.create()
        self.update_grid_list = []
        
    def create(self) -> None:
        return ui.grid(columns=2).classes('w-full gap-4')
    
    def update(self, selected) -> None:
        with self.grid: 
            self.grid.clear()
            if selected:
                self.update_grid_list.clear()
                for entry in selected:
                    name = entry.get('node_name')
                    spark_list = self.data.spark_list
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
                        cd = CardData(spark['node_name'], spark['node_name'], params)
                        def on_save(node_name: str, parameters: Dict) -> None:
                            self.data.set_spark_parameters_fnc(node_name, 'parameters', parameters)
                        spark_card = SparkCard(cd, on_save=on_save)
                        
                        self.update_grid_list.append(spark_card.update)

# ---------------------------------------------------------------------
# Assemble the UI using the dataclass
# ---------------------------------------------------------------------
def content(data: ConfigData) -> None:
    def on_spark_selected(selected) -> None:
        spg.update(selected.selection)
        
    with ui.splitter(value=20, limits=(5, 80)).classes('w-full') as splitter:
        with splitter.before:
            with ui.column().classes('w-full').style('padding: 15px;'):
                sp = SparkList(data, on_select=on_spark_selected)
        with splitter.after:
            with ui.column().classes('w-full').style('padding: 15px;'):
                spg = SparkCardGrid(data)
    
    # connect the update function to the ConfigData instance
    def update() -> None:
        sp.update()
    
    data.spark_list_callback = update
