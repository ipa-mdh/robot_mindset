import math
from pathlib import Path
from loguru import logger
from nicegui import ui, app

from robot_mindset.gui.loguru_sink import LoguruSink
from robot_mindset.gui.message import message


from TROS import Teil2

nicegui_sink = LoguruSink()
logger.add(nicegui_sink, level="INFO",
        format="{time:YYYY:MM:DD HH:mm:ss!UTC} | {name} | {message}",
        backtrace=True,
        diagnose=True)

def set_default_input():
    app.storage.user['lam_nm'] =  550.0
    app.storage.user['P_value'] =  5.0
    app.storage.user['P_unit'] =  "-3"
    app.storage.user['phi_value'] =  5.0
    app.storage.user['phi_unit'] =  "grad"
    app.storage.user['phi_x_value'] =  5.0
    app.storage.user['phi_x_unit'] =  "grad"
    app.storage.user['phi_y_value'] =  5.0
    app.storage.user['phi_y_unit'] =  "grad"
    app.storage.user['alpha_value'] =  1.5
    app.storage.user['alpha_unit'] =  "mrad"
    app.storage.user['exposure_time'] =  "100.00"
    app.storage.user['pulsed'] =  False
    app.storage.user['pulsed_data'] =  {}
    app.storage.user['pulse_frequency_value'] =  10.0
    app.storage.user['pulse_frequency_unit'] =  "0"
    app.storage.user['pulse_on_duration_value'] = 10.0
    app.storage.user['pulse_on_duration_unit'] = "-3"
    app.storage.user['laser_shape'] = 'gauss'
    app.storage.user['trace'] = ""

def get_angle(value_key, unit_key):
    rv = 0.0
    
    if app.storage.user.get(unit_key, "grad") == "grad":
        rv = math.radians(float(app.storage.user.get(value_key, 0.0)))
    elif app.storage.user.get(unit_key, "grad") == "mrad":
        rv = float(app.storage.user.get(value_key, 0.0)) * math.pow(10,-3)
    else:
        rv = 0.0015
        ui.notify(f"Unknown unit for {unit_key}. Using default value 1.5 mrad", type='error')
        
    return rv

def get_input():
    
    alpha = get_angle('alpha_value', 'alpha_unit')
    phi = get_angle('phi_value', 'phi_unit')
    phi_x = get_angle('phi_x_value', 'phi_x_unit')
    phi_y = get_angle('phi_y_value', 'phi_y_unit')
    
    d = {
        "lam": float(app.storage.user.get('lam_nm', 0.0)) * math.pow(10,-9),
        "P": float(app.storage.user.get('P_value', 0.0)) * math.pow(10, int(app.storage.user.get('P_unit', "0"))),
        "exposure_time": float(app.storage.user.get('exposure_time', "100.00")),
        "phi": phi,
        "phi_x": phi_x,
        "phi_y": phi_y,
        "alpha": alpha,
        "pulsed": app.storage.user.get('pulsed', False),
        "f_pulse": float(app.storage.user.get('pulse_frequency_value', 10.0)) * math.pow(10, int(app.storage.user.get('pulse_frequency_unit', 0))),
        "t_on": float(app.storage.user.get('pulse_on_duration_value' , 0.0)) * math.pow(10, int(app.storage.user.get('pulse_on_duration_unit' , 0))),
    }
    return d

def get_rows_pulsed_laser_intermediate_results():
    app.storage.user.get('pulsed_data', {})
    
    rows = []
    input_data = get_input()
    
    for key, value in app.storage.user.get('pulsed_data', {}).items():
        logger.info(f"key: {key}, value: {value}")
        nohd = Teil2.get_NOHD_gauss(P=input_data["P"], E_EGW=value, phi=input_data["phi"])
        rows.append({'case': key, 'egw_value': f"{value:.2f}", 'nohd_value': f"{nohd:.3f}"})
    return rows
    
def calc_nohd():
    input = get_input()
    P = input['P']
    E_EGW = app.storage.user.get('E_EGW', 0.0)

    try:
        shape = app.storage.user.get('laser_shape', 'gauss')
        if shape == 'gauss':
            phi = input['phi']
            NOHD = Teil2.get_NOHD_gauss(P=P, E_EGW=E_EGW, phi=phi)
        elif shape == 'line':
            phi_x = input['phi_x']
            phi_y = input['phi_y']
            NOHD = Teil2.get_NOHD_line(P=P, E_EGW=E_EGW, phi_x=phi_x, phi_y=phi_y)
        else:
            raise ValueError(f"Unknown laser shape: {shape}")

        ui.notify('NOHD wurde berechnet', type='positive')
    except Exception as e:
            NOHD = -1
            ui.notify(e, type='negative')
    finally:
        app.storage.user['NOHD'] = NOHD
        
def calc():
    input = get_input()
    nicegui_sink.reset_storage()
    

    logger.info("--- Input ---")
    logger.info(f"{input}")
    
    try:
        if input['pulsed']:
            E_EGW, list = Teil2.get_E_EGW_pulsed_laser(lam=input['lam'],
                                                 t=input['exposure_time'],
                                                 t_on=input['t_on'],
                                                 f_pulse=input['f_pulse'],
                                                 alpha=input['alpha'],
                                                 trace=True)
            app.storage.user['pulsed_data'] = list
            table_pulsed_laser_intermediate_results.update_rows(get_rows_pulsed_laser_intermediate_results())
        else:
            E_EGW = Teil2.get_E_EGW_continous_wave(lam=input['lam'],
                                                   t=input['exposure_time'],
                                                   alpha=input['alpha'],
                                                   trace=True)
    except Exception as e:
        E_EGW= -1
        ui.notify(e, type='negative')
        table_pulsed_laser_intermediate_results.update_rows([])
    finally:
        app.storage.user['E_EGW'] = E_EGW

    calc_nohd()
    app.storage.user['trace'] = "".join(app.storage.user.get("loguru_storage", ""))

# def create_report(self):
#     nicegui_sink.get_storage()

def init():
    app.storage.user['alpha_tooltip'] = "Winkelausdehnung der Lichtquelle kleiner als 1.5 mrad gilt als eine Punktlichtquelle"
    set_default_input()

def content() -> None:
    init()
    
    with ui.grid(columns=1).classes('gap-2 justify-self-center').style('width: 100%; max-width: 500px;'):
        with ui.card().classes('col-span-1'):
            ui.number(label="Wellenlänge / nm", min=100, max=2600).bind_value(app.storage.user, 'lam_nm') \
                            .classes('w-full')
            with ui.row().classes('w-full'):
                ui.number(label="Leistung", min=0).bind_value(app.storage.user, 'P_value') \
                                .classes('flex-grow')
                ui.select(options={"6": "MW", "3": "kW", "0": "W", "-3": "mW", "-6": "µW"}, value="-3").bind_value(app.storage.user, 'P_unit')\
                    .classes('w-15')

            def handle_selection_exposure_time(buffer):
                key = "{:.2f}".format(float(buffer.value))
                d = Teil2.get_exposure_time_description_dict()
                if key in d:
                    app.storage.user['exposure_time_description'] = d[key]
                else:
                    app.storage.user['exposure_time_description'] = "non default value"

            with ui.select(options=Teil2.get_exposure_time_value_list(), label='Expositionsdauer / s',
                            new_value_mode="add-unique",
                            on_change=handle_selection_exposure_time).bind_value(app.storage.user, 'exposure_time') \
                            .classes('w-full') as ui_exposure_time:
                ui.tooltip(text="").bind_text_from(app.storage.user, 'exposure_time_description').classes('text-base')

            def pulsed_changed():
                pulsed_enabled = app.storage.user.get('pulsed', False)
                if pulsed_enabled and ui_exposure_time.value:
                    lam = get_input()['lam']
                    alpha = get_input()['alpha']
                    T_max = Teil2.get_T_max(lam, alpha)
                    if float(ui_exposure_time.value) > T_max:
                        app.storage.user['exposure_time'] = f"{T_max:.2f}"
                        ui.notify(f"TROS Teil 2 Seite 56 Hinweis 2: Expositionsdauer wird auf {T_max:.2f} s gesetzt.",
                                type="warning")
            with ui.row().classes('w-full'):
                ui.checkbox("Pulsed Laser", on_change=pulsed_changed).bind_value(app.storage.user, 'pulsed')
            with ui.row().bind_visibility_from(app.storage.user, 'pulsed').classes('w-full'):
                ui.number(label="Pulslänge").bind_value(app.storage.user, 'pulse_on_duration_value') \
                    .classes('flex-grow')
                ui.select(options={"0": "s", "-3": "ms", "-6": "µs", "-9": "ns", "-12": "ps", "-15": "fs"}, value="-3").bind_value(app.storage.user, 'pulse_on_duration_unit')\
                    .classes('w-15')
            with ui.row().bind_visibility_from(app.storage.user, 'pulsed').classes('w-full'):
                ui.number(label="Frequenz").bind_value(app.storage.user, 'pulse_frequency_value')\
                    .classes('flex-grow')
                ui.select(options={"0": "Hz", "3": "kHz", "6": "MHz"}, value="0").bind_value(app.storage.user, 'pulse_frequency_unit')\
                    .classes('w-15')
                    
        with ui.card().classes('col-span-1'):

            def update_laser_shape(tab):
                app.storage.user['laser_shape'] = tab.value
            with ui.tabs().classes('w-full') as tabs_laser_shape:
                laser_shape_gauss = ui.tab('gauss', label='Gaußstrahl')
                laser_shape_line = ui.tab('line', label='Laserlinie')
            
            with ui.tab_panels(tabs_laser_shape, value=laser_shape_gauss, on_change=update_laser_shape).classes('w-full'):
                with ui.tab_panel(laser_shape_gauss).classes('w-full'):
                    with ui.row().classes('w-full'):
                        ui.number(label="Strahldivergenz", min=0.000001, max=360).bind_value(app.storage.user, 'phi_value') \
                                        .classes('flex-grow')
                        ui.select(options={"grad": "Grad", "mrad": "mrad"}, value="grad").bind_value(app.storage.user, 'phi_unit')\
                            .classes('w-15')
                    with ui.row().classes('w-full'):
                        with ui.number(label="Winkelausdehnung", min=0.00001).bind_value(app.storage.user, 'alpha_value') \
                                        .classes('flex-grow'):
                            ui.tooltip(text="").bind_text_from(app.storage.user, 'alpha_tooltip').classes('text-base')
                        ui.select(options={"grad": "Grad", "mrad": "mrad"}, value="mrad").bind_value(app.storage.user, 'alpha_unit')\
                            .classes('w-15')
                with ui.tab_panel(laser_shape_line).classes('w-full'):
                    with ui.row().classes('w-full'):
                        ui.number(label="Strahldivergenz x", min=0.000001, max=360).bind_value(app.storage.user, 'phi_x_value') \
                                        .classes('flex-grow')
                        ui.select(options={"grad": "Grad", "mrad": "mrad"}, value="grad").bind_value(app.storage.user, 'phi_x_unit')\
                            .classes('w-15')
                        ui.number(label="Strahldivergenz y", min=0.000001, max=360).bind_value(app.storage.user, 'phi_y_value') \
                                        .classes('flex-grow')
                        ui.select(options={"grad": "Grad", "mrad": "mrad"}, value="grad").bind_value(app.storage.user, 'phi_y_unit')\
                            .classes('w-15')
                    with ui.row().classes('w-full'):
                        with ui.number(label="Winkelausdehnung", min=0.0000001).bind_value(app.storage.user, 'alpha_value') \
                                        .classes('flex-grow'):
                            ui.tooltip(text="").bind_text_from(app.storage.user, 'alpha_tooltip').classes('text-base')
                        ui.select(options={"grad": "Grad", "mrad": "mrad"}, value="mrad").bind_value(app.storage.user, 'alpha_unit')\
                            .classes('w-15')
                            
        ui.button('Berechnen', on_click=calc).classes('w-full')
        
        with ui.row().classes('w-full'):
            ui.number(label = "Expositionsgrenzwert / W/m^2",
                                            min = 0.000001,
                                            format="%.3f",
                                            on_change =calc_nohd) \
                .bind_value(app.storage.user, 'E_EGW') \
                .classes('flex-grow')
            ui.number(label="Augensicherheitsabstand NOHD / m", format="%.3f")\
                .bind_value(app.storage.user, 'NOHD') \
                .classes('flex-grow')
                
            with ui.expansion('Zwischenergebnisse nach Anhang 4.1 (3)').classes('w-full').bind_visibility_from(app.storage.user, 'pulsed'):
                global table_pulsed_laser_intermediate_results
                            
                columns = [
                    {'name': '', 'label': 'Fall', 'field': 'case',      'required': True, 'align': 'left'},
                    {'name': '', 'label': 'E_EGW / W/m²',  'field': 'egw_value', 'sortable': True},
                    {'name': '', 'label': 'NOHD/ m', 'field': 'nohd_value','sortable': True}
                ]
                table_pulsed_laser_intermediate_results = ui.table(columns = columns, rows=[], row_key='name').classes('w-full')
    

    # nicegui_error_sink = LoguruSink()
    # nicegui_error_sink.set_callback(ui.notify, type='warning')
    # id = logger.add(nicegui_error_sink, level="ERROR")
    # logger.remove(id)
    