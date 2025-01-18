from contextlib import contextmanager
from nicegui import ui, app

from robot_mindset.gui.menu import menu

@contextmanager
def frame(navtitle: str, share_dir, footer_generator=None):
    """Custom page frame to share the same styling and behavior across all pages"""
    ui.colors(primary='#000000',
              secondary='#005C7F',
              accent='#111111',
              positive='#10BC69',
              negative='#BC1063',
              warning='#E67614',
              info='#106CBC',
              dark='#1d1d1d',
              dark_page='#121212',
    )
    
    ui.add_css("""
        .my-json-editor {
            /* define a custom theme color */
            --jse-theme-color: #000000;
            --jse-theme-color-highlight: #687177;
        }
    """)
    
    dark = ui.dark_mode()
    
    with ui.header().classes('justify-between bg-white'):
        with ui.grid(columns=3).classes('gap-1 w-full wrap'):
            ui.interactive_image(share_dir / 'image/robot_mindset.svg', on_mouse=lambda: ui.navigate.to("/"), events=["mouseup"])\
                .classes('col-span-1 justify-self-start self-start min-w-[150px] max-w-[300px]')
            ui.label(navtitle).classes('col-span-1 justify-self-center text-h4 text-grey-8 nowrap whitespace-nowrap')
            with ui.row().classes('col-span-1 justify-self-end'):
                menu(dark)
        ui.separator()
    with ui.grid(columns=1).classes('w-full flex-grow justify-items-center'):
        yield

    if footer_generator:
        with ui.footer().style('background-color: #111B1E;').style('max-height: 250px; padding: 0px;'):
            footer_generator()