from nicegui import ui, app

from robot_mindset.gui import theme
from robot_mindset.gui.message import message
from robot_mindset.gui.pages import spark_overview, overview, changelog

# # Add custom CSS for the JSON editor
# ui.add_css("""
#   .my-json-editor {
#     /* define a custom theme color */
#     jse-theme-color: #383e42;
#     jse-theme-color-highlight: #687177;
#   }
# """)

def footer():
    with ui.expansion('Project: example').classes('w-full').style('margin: 0px; padding: 0px;'):
        with ui.scroll_area().style('max-height: 180px;'):
            ui.code(content="Project details", language='plaintext').classes('w-full')

def create(share_dir) -> None:
    @ui.page('/')
    def main_page() -> None:
        with theme.frame('', share_dir, footer):
            overview.content(share_dir)
            # with ui.column().classes('absolute-center items-center'):
            #     ui.label(f'Hello').classes('text-2xl')
            #     ui.link('b', '/b').classes('text-xl')
            #     # ui.button(on_click=logout, icon='logout').props('outline round').classes('mt-4')

    @ui.page('/changelog')
    def changelog_page():
        with theme.frame('Changelog', share_dir):
            changelog.content(share_dir)

    # @ui.page('/tros2')
    # def tors2_page() -> None:
    #     def footer():
    #         with ui.expansion('Trace').bind_value(app.storage.user, 'trace_expansion').classes('w-full').style('margin: 0px; padding: 0px;'):
    #             with ui.scroll_area().style('max-height: 180px;'):
    #                 ui.code(content="", language='plaintext').bind_content_from(app.storage.user, "trace").classes('w-full')
            
    #     with theme.frame('TROS Teil 2', footer):
    #         page_TROS_Teil2.content()

    @ui.page('/b')
    def example_page_b():
        with theme.frame('- Example B -', share_dir):
            message('Example B')
            
    @ui.page('/spark-overview')
    def spark_overview_page():
        with theme.frame('Spark Overview', share_dir, footer):
            spark_overview.content(share_dir)
            
    # Create a JSON editor and apply the custom styles
    @ui.page('/json-editor')
    def json_editor():
        ui.add_css('''
            .red {
                color: red;
            }
        ''')
        ui.add_css("""
            .my-json-editor {
                /* define a custom theme color */
                --jse-theme-color: #383e42;
                --jse-theme-color-highlight: #687177;
            }
        """)
        ui.label('This is red with CSS.').classes('red')
        with ui.column():
            ui.label('Custom JSON Editor')
            ui.json_editor({'content': {'json': {'example': 'value'}}}).classes('my-json-editor')  # Apply custom class

