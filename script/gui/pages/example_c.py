from gui import theme
from gui.message import message

from nicegui import APIRouter, ui

router = APIRouter(prefix='/TROS')


@router.page('/')
def example_page():
    with theme.frame('- TROS -'):
        message('TROS Teil 2')
        # for i in range(1, 4):
        #     ui.link(f'Item {i}', f'/c/items/{i}').classes('text-xl text-grey-8')
        ui.link('NOHD', '/TROS/nohd').classes('text-xl text-grey-8')


@router.page('/nohd', dark=True)
def item(id: str):
    with theme.frame('- NOHD -'):
        message('Item NOHD')
        ui.link('go back', router.prefix).classes('text-xl text-grey-8')
