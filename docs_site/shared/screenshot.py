"""``screenshot`` 지시어: docs_site/images/screenshots/<파일> 이 있으면 그 그림을, 없으면 점선 '스크린샷 자리' 상자를 보여 준다.

    ```{screenshot} world_overview.png
    캡션 (그림이 없을 때는 상자에 '여기에 넣을 화면' 으로 찍힘).
    ```
PNG 를 images/screenshots/ 에 넣고 다시 빌드(make clean html)하면 상자가 그림으로 바뀐다.
"""
import os

from docutils import nodes
from sphinx.util.docutils import SphinxDirective

SHOTS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'images', 'screenshots'))
LABEL = {'ko': ('스크린샷 자리', '여기에 넣을 화면', '파일'), 'en': ('Screenshot needed', 'What to capture', 'File')}


class Screenshot(SphinxDirective):
    required_arguments = 1
    has_content = True

    def run(self):
        name = self.arguments[0]
        path = os.path.join(SHOTS, name)
        caption = ' '.join(self.content).strip()
        if os.path.exists(path):
            self.env.note_dependency(path)
            docdir = os.path.dirname(self.env.doc2path(self.env.docname))
            fig = nodes.figure(align='center')
            fig += nodes.image(uri=os.path.relpath(path, docdir), alt=caption or name)
            if caption:
                fig += nodes.caption(text=caption)
            return [fig]
        title, what, file_ = LABEL.get(self.config.language, LABEL['en'])
        box = nodes.admonition(classes=['screenshot'])
        box += nodes.title(text=title)
        if caption:
            box += nodes.paragraph(text=f'{what}: {caption}')
        para = nodes.paragraph(text=f'{file_}: ')
        para += nodes.literal(text=f'docs_site/images/screenshots/{name}')
        box += para
        return [box]


def setup(app):
    app.add_directive('screenshot', Screenshot)
    return {'parallel_read_safe': True}
