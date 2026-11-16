"""``screenshot`` directive: shows docs_site/images/screenshots/<file> if it exists, otherwise a dashed placeholder.

    ```{screenshot} world_overview.png
    Caption text (also printed in the placeholder as "what to capture").
    ```
Drop the PNG into images/screenshots/ and rebuild (make clean html) to swap the placeholder for the picture.
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
