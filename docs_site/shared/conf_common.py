"""한국어(ko/)·영어(en/) 빌드가 같이 쓰는 설정. 각 conf.py 가 SHARED(이 폴더)를 정한 뒤 이 파일을 exec() 한다."""
import os
import sys

HERE = SHARED   # ko/conf.py, en/conf.py 가 exec() 전에 정함
sys.path.insert(0, HERE)

project = 'Disaster Exploration Sim'
author = 'nhd2642'
copyright = '2026, nhd2642'
release = 'mission v2'

extensions = [
    'myst_parser',
    'sphinx.ext.graphviz',
    'sphinx.ext.todo',
    'sphinx_copybutton',
    'sphinx_design',
    'screenshot',
]
myst_enable_extensions = ['colon_fence', 'deflist', 'attrs_inline', 'attrs_block', 'tasklist']
myst_heading_anchors = 3
source_suffix = {'.md': 'markdown'}
exclude_patterns = ['_build']

templates_path = [os.path.join(HERE, '_templates')]
html_static_path = [os.path.join(HERE, '_static')]
html_css_files = ['css/custom.css']
html_js_files = ['js/lang_switch.js']
html_logo = os.path.join(HERE, '_static', 'logo.svg')
html_favicon = os.path.join(HERE, '_static', 'logo.svg')
html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'navigation_depth': 3,
    'collapse_navigation': False,
    'style_external_links': True,
    'logo_only': False,
    'prev_next_buttons_location': 'both',
}
html_show_sourcelink = False
pygments_style = 'monokai'

# 다이어그램: 빌드할 때 Graphviz(dot)가 SVG 로 그림. 두 언어가 같은 원본을 쓰도록 라벨은 영어.
graphviz_output_format = 'svg'
graphviz_dot_args = [
    '-Gbgcolor=transparent', '-Gfontname=Helvetica', '-Nfontname=Helvetica', '-Efontname=Helvetica',
    '-Gfontsize=12', '-Nfontsize=12', '-Efontsize=10', '-Gpad=0.2',
]
copybutton_prompt_text = r'\$ '
copybutton_prompt_is_regexp = True
