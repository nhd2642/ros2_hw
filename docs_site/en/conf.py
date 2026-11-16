import os
SHARED = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'shared'))
exec(open(os.path.join(SHARED, 'conf_common.py')).read())
language = 'en'
html_title = 'Disaster Exploration Sim Docs'
html_context = {'other_lang': 'ko', 'other_lang_label': '한국어'}
