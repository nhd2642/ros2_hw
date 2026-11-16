import os
SHARED = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'shared'))
exec(open(os.path.join(SHARED, 'conf_common.py')).read())
language = 'ko'
project = '재난 탐색 시뮬레이션'
html_title = '재난 탐색 시뮬레이션 문서'
html_context = {'other_lang': 'en', 'other_lang_label': 'English'}
