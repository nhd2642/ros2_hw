# 두 언어 빌드:     make html       -> _build/html/{ko,en}/index.html (_build/html/index.html 을 열면 됨)
# 고치면서 보기:    make live       (한국어, 저장할 때마다 다시 빌드; 영어는 DOCLANG=en)
# 결과 그래프 다시:  make figures    (다른 results/ 폴더를 쓰려면 RUNS="<실행> <실행>")
# 로컬 전용: 여기 있는 어떤 대상도 사이트를 외부에 올리지 않는다.
SPHINXBUILD ?= python3 -m sphinx
OUT = _build/html
DOCLANG ?= ko
RUNS ?=

.PHONY: html ko en figures serve live clean
html: ko en
	cp index.html $(OUT)/index.html
ko:
	$(SPHINXBUILD) -b html -q ko $(OUT)/ko
en:
	$(SPHINXBUILD) -b html -q en $(OUT)/en
figures:
	python3 scripts/make_figures.py $(RUNS)
serve: html
	python3 -m http.server -d $(OUT) --bind 127.0.0.1 8000
live:
	python3 -m sphinx_autobuild $(DOCLANG) _build/live/$(DOCLANG) --host 127.0.0.1 --port 8000 \
	  --watch shared --watch diagrams --watch images
clean:
	rm -rf _build
