# Build both languages:  make html   -> _build/html/{ko,en}/index.html (open _build/html/index.html)
# Live preview:          make live            (Korean, rebuilds on save; DOCLANG=en for English)
# Redraw result graphs:  make figures         (RUNS="<run> <run>" to use other results/ folders)
# Local only: nothing here publishes the site.
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
