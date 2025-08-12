run:
	python3 cli.py

lint:
	python3 -m pyflakes myterm || true

compile:
	python3 -m py_compile $(shell find myterm -name "*.py") cli.py
