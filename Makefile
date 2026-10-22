PYTHON := $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)
.PHONY: install run test smoke docker-build docker-run

install:
	$(PYTHON) -m pip install -r requirements.txt
	npm install

run:
	$(PYTHON) -m uvicorn backend.main:app --host 0.0.0.0 --port 8000

test:
	$(PYTHON) -m pytest -q

smoke:
	npx playwright install --with-deps chromium
	$(PYTHON) -m uvicorn backend.main:app --host 0.0.0.0 --port 8000 & echo $$! > .uvicorn.pid
	@for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 20 25 30; do \
		curl -sf http://localhost:8000/ >/dev/null && break; \
		sleep 1; \
	done
	npx playwright test tests/e2e/smoke.spec.ts
	-kill `cat .uvicorn.pid` 2>/dev/null || true
	@rm -f .uvicorn.pid

docker-build:
	docker build -t arm-teleop .

docker-run:
	docker run --rm -p 8000:8000 arm-teleop
