SHELL := /bin/bash

BUILD_DIR  := .
OUTPUT_DIR := /build

ARCH     ?= $(shell uname -m)
TAG      := $(ARCH)_$(shell git rev-parse --short HEAD 2>/dev/null || echo NOHASH)
REPO     ?= $(shell git config --get remote.origin.url 2>/dev/null \
              | sed -e 's|.*github.com[:/]||' -e 's|\.git$$||' \
              | tr '[:upper:]' '[:lower:]')

DOCKER_IMAGE        := adore-model-checker-deb-builder:$(TAG)
REGISTRY_IMAGE      := ghcr.io/$(REPO)/adore-model-checker-deb-builder:$(TAG)
REGISTRY_IMAGE_LATEST := ghcr.io/$(REPO)/adore-model-checker-deb-builder:latest

.PHONY: build
build:
	mkdir -p $(BUILD_DIR)
	@echo "Attempting to pull $(REGISTRY_IMAGE) from registry..."
	docker pull "$(REGISTRY_IMAGE)" 2>/dev/null && \
	    docker tag  "$(REGISTRY_IMAGE)" "$(DOCKER_IMAGE)" || \
	docker pull "$(REGISTRY_IMAGE_LATEST)" 2>/dev/null && \
	    docker tag  "$(REGISTRY_IMAGE_LATEST)" "$(DOCKER_IMAGE)" || \
	    echo "No cached image found, building from scratch..."
	docker build \
	    --cache-from "$(DOCKER_IMAGE)" \
	    -t "$(DOCKER_IMAGE)" \
	    .
	docker run --rm -v "$(BUILD_DIR)":"$(OUTPUT_DIR)" "$(DOCKER_IMAGE)"
	docker cp $$(docker create --rm "$(DOCKER_IMAGE)"):"$(OUTPUT_DIR)"/ "$(BUILD_DIR)"

.PHONY: push
push: ## Push image to ghcr.io
	docker tag  "$(DOCKER_IMAGE)" "$(REGISTRY_IMAGE)"
	docker push "$(REGISTRY_IMAGE)"
	docker tag  "$(DOCKER_IMAGE)" "$(REGISTRY_IMAGE_LATEST)"
	docker push "$(REGISTRY_IMAGE_LATEST)"

.PHONY: install
install: _install clean

.PHONY: _install
_install:
	python3 setup.py install

.PHONY: clean
clean:
	rm -rf ros2tools.egg-info build dist adore_model_checker/__pycache__
	rm -rf build
	docker rmi "$(DOCKER_IMAGE)" --force || true
