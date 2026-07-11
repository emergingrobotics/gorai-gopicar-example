ROBOT_CONFIG ?= robot.json
BINARY_NAME  ?= picarx
TARGET       ?= $(shell go env GOOS)/$(shell go env GOARCH)
TAGS         ?= rpicam
PREFIX       ?= $(HOME)/bin

.DEFAULT_GOAL := help
.PHONY: build build-arm64 run install test test-hw validate clean deploy fmt vet check help

build: ## Build standalone binary
	gorai build $(ROBOT_CONFIG) -o bin/$(BINARY_NAME) --target $(TARGET) --tags $(TAGS)

build-arm64: ## Cross-compile for Raspberry Pi 64-bit
	gorai build $(ROBOT_CONFIG) -o bin/$(BINARY_NAME) --target linux/arm64 --tags $(TAGS)

run: build ## Build then run the standalone binary (with local components)
	./bin/$(BINARY_NAME) run $(ROBOT_CONFIG)

install: build ## Build then copy the binary to ~/bin (override with PREFIX=)
	@mkdir -p $(PREFIX)
	install -m 0755 bin/$(BINARY_NAME) $(PREFIX)/$(BINARY_NAME)
	@echo "==> Installed $(PREFIX)/$(BINARY_NAME)"

test: ## Fast hardware-free tests
	go test ./...

test-hw: ## On-Pi hardware integration tests
	go test -tags hardware ./...

validate: ## Validate robot configuration
	gorai validate $(ROBOT_CONFIG)

fmt: ; go fmt ./...
vet: ; go vet ./...
check: fmt vet test ## All checks

clean: ; rm -rf bin/

deploy: build-arm64 ## Build arm64 + scp to the Pi (set DEPLOY_HOST)
	@if [ -z "$(DEPLOY_HOST)" ]; then echo "Set DEPLOY_HOST (e.g. make deploy DEPLOY_HOST=pi@raspberrypi)"; exit 1; fi
	scp bin/$(BINARY_NAME) robot.json calibration.json $(DEPLOY_HOST):~/

help: ; @grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}'
