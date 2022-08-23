MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/buildtools/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)

IMAGE_NAME?=nicholascao2021/starling-controller
NETWORK?=bridge
ENV?=
NODE_ID?=0
RUN_ARGS?=-v /dev/ttyACM$(NODE_ID):/dev/ttyACM$(NODE_ID) --privileged -e VEHICLE_MAVLINK_SYSID=$(NODE_ID)

all: build

help:
	@echo "all - run build"
	@echo "build - build the dockerfile for this project"
	@echo "run - builds and runs the dockerfile for this project"
	@echo "run_bash - builds and runs the dockerfile putting you into a bash shell"
	@echo "local-build-push - locally builds and pushes amd64 and arm64 variants of your container"
	@echo "help - show this help screen"

build:
	$(BAKE) starling_controller

local-build-setup:
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
	docker buildx create --name mybuilder
	docker buildx use mybuilder
	docker buildx inspect --bootstrap

local-build-push:
	docker buildx bake --builder mybuilder -f $(BAKE_SCRIPT) --push starling-controller

run: build
	docker run -it --rm --net=$(NETWORK) $(ENV) -e USE_SIMULATED_TIME=true $(RUN_ARGS) $(IMAGE_NAME)

runonly:
	docker run -it --rm --net=$(NETWORK) $(ENV) -e USE_SIMULATED_TIME=true $(RUN_ARGS) $(IMAGE_NAME)

run_bash: build
	docker run -it --rm --net=$(NETWORK) $(ENV) -e USE_SIMULATED_TIME=true $(RUN_ARGS) $(IMAGE_NAME) bash

.PHONY: all build local-build-setup local-build-push run run_bash