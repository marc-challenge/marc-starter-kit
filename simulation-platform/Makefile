# Copyright (c) 2026, IoT Convergence & Open Sharing System (IoTCOSS)
# MARC 2026 - thin aliases (the canonical path is docker compose / marc.sh). `make -C docker <target>`.
.PHONY: setup platform dataset-gen demo down clean

setup:    ; @bash marc.sh setup
platform: ; @bash marc.sh platform
dataset-gen: ; @bash marc.sh dataset-gen
demo:     ; @bash marc.sh demo $(TEAM)
down:     ; @bash marc.sh down
clean:    ; @bash marc.sh clean
