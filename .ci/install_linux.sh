#!/usr/bin/env bash

set -e

cp -r "${AIKIDO_BUILD_DIR}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}"
