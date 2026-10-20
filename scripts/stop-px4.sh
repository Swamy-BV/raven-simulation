#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
set -euo pipefail
px4=$(realpath -- "${1:?Provide PX4 source checkout}")
"$px4/build/px4_sitl_default/bin/px4-shutdown" --instance 1
