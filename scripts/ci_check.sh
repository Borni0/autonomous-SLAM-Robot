#!/usr/bin/env bash
# CI smoke check that runs on every commit. Exits non-zero on the
# first failure so GitHub Actions / GitLab CI can fail the build.
#
# Usage: ./scripts/ci_check.sh

set -euo pipefail

cd "$(dirname "$0")/.."

echo "==> Python syntax check"
find src -name '*.py' -print0 | xargs -0 -I{} python3 -m py_compile "{}"

echo "==> Pure-Python unit tests"
./scripts/run_all_tests.py

echo "==> YAML well-formedness (skipping large nav2_params.yaml if needed)"
for f in $(find src -name '*.yaml'); do
    python3 -c "import yaml,sys; yaml.safe_load(open('$f'))" || {
        echo "YAML parse failed for $f"
        exit 1
    }
done

echo "==> All checks passed."