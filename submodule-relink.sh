#!/bin/bash

# Init
set -e
cd "$(dirname "$0")"

# Pulls from sdm-vcu-firm@main
git -C vcu-firm checkout main
git -C vcu-firm pull


# Update link
git add vcu-firm && git diff --cached --quiet || git commit -m "vcu-firm@main link bump" && git push

