#!/bin/bash

# Pulls from sdm-vcu-firm@main
cd vcu-firm && git checkout main && git pull && cd ..;

# Update link to most recent commit
git add vcu-firm && git commit -m "Repo link bump";
git pull && git push

