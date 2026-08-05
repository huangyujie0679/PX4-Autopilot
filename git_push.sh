#!/bin/bash

echo "========== Git Status =========="
git status

echo ""
read -p "Commit message: " msg

git add msg src/modules/guidance src/modules/target_manager

git commit -m "$msg"

git push
