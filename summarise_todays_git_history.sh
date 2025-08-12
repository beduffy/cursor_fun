#!/bin/bash
echo "Today's Git Summary:"
git log --since="midnight" --author="$(git config user.name)" --oneline | tee /tmp/git_summary.txt
echo -e "\nSummary by AI:"
cat /tmp/git_summary.txt | openai_api_call  # Replace with actual API call
# TODO finish
