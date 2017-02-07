#!/usr/bin/env bash
output="$($@)"
if [[ "$output" ]]; then
  echo "$output"
  exit 1
fi
