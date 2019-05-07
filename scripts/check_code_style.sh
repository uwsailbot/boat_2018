#!/bin/bash

echo "Checking code style with yapf..."
yapf --diff --recursive -e .ci_config . > /dev/null

if [ $? -ne 0 ]; then
	echo >&2 "Code does not meet style requirements! Please run yapf to lint and format the code."
	exit 1
fi

echo "Code style meets requirements!"
exit 0
