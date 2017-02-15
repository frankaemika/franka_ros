#!/usr/bin/env bash
# grep the result of clang-format with -output-replacements-xml flag
# if any "replacement" tags are found, then code is incorrectly formatted
echo $($@) | grep '<replacement ' > /dev/null
if [ $? -ne 1 ]; then 
    echo "Files have to be formatted with clang-format"
    exit 1;
fi
