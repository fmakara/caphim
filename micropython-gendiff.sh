#!/bin/bash
if [ -z "$1" ]; then
    echo "this needs a name!"
    exit 1
fi
echo "generating for $1..."
cd micropython
git diff > ../micropython-mods/$1-mods.diff
(git ls-files --others --exclude-standard -z | xargs -0 -n 1 git --no-pager diff /dev/null ) > ../micropython-mods/$1-adds.diff
