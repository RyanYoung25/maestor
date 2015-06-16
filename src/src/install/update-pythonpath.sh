#!/bin/bash
if [[ `echo "$0" | grep "/" | wc -l` > 0 ]]; then
    cd ${0%/*}
fi
cd ../scripts/
PATH_TO_PACKAGE="$(pwd)"
NEW_PYTHON_LINE="export PYTHONPATH=\$PYTHONPATH:"$PATH_TO_PACKAGE
echo $NEW_PYTHON_LINE >> $HOME/.bashrc 
