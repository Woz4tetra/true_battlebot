#!/bin/bash
BASE_DIR=$(realpath "$(dirname "${0}")")

"${BASE_DIR}"/../copy_project
while true; do
    find $(realpath "${BASE_DIR}"/../..) -not -xtype l -follow | entr "${BASE_DIR}"/../copy_project
done
