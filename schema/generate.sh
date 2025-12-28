#!/usr/bin/env sh

[ -z "$(command -v flatc)" ] && echo "flatc must be installed"

[ "flatc version 25.12.19" != "$(flatc --version)" ] && echo "flatc must have version 25.12.19"

cd "$(dirname $0)"

flatc --cpp -o ../include/ ./prefixtable.fbs
