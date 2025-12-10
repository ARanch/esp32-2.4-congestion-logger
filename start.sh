#!/bin/bash

NOBUILD=false
NAME="test"

while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--nobuild)
            NOBUILD=true
            shift
            ;;
        *)
            NAME="$1"
            shift
            ;;
    esac
done

if [ "$NOBUILD" = false ]; then
    cd firmware
    pio run --target upload
    cd ..
fi

source venv/bin/activate
python logger.py --port /dev/cu.usbserial-0001 --name "$NAME"
