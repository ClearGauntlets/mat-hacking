#!/bin/bash

function help () {
    echo "Pass -d to not use a device."
}

use_device="--device=/dev/ttyUSB0:rwm"

while getopts ":hd" option; do
    case $option in
        d) # Dry Run. Use a device?
            echo "Got -d. Not hooking up /dev/ttyUSB0."
            unset use_device
            ;;
        h)
            help
            exit;;
    esac
done

IMAGINE_DIR=$(pwd)
#mkdir -p "$IMAGINE_DIR"
podman run --rm -it                             \
    --name=esp-idf-dev                          \
    -v "$IMAGINE_DIR":/workspace                \
    --group-add keep-groups                     \
    --annotation io.crun.keep_original_groups=1 \
    --annotation run.oci.keep_original_groups=1 \
    --security-opt label=disable \
    $use_device \
    esp-idf-dev

# To open a shell, run: podman exec -it esp-idf-dev bash
# To add device, insert a device mapping:
#    --device=/dev/ttyUSB0:rwm                   \
