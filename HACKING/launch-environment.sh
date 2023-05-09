#!/bin/bash
IMAGINE_DIR=$(pwd)
#mkdir -p "$IMAGINE_DIR"
podman run --rm -it                             \
    --name=esp-idf-dev                          \
    -v "$IMAGINE_DIR":/workspace                \
    --group-add keep-groups                     \
    --annotation io.crun.keep_original_groups=1 \
    --annotation run.oci.keep_original_groups=1 \
    --security-opt label=disable \
    esp-idf-dev

# To open a shell, run: podman exec -it esp-idf-dev bash
# To add device, insert a device mapping:
#    --device=/dev/ttyUSB0:rwm                   \
