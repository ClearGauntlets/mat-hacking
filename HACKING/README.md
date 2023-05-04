# Contributing to this project

First of all, thanks! Second of all, this is pretty much just a stub right now, but here's how you use the Dockerfile. Note that this is only for development.

## How to use this image

This image aims to be a fully integrated, easy-to-use environment for developing the software for the ESP32 platform we're using for this project. It includes the [esp-idf](https://github.com/espressif/esp-idf/) repository, and all packages required to use it.

If you need to install podman, check the [podman documentation](https://podman.io/getting-started/installation.html). **Podman >=3.2 is required.**

First, you need to get the container. You can either build the container with `podman build . --tag imaginerit-embedded-dev`, or get it from Docker hub with `podman pull willnilges/imaginerit-embedded-dev:v0.0.1`

Then, start the container. You might find it convenient to clone this repo outside of the container and mount it inside, so that you have persistent access. Do so using the '-v' flag. For example...

```
podman run -dit --name imagine-embedded-dev -v <cloned repo location>:/mnt/ImagineRIT2022:Z --annotation io.crun.keep_original_groups=1 --device=/dev/ttyUSB0:/dev/ttyUSB0:rwm imaginerit-embedded-dev
```

You may have to add `--security-opt label=disable` if you are unable to access your embedded device.

Then you can attach a shell to the pod

`
podman exec -it imagine-embedded-dev bash
`

You can also use the included `launch-environment.sh` shell script. **BE SURE TO RUN IT FROM THE BASE REPOSITORY.**

```
./HACKING/launch-environment.sh
```

Once you're inside, use these commands to run the demo:

```
cd /ImagineRIT2022
idf.py -p /dev/ttyUSB0 flash monitor
```

See [the esp-idf documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for more information.

## Enrolling/Unenrolling ESPs

To enroll an ESP, you'll need its MAC address and its latitude/longitude, as well as the backend server token. Ask Willard or Joe.

```
./enroll-sniffer.sh -m '88:88:88:88:88:88' -x '-69.0' -y '69.0' -p 'what are you, a cop?'
```

To unenroll an ESP, just add the `-u` flag and omit the latitude/longitude.

```
./enroll-sniffer.sh -m '88:88:88:88:88:88' -u -p 'what are you, a cop?'
```

These options also work.

```
Options:
    -u|--unenroll
    -m|--mac
    -y|--latitude|--lat
    -x|--longitude|--lon
    -p|--password|--pass
```
