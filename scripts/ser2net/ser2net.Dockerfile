FROM ubuntu:20.04

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ser2net && \
    rm -rf /var/lib/apt/lists/*

# Shell form is required: the startup sequence chains commands with `&&`,
# which JSON exec notation cannot express.
# hadolint ignore=DL3025
CMD echo -n "Starting " && \
    ser2net -v && \
    ls /dev/tty* && \
    cat /etc/ser2net.yml && \
    ser2net -d -c /etc/ser2net.yml
