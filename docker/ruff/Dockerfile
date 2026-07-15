ARG RUFF_VERSION=0.15.0

FROM ghcr.io/astral-sh/ruff:${RUFF_VERSION}-alpine AS ruff

FROM alpine:3.22

COPY --from=ruff /usr/local/bin/ruff /usr/local/bin/ruff

ENTRYPOINT ["/usr/local/bin/ruff"]
