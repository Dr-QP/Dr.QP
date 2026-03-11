#!/bin/bash
# Firewall initialization script for the Dr.QP devcontainer.
# Restricts outbound internet access to the domains listed in the allowlist file.
# The allowlist file lives in the repo so it is branch-configurable without
# rebuilding the Docker image.
#
# Adapted from the Claude Code reference implementation:
# https://github.com/anthropics/claude-code/blob/main/.devcontainer/init-firewall.sh
set -euo pipefail
IFS=$'\n\t'

# Allowlist file is read from the mounted workspace so per-branch edits take
# effect on the next container start without an image rebuild.
ALLOWLIST="${FIREWALL_ALLOWLIST:-/opt/ros/overlay_ws/.devcontainer/firewall-allowlist.txt}"

if [ ! -f "$ALLOWLIST" ]; then
    echo "ERROR: Firewall allowlist not found: $ALLOWLIST"
    exit 1
fi

echo "Loading allowlist from $ALLOWLIST"

# 1. Extract Docker DNS info BEFORE any flushing
DOCKER_DNS_RULES=$(iptables-save -t nat | grep "127\.0\.0\.11" || true)

# Flush existing rules and delete existing ipsets
iptables -F
iptables -X
iptables -t nat -F
iptables -t nat -X
iptables -t mangle -F
iptables -t mangle -X
ipset destroy allowed-domains 2>/dev/null || true

# 2. Selectively restore ONLY internal Docker DNS resolution
if [ -n "$DOCKER_DNS_RULES" ]; then
    echo "Restoring Docker DNS rules..."
    iptables -t nat -N DOCKER_OUTPUT 2>/dev/null || true
    iptables -t nat -N DOCKER_POSTROUTING 2>/dev/null || true
    echo "$DOCKER_DNS_RULES" | xargs -L 1 iptables -t nat
else
    echo "No Docker DNS rules to restore"
fi

# Allow DNS and localhost before any restrictions
iptables -A OUTPUT -p udp --dport 53 -j ACCEPT
iptables -A INPUT  -p udp --sport 53 -j ACCEPT
iptables -A OUTPUT -p tcp --dport 22 -j ACCEPT
iptables -A INPUT  -p tcp --sport 22 -m state --state ESTABLISHED -j ACCEPT
iptables -A INPUT  -i lo -j ACCEPT
iptables -A OUTPUT -o lo -j ACCEPT

# Create ipset with CIDR support
ipset create allowed-domains hash:net

# Process the allowlist file
while IFS= read -r line; do
    # Strip comments and blank lines
    line="${line%%#*}"
    line="${line//[[:space:]]/}"
    [ -z "$line" ] && continue

    if [ "$line" = "github-meta" ]; then
        echo "Fetching GitHub IP ranges..."
        gh_ranges=$(curl -s https://api.github.com/meta)
        if [ -z "$gh_ranges" ]; then
            echo "ERROR: Failed to fetch GitHub IP ranges"
            exit 1
        fi
        if ! echo "$gh_ranges" | jq -e '.web and .api and .git' >/dev/null; then
            echo "ERROR: GitHub API response missing required fields"
            exit 1
        fi
        while read -r cidr; do
            if [[ ! "$cidr" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}/[0-9]{1,2}$ ]]; then
                echo "ERROR: Invalid CIDR from GitHub meta: $cidr"
                exit 1
            fi
            echo "Adding GitHub range $cidr"
            ipset add allowed-domains "$cidr"
        done < <(echo "$gh_ranges" | jq -r '(.web + .api + .git)[]' | aggregate -q)
    else
        echo "Resolving $line..."
        ips=$(dig +noall +answer A "$line" | awk '$4 == "A" {print $5}')
        if [ -z "$ips" ]; then
            echo "WARNING: Failed to resolve $line (skipping)"
            continue
        fi
        while read -r ip; do
            if [[ ! "$ip" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
                echo "ERROR: Invalid IP from DNS for $line: $ip"
                exit 1
            fi
            echo "Adding $ip for $line"
            ipset add allowed-domains "$ip" 2>/dev/null || true
        done < <(echo "$ips")
    fi
done < "$ALLOWLIST"

# Allow the host network (default route)
HOST_IP=$(ip route | grep default | cut -d" " -f3)
if [ -z "$HOST_IP" ]; then
    echo "ERROR: Failed to detect host IP"
    exit 1
fi
HOST_NETWORK=$(echo "$HOST_IP" | sed "s/\.[0-9]*$/.0\/24/")
echo "Host network: $HOST_NETWORK"
iptables -A INPUT  -s "$HOST_NETWORK" -j ACCEPT
iptables -A OUTPUT -d "$HOST_NETWORK" -j ACCEPT

# Default DROP policy
iptables -P INPUT   DROP
iptables -P FORWARD DROP
iptables -P OUTPUT  DROP

# Allow established connections
iptables -A INPUT  -m state --state ESTABLISHED,RELATED -j ACCEPT
iptables -A OUTPUT -m state --state ESTABLISHED,RELATED -j ACCEPT

# Allow outbound to the allowlist
iptables -A OUTPUT -m set --match-set allowed-domains dst -j ACCEPT

# Reject everything else immediately
iptables -A OUTPUT -j REJECT --reject-with icmp-admin-prohibited

echo "Firewall configuration complete"
echo "Verifying firewall rules..."
if curl --connect-timeout 5 https://example.com >/dev/null 2>&1; then
    echo "ERROR: Firewall verification failed - was able to reach https://example.com"
    exit 1
else
    echo "Firewall verification passed - example.com is blocked as expected"
fi
if ! curl --connect-timeout 5 https://api.github.com/zen >/dev/null 2>&1; then
    echo "ERROR: Firewall verification failed - unable to reach https://api.github.com"
    exit 1
else
    echo "Firewall verification passed - api.github.com is reachable as expected"
fi
