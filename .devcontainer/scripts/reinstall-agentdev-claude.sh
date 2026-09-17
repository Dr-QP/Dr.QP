#!/usr/bin/env bash
set -euo pipefail

# Register a catalog root as a Claude Code marketplace and install every plugin
# it publishes.
#
# Called twice with different roots. postCreate passes the catalog staged in the
# image, at user scope, so any workspace gets the catalog. postStart passes
# nothing, which defaults to this checkout at local scope and re-registers the
# workspace copy on top — that is how the catalog is developed in place.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

catalog_root="${1:-$(cd "$script_dir/../.." && pwd)}"
scope="${2:-local}"

marketplace_json="$catalog_root/.claude-plugin/marketplace.json"

# Only the catalog's own repository ships a marketplace. Anywhere else this is a
# no-op, leaving whatever was installed from the image in place.
if [[ ! -f "$marketplace_json" ]]; then
  echo "$catalog_root declares no Claude marketplace; nothing to reinstall."
  exit 0
fi

marketplace_name="$(jq -er '.name' "$marketplace_json")"
mapfile -t plugin_names < <(jq -er '.plugins[].name' "$marketplace_json")

# `claude plugin marketplace remove` only accepts a marketplace name, never a
# path. Collect the name this root currently declares plus any registered under
# an older name but still pointing here, so a rename does not leave a stale entry
# shadowing the new one.
mapfile -t stale_names < <(
  claude plugin marketplace list --json \
    | jq -r --arg root "$catalog_root" '
        .[]
        | select(.path == $root or .installLocation == $root)
        | .name
      '
)

# The sweep below removes blind, so most of its calls are expected to fail: on a
# clean machine there is nothing registered to remove yet, and a plugin only ever
# lives in one of the three scopes each is tried in. Hold each call's output and
# report it only when the call succeeded, or when it failed for some reason other
# than the thing not being there to remove.
absent_pattern='not found|is installed in [a-z]+ scope'

try_remove() {
  local output status=0
  output="$("$@" 2>&1)" || status=$?
  if ((status == 0)); then
    [[ -z "$output" ]] || printf '%s\n' "$output"
  elif [[ ! "$output" =~ $absent_pattern ]]; then
    printf '%s\n' "${output:-"failed: $* (exit $status)"}" >&2
  fi
}

# Both removals sweep every scope on purpose: the image installs at user scope
# and this checkout at local scope, so a declaration left in either one shadows
# the other just as effectively, and `add` would report the stale name instead of
# re-reading marketplace.json. `marketplace remove` does this when --scope is
# omitted; `uninstall` needs each scope named.
while read -r name; do
  [[ -n "$name" ]] || continue
  for plugin_name in "${plugin_names[@]}"; do
    for uninstall_scope in user project local; do
      try_remove claude plugin uninstall "$plugin_name@$name" --scope "$uninstall_scope"
    done
  done
  try_remove claude plugin marketplace remove "$name"
done < <(printf '%s\n' "$marketplace_name" "${stale_names[@]}" | sort -u)

claude plugin marketplace add "$catalog_root" --scope "$scope"
for plugin_name in "${plugin_names[@]}"; do
  claude plugin install "$plugin_name@$marketplace_name" --scope "$scope"
done
