#!/usr/bin/env bash
set -euo pipefail

# Register a catalog root as a Codex marketplace and install the plugin it
# publishes.
#
# Called twice with different roots. postCreate passes the catalog staged in the
# image, so any workspace gets the catalog. postStart passes nothing, which
# defaults to this checkout and re-registers the workspace copy on top — that is
# how the catalog is developed in place. Codex has no installation scopes.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

catalog_root="${1:-$(cd "$script_dir/../.." && pwd)}"

marketplace_json="$catalog_root/.agents/plugins/marketplace.json"

# Only the catalog's own repository ships a marketplace. Anywhere else this is a
# no-op, leaving whatever was installed from the image in place.
if [[ ! -f "$marketplace_json" ]]; then
  echo "$catalog_root declares no Codex marketplace; nothing to reinstall."
  exit 0
fi

marketplace_name="$(jq -er '.name' "$marketplace_json")"
# The Codex manifest publishes one plugin by design: self-improve ships no Codex
# manifest, so the two ecosystems publish different sets. Indexing is accurate
# here; generality no caller exercises is not.
plugin_name="$(jq -er '.plugins[0].name' "$marketplace_json")"

# Remove the marketplace declared by this root plus any marketplace still
# registered under an older name but pointing at the same root. Otherwise a
# marketplace rename can leave a stale plugin installation and cache behind.
mapfile -t stale_names < <(
  codex plugin marketplace list --json \
    | jq -r --arg root "$catalog_root" '
        .marketplaces[]
        | select(.root == $root or .marketplaceSource.source == $root)
        | .name
      '
)

# The sweep removes blind, so on a clean machine there is nothing registered to
# remove yet. Hold each call's output and report it only when the call succeeded,
# or when it failed for some reason other than the thing not being there.
absent_pattern='not found|is not configured or installed'

try_remove() {
  local output status=0
  output="$("$@" 2>&1)" || status=$?
  if ((status == 0)); then
    [[ -z "$output" ]] || printf '%s\n' "$output"
  elif [[ ! "$output" =~ $absent_pattern ]]; then
    printf '%s\n' "${output:-"failed: $* (exit $status)"}" >&2
  fi
}

while read -r name; do
  [[ -n "$name" ]] || continue
  try_remove codex plugin remove "$plugin_name@$name"
  try_remove codex plugin marketplace remove "$name"
done < <(printf '%s\n' "$marketplace_name" "${stale_names[@]}" | sort -u)

codex plugin marketplace add "$catalog_root"
codex plugin add "$plugin_name@$marketplace_name"
