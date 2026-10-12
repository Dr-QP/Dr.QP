#!/usr/bin/env bash
set -euo pipefail

# Point the user's OpenCode config at a catalog root's bridge plugin (default: this
# checkout), as reinstall-agentdev-codex.sh does for Codex. Each run leaves exactly
# one bridge entry, pointing at the last root.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

catalog_root="${1:-$script_dir/../..}"

bridge_suffix="/.agents/plugins/agentdev/.opencode-plugin"

# Only a catalog that ships the bridge has anything to register. Anywhere else this
# is a no-op, leaving whatever was registered from the image in place.
if [[ ! -d "$catalog_root$bridge_suffix" ]]; then
  echo "$catalog_root ships no OpenCode bridge plugin; nothing to register."
  exit 0
fi

bridge="$(cd "$catalog_root$bridge_suffix" && pwd)"
config_dir="${OPENCODE_CONFIG_DIR:-${XDG_CONFIG_HOME:-$HOME/.config}/opencode}"
config="$config_dir/opencode.json"

mkdir -p "$config_dir"
[[ -s "$config" ]] || echo '{}' >"$config"

updated="$(mktemp "$config_dir/opencode.json.XXXXXX")"
trap 'rm -f "$updated"' EXIT

# A plugin entry is a spec string or a [spec, options] pair.
jq --arg bridge "$bridge" --arg suffix "$bridge_suffix" '
  .plugin = [
    (.plugin // [])[]
    | select((if type == "array" then .[0] else . end) | tostring | endswith($suffix) | not)
  ] + [$bridge]
' "$config" >"$updated"
mv "$updated" "$config"

echo "Registered the OpenCode bridge $bridge in $config."
