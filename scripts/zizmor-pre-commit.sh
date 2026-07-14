#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)
install_root="$root_dir/.tmp/zizmor"
binary="$install_root/zizmor"

if [[ ! -x "$binary" ]]; then
  case "$(uname -s)-$(uname -m)" in
    Linux-aarch64|Linux-arm64)
      target='aarch64-unknown-linux-gnu'
      checksum='afa6ea4ad183582adaf1610399ed9e697322a2374d377ccdb149216840f5171d'
      ;;
    Linux-x86_64)
      target='x86_64-unknown-linux-gnu'
      checksum='f7633bbf155cddebd3fdfdcbfaa777a4e89e4da1bc719b08dfa9ae50e5daae46'
      ;;
    Darwin-arm64)
      target='aarch64-apple-darwin'
      checksum='24bee9d58ac48639a3b10ca3cf5f797d52937ab2d7d2daf73c62bbfc0651a6ed'
      ;;
    Darwin-x86_64)
      target='x86_64-apple-darwin'
      checksum='952175c0cfcd8d9ab3bbc86a4258813b97962bbd2585c95f04485f7e1a8ca044'
      ;;
    *)
      echo "Unsupported platform for zizmor: $(uname -s)-$(uname -m)" >&2
      exit 1
      ;;
  esac

  mkdir -p "$install_root"
  archive="$install_root/zizmor.tar.gz"
  url="https://github.com/zizmorcore/zizmor/releases/download/v1.22.0/zizmor-${target}.tar.gz"

  curl --fail --location --retry 3 --output "$archive" "$url"
  actual_checksum=$(sha256sum "$archive" | awk '{print $1}')
  if [[ "$actual_checksum" != "$checksum" ]]; then
    echo 'Downloaded zizmor archive failed checksum verification.' >&2
    rm -f "$archive"
    exit 1
  fi

  tar -xzf "$archive" -C "$install_root"
  rm -f "$archive"
fi

exec "$binary" "$@"
