#!/bin/bash

if [ -z "$SOFF_PREFIX" ]; then
  echo "Define the SOFF_PREFIX environment variable before running $0."
  exit 1
fi

if [ ! -d "$SOFF_PREFIX" ]; then
  echo "SOFF_PREFIX=$SOFF_PREFIX is not a valid directory.";
  exit 1
fi

set -e

echo "[SOFF] Copying files to $SOFF_PREFIX..."
if [ -d "$SOFF_PREFIX/bsp" ]; then
  echo "[SOFF] All files in the existing $SOFF_PREFIX/bsp directory will be erased."
  read -r -p "[SOFF] Are you sure? [y/N] " response
  case "$response" in
    [yY][eE][sS]|[yY])
      ;;
    *)
      echo "[SOFF] Aborted by the user."
      exit 1
      ;;
  esac
  rm -rf "$SOFF_PREFIX/bsp"
fi
cp -r ./bsp "$SOFF_PREFIX"
mkdir -p "$SOFF_PREFIX/bin"
cp bin/* "$SOFF_PREFIX/bin/"

echo "[SOFF] soff-hardware-opae installation complete."
