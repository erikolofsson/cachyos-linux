#!/bin/bash

set -e

rm -f *.tar.zst 

PACMAN_PKGBASE=linux-cachyos-imac5k make -j36 pacman-pkg
sudo pacman -U linux-cachyos-imac5k-7.*_imac5k_*-x86_64.pkg.tar.zst linux-cachyos-imac5k-headers-*_imac5k_*-x86_64.pkg.tar.zst
