#!/usr/bin/env bash
# -*- coding: utf-8 -*-

# Miniconda3 doesn't work on ARM64 as of now, see https://github.com/conda/conda/issues/11141
# wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O ~/miniconda.sh
# bash ~/miniconda.sh -b -p $HOME/miniconda

wget https://github.com/conda-forge/miniforge/releases/download/4.12.0-3/Mambaforge-4.12.0-3-Linux-aarch64.sh -O ~/mambaforge.sh
bash ~/mambaforge.sh -b -p $HOME/mambaforge
rm -f ~/mambaforge.sh
