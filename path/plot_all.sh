#!/bin/bash
for file in *.csv; do
    python3 plot_trajectory.py $file
done
