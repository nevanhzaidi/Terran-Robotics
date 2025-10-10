#!/bin/bash
set -e

echo "Setting up Python virtual environment..."
python3 -m venv venv
source venv/bin/activate

echo "Installing dependencies..."
pip install --upgrade pip
pip install numpy opencv-python scipy matplotlib pandas jupyter notebook

echo "Running the solution..."
python src/main.py

echo "Solution completed!"

