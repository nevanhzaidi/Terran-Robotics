#!/bin/bash
set -e

echo "Setting up Python virtual environment for testing..."
python3 -m venv venv
source venv/bin/activate

echo "Installing dependencies..."
pip install --upgrade pip
pip install numpy opencv-python scipy matplotlib pandas jupyter notebook pytest

echo "Running tests..."
python -m pytest tests/ -v

echo "Tests completed!"

