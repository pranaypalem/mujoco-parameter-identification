#!/bin/bash

# Setup script for local development environment
# This script creates a virtual environment and tests the CI pipeline locally

set -e  # Exit on any error

echo "🚀 Setting up MuJoCo Parameter Identification local environment"
echo "================================================================="

# Check if python3 is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 is not installed. Please install Python 3.8 or higher."
    exit 1
fi

# Create virtual environment
echo "📦 Creating virtual environment..."
if [ -d "venv" ]; then
    echo "⚠️  Virtual environment already exists. Removing..."
    rm -rf venv
fi

python3 -m venv venv
source venv/bin/activate

echo "✅ Virtual environment created and activated"

# Upgrade pip
echo "⬆️  Upgrading pip..."
python -m pip install --upgrade pip

# Install dependencies
echo "📦 Installing dependencies..."
pip install -r requirements-dev.txt

echo "✅ Dependencies installed"

# Run local CI pipeline tests
echo "🔍 Running local CI pipeline tests..."
echo "======================================"

echo "1️⃣  Code formatting check..."
if black --check src/; then
    echo "✅ Code formatting is correct"
else
    echo "❌ Code formatting issues found. Running formatter..."
    black src/
    echo "✅ Code formatted"
fi

echo "2️⃣  Import sorting check..."
if isort --check-only src/; then
    echo "✅ Import sorting is correct"
else
    echo "❌ Import sorting issues found. Running sorter..."
    isort src/
    echo "✅ Imports sorted"
fi

echo "3️⃣  Linting with flake8..."
if make lint; then
    echo "✅ Linting passed"
else
    echo "❌ Linting failed. Please fix the issues above."
    exit 1
fi

echo "4️⃣  Type checking with mypy..."
if make typecheck; then
    echo "✅ Type checking passed"
else
    echo "⚠️  Type checking issues found. Please review."
fi

echo "5️⃣  Running tests..."
if make test; then
    echo "✅ All tests passed"
else
    echo "❌ Some tests failed. Please fix the issues above."
    exit 1
fi

echo "6️⃣  Running tests with coverage..."
if make test-coverage; then
    echo "✅ Tests with coverage completed"
else
    echo "⚠️  Coverage test issues. Please review."
fi

echo "✅ Local CI pipeline completed successfully!"
echo ""
echo "🎉 Environment setup complete!"
echo "==================================="
echo ""
echo "To activate the virtual environment in the future:"
echo "  source venv/bin/activate"
echo ""
echo "To deactivate:"
echo "  deactivate"
echo ""
echo "Available make commands:"
echo "  make help          - Show all available commands"
echo "  make install       - Install dependencies"
echo "  make format        - Format code"
echo "  make lint          - Run linting"
echo "  make test          - Run tests"
echo "  make generate-gif  - Generate simulation GIF"
echo "  make clean         - Clean temporary files"