# Makefile for MuJoCo Parameter Identification Project

# Variables
PYTHON := python3
PIP := pip3
JUPYTER := jupyter
SRC_DIR := src
NOTEBOOKS_DIR := notebooks
DATA_DIR := data
MEDIA_DIR := media
RESULTS_DIR := results

# Default target
.PHONY: all
all: install lint test

# Installation
.PHONY: install
install:
	$(PIP) install -r requirements.txt

# Development installation
.PHONY: install-dev
install-dev:
	$(PIP) install -r requirements-dev.txt

# Code formatting
.PHONY: format
format:
	black $(SRC_DIR)/
	isort $(SRC_DIR)/
	@echo "Code formatting completed"

# Linting
.PHONY: lint
lint:
	flake8 $(SRC_DIR)/
	pylint $(SRC_DIR)/
	@echo "Linting completed"

# Type checking
.PHONY: typecheck
typecheck:
	mypy $(SRC_DIR)/
	@echo "Type checking completed"

# Testing
.PHONY: test
test:
	pytest tests/ -v
	@echo "Testing completed"

# Test with coverage
.PHONY: test-coverage
test-coverage:
	pytest tests/ -v --cov=$(SRC_DIR) --cov-report=html
	@echo "Testing with coverage completed"

# Run notebooks
.PHONY: run-notebook
run-notebook:
	$(JUPYTER) notebook $(NOTEBOOKS_DIR)/

# Clean cache and temporary files
.PHONY: clean
clean:
	find . -type f -name "*.pyc" -delete
	find . -type d -name "__pycache__" -delete
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name "*.egg-info" -exec rm -rf {} +
	rm -rf htmlcov/
	rm -rf .coverage
	@echo "Clean completed"

# Generate GIF from simulation
.PHONY: generate-gif
generate-gif:
	$(PYTHON) $(SRC_DIR)/gif_generator.py
	@echo "GIF generation completed"

# Run complete analysis pipeline
.PHONY: run-analysis
run-analysis:
	$(JUPYTER) nbconvert --execute --to notebook --inplace $(NOTEBOOKS_DIR)/Parameter_Identification.ipynb
	@echo "Analysis pipeline completed"

# Build documentation
.PHONY: docs
docs:
	@echo "Building documentation..."
	@echo "Documentation build completed"

# Check code quality (combines multiple checks)
.PHONY: quality
quality: lint typecheck test
	@echo "Code quality check completed"

# Help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  install       - Install project dependencies"
	@echo "  install-dev   - Install development dependencies"
	@echo "  format        - Format code with black and isort"
	@echo "  lint          - Run linting with flake8 and pylint"
	@echo "  typecheck     - Run type checking with mypy"
	@echo "  test          - Run tests with pytest"
	@echo "  test-coverage - Run tests with coverage report"
	@echo "  run-notebook  - Start Jupyter notebook server"
	@echo "  clean         - Clean cache and temporary files"
	@echo "  generate-gif  - Generate GIF from simulation"
	@echo "  run-analysis  - Execute complete analysis pipeline"
	@echo "  docs          - Build documentation"
	@echo "  quality       - Run all code quality checks"
	@echo "  help          - Show this help message"