# This file lets you format the code-base with a single command.
FILES := $(shell find . -name '*.lf')

.PHONY: help
help:
	@echo "Available commands:"
	@echo "  make format        - Format all C source files using clang-format"
	@echo "  make clean         - Clean up build and documentation"
	
.PHONY: format
format:
	lff $(FILES)

.PHONY: clean
clean:
	rm -rf *.lft *.csv *.log src-gen fed-gen include bin
	
# Set help as the default target
.DEFAULT_GOAL := help
