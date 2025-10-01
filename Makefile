# ChipFlow Generated Project Makefile

.PHONY: simulate build clean help install

# Default target
all: simulate

# Install dependencies (PDM should already be available via devcontainer)
install:
	@echo "Installing dependencies..."
	pdm install

# Run simulation  
simulate: install
	@echo "Running ChipFlow simulation..."
	pdm run python -m design

# Build for synthesis
build: install
	@echo "Building ChipFlow design..."
	mkdir -p build
	pdm run python -c "from design.design import MySoC; from amaranth.back import verilog; soc = MySoC(); print(verilog.convert(soc, name='soc_top'))" > build/soc_top.v

# Clean build artifacts
clean:
	rm -rf build/ __pycache__/ design/__pycache__/

# Show help
help:
	@echo "ChipFlow Generated Project"
	@echo ""
	@echo "Available targets:"
	@echo "  simulate  - Run RTL simulation" 
	@echo "  build     - Build for synthesis"
	@echo "  clean     - Clean build artifacts"
	@echo "  help      - Show this help"

# Show generated file info
info:
	@echo "Design: $(shell cat design.json | jq -r '.activeConfigId')"
	@echo "Blocks: $(shell cat design.json | jq -r '.enabledBlocks | length')"
	@echo "Generated: $(shell cat design.json | jq -r '.timestamp')"
