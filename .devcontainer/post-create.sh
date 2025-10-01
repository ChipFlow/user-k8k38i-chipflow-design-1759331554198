#!/bin/bash

set -e

echo "🚀 Setting up ChipFlow development environment..."

# Install system dependencies required for ChipFlow and silicon toolchain
echo "📦 Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    unzip \
    vim \
    nano \
    htop \
    tree \
    jq \
    make \
    cmake \
    pkg-config \
    libffi-dev \
    libssl-dev \
    zlib1g-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    libncurses5-dev \
    libgdbm-dev \
    libnss3-dev \
    libssl-dev \
    libreadline-dev \
    libffi-dev \
    liblzma-dev

# Install PDM (Python Dependency Manager)
echo "🐍 Installing PDM..."
curl -sSL https://pdm-project.org/install-pdm.py | python3 -

# Add PDM to PATH
export PATH="/home/vscode/.local/bin:$PATH"
echo 'export PATH="/home/vscode/.local/bin:$PATH"' >> ~/.bashrc

# Verify PDM installation
pdm --version

# Install project dependencies
echo "📚 Installing Python dependencies..."
if [ -f "pyproject.toml" ]; then
    # Install all dependencies including dev dependencies
    pdm install --dev
    
    # Create activation script for easier environment access
    echo "📝 Creating environment activation script..."
    cat > activate_env.sh << 'EOF'
#!/bin/bash
# Activate the PDM virtual environment
eval $(pdm info --env)
echo "✅ ChipFlow environment activated!"
echo "📋 Available commands:"
echo "  pdm run chipflow    - Run ChipFlow CLI"
echo "  pdm run sim-run     - Run simulation"
echo "  pdm run sim-check   - Run simulation with verification"
echo "  pdm run test        - Run tests"
echo "  pdm run lint        - Run linting"
EOF
    chmod +x activate_env.sh
    
    echo "🎯 To activate the environment, run: source activate_env.sh"
else
    echo "⚠️  No pyproject.toml found, skipping dependency installation"
fi

# Install additional useful tools for silicon development
echo "🛠️  Installing additional development tools..."

# Install ripgrep for fast searching
curl -LO https://github.com/BurntSushi/ripgrep/releases/download/13.0.0/ripgrep_13.0.0_amd64.deb
sudo dpkg -i ripgrep_13.0.0_amd64.deb
rm ripgrep_13.0.0_amd64.deb

# Install fd for fast file finding
curl -LO https://github.com/sharkdp/fd/releases/download/v8.7.0/fd_8.7.0_amd64.deb
sudo dpkg -i fd_8.7.0_amd64.deb
rm fd_8.7.0_amd64.deb

# Set up git configuration helpers
echo "🔧 Setting up git configuration..."
git config --global init.defaultBranch main
git config --global pull.rebase false
git config --global core.editor nano

# Create a welcome message
echo "📋 Creating welcome message..."
cat > README_CODESPACE.md << 'EOF'
# 🚀 ChipFlow Codespace Development Environment

Welcome to your ChipFlow development environment! This Codespace is pre-configured with all the tools and dependencies you need for ChipFlow silicon development.

## 🏃‍♂️ Quick Start

1. **Activate the environment:**
   ```bash
   source activate_env.sh
   ```

2. **Run your first simulation:**
   ```bash
   pdm run sim-check
   ```

3. **Explore available commands:**
   ```bash
   pdm run --list
   ```

## 📚 Available Commands

- `pdm run chipflow` - ChipFlow CLI interface
- `pdm run sim-run` - Run simulation
- `pdm run sim-check` - Run simulation with verification  
- `pdm run test` - Run test suite
- `pdm run lint` - Code linting
- `pdm run submit` - Submit to ChipFlow silicon service

## 🔧 Development Tools

- **Python**: PDM for dependency management
- **Editor**: VS Code with Python extensions
- **Git**: Pre-configured with sensible defaults
- **Search**: ripgrep (`rg`) and fd for fast file operations
- **Debugging**: Built-in Python debugger support

## 📖 Documentation

- [ChipFlow Documentation](https://docs.chipflow.io)
- [PDM Documentation](https://pdm.fming.dev)
- [Amaranth HDL Documentation](https://amaranth-lang.org/docs/amaranth/latest/)

Happy coding! 🎉
EOF

echo "✅ ChipFlow development environment setup complete!"
echo "📖 Check README_CODESPACE.md for usage instructions"
echo "🔄 To get started, run: source activate_env.sh"