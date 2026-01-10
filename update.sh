#!/usr/bin/env bash
#
# modem73 update script
# https://github.com/RFnexus/modem73
#

set -e

REPO_URL="https://github.com/RFnexus/modem73.git"
INSTALL_DIR="${MODEM73_DIR:-$HOME/modem73}"

if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    NC='\033[0m'
else
    RED=''
    GREEN=''
    YELLOW=''
    NC=''
fi

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

command -v git >/dev/null 2>&1 || error "git is required but not installed"
command -v make >/dev/null 2>&1 || error "make is required but not installed"


if command -v g++ >/dev/null 2>&1; then
    CXX="g++"
elif command -v clang++ >/dev/null 2>&1; then
    CXX="clang++"
else
    error "C++ compiler (g++ or clang++) is required but not installed"
fi

info "modem73 updater"
echo "  Repository: $REPO_URL"
echo "  Install to: $INSTALL_DIR"
echo ""

if [ -d "$INSTALL_DIR/.git" ]; then
    info "Updating = installation..."
    cd "$INSTALL_DIR"
    
    if ! git diff --quiet 2>/dev/null; then
        warn "Local changes detected. Stashing..."
        git stash
    fi
    
    BRANCH=$(git rev-parse --abbrev-ref HEAD)
    
    git fetch origin
    LOCAL=$(git rev-parse HEAD)
    REMOTE=$(git rev-parse "origin/$BRANCH")
    
    if [ "$LOCAL" = "$REMOTE" ]; then
        info "Already up to date"
        NEEDS_BUILD=0
    else
        info "Pulling updates..."
        git pull --ff-only origin "$BRANCH"
        NEEDS_BUILD=1
    fi
else
    info "Cloning repository..."
    mkdir -p "$(dirname "$INSTALL_DIR")"
    git clone "$REPO_URL" "$INSTALL_DIR"
    cd "$INSTALL_DIR"
    NEEDS_BUILD=1
fi

# Build
if [ "${NEEDS_BUILD:-1}" = "1" ] || [ "$1" = "--force" ]; then
    info "Building modem73..."



    make clean 2>/dev/null || true
    


    if make -j"$(nproc 2>/dev/null || echo 2)"; then
        info "Build sucessful!"
        



        echo ""
        echo "  Commit: $(git rev-parse --short HEAD)"
        echo "  Date:   $(git log -1 --format=%ci)"
        echo ""
        



        if [ -f "$INSTALL_DIR/modem73" ]; then
            info "Binary: $INSTALL_DIR/modem73"
        fi
    else
        error "Build failed"
    fi
else
    info "No build needed (use --force to rebuild)"
fi



info "Done!"
