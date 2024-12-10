#!/bin/bash

# Define versions
INSTALL_NODE_VER=22
INSTALL_NVM_VER=0.40.1

echo "==> Ensuring .bashrc exists and is writable"
touch ~/.bashrc

echo "==> Installing node version manager (NVM). Version $INSTALL_NVM_VER"
# Removed if already installed
rm -rf ~/.nvm
# Unset exported variable
export NVM_DIR=

# Install nvm 
curl -o- https://raw.githubusercontent.com/creationix/nvm/v$INSTALL_NVM_VER/install.sh | bash
# Make nvm command available to terminal
source ~/.nvm/nvm.sh

echo "==> Installing node js version $INSTALL_NODE_VER"
nvm install $INSTALL_NODE_VER

echo "==> Make this version system default"
nvm alias default $INSTALL_NODE_VER
nvm use default

echo "==> Checking for versions"
nvm --version
node --version
npm --version

echo "==> List installed node versions"
nvm ls

nvm cache clear
echo "==> Now you're all setup and ready for development. If changes are yet totake effect, I suggest you restart your computer"
