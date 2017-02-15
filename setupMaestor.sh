MAESTOR_DIR=$(pwd)
sudo mkdir -p /opt/maestor/
sudo mkdir -p /opt/maestor/logs/
sudo ln -s $MAESTOR_DIR/config /opt/maestor/config
sudo ln -s $MAESTOR_DIR/models /opt/maestor/models
