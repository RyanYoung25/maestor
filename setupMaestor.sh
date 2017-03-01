MAESTOR_DIR=$(pwd)
sudo mkdir -p /opt/maestor/
sudo mkdir -p /opt/maestor/logs/

mkdir -p ./bin

ln -s $MAESTOR_DIR/../../devel/lib/maestor/maestor_node ./bin/maestor_node

sudo ln -s $MAESTOR_DIR/config /opt/maestor/config
sudo ln -s $MAESTOR_DIR/models /opt/maestor/models
sudo ln -s $MAESTOR_DIR/console /opt/maestor/console
sudo ln -s $MAESTOR_DIR/run /opt/maestor/run
sudo ln -s $MAESTOR_DIR/bin /opt/maestor/bin

