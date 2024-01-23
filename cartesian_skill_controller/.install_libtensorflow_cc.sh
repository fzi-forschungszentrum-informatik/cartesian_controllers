#/usr/bin/bash
cd $HOME
sudo apt-get install wget
TF_VERSION=2.13.0
ARCH=$(dpkg --print-architecture)
wget https://github.com/ika-rwth-aachen/libtensorflow_cc/releases/download/v${TF_VERSION}/libtensorflow-cc_${TF_VERSION}_${ARCH}.deb
sudo dpkg -i libtensorflow-cc_${TF_VERSION}_${ARCH}.deb
sudo ldconfig
