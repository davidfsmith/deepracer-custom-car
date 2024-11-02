#!/usr/bin/env bash

# This script builds and packages AWS DeepRacer components for ARM64 architecture.
# It supports the following packages:
# - aws-deepracer-util
# - aws-deepracer-device-console
# - aws-deepracer-core
# - aws-deepracer-sample-models

# Usage:
# ./build-deepracer-packages.sh [-p "package1 package2 ..."]

# Options:
# -p: Specify the packages to build. If not provided, the default packages will be built.

# The script performs the following steps:
# 1. Sets up the environment and directories.
# 2. Copies necessary DeepRacer repository files.
# 3. Clones the mxcam repository if not already present.
# 4. Checks for missing packages and downloads them if necessary.
# 5. Builds the specified packages for ARM64 architecture.

# Each package build involves:
# - Extracting the original AMD64 package.
# - Modifying the package contents and control files for ARM64.
# - Repacking the modified package.
# - Moving the final package to the distribution directory.

# Note:
# - Ensure that the required files and directories (e.g., versions.json, files directory) are present.
# - The script requires sudo privileges for certain operations.
set -e

export DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

PACKAGES="aws-deepracer-util aws-deepracer-device-console aws-deepracer-core aws-deepracer-sample-models"

while getopts "p:" opt; do
  case $opt in
  p)
    PACKAGES=$OPTARG
    ;;
  \?)
    echo "Invalid option -$OPTARG" >&2
    usage
    ;;
  esac
done

if [ -z "$PACKAGES" ]; then
       echo "No packages provided. Exiting."
       exit 1
fi

# DeepRacer Repos
sudo cp $DIR/../install_scripts/files/common/deepracer.asc /etc/apt/trusted.gpg.d/
sudo cp $DIR/../install_scripts/files/common/aws_deepracer.list /etc/apt/sources.list.d/

# Get mxcam
if [ ! -d "$DIR/../deps/geocam-bin-armhf" ]; then
       mkdir -p $DIR/../deps/
       cd $DIR/../deps/
       git clone https://github.com/doitaljosh/geocam-bin-armhf
fi

rm -rf $DIR/../pkg-build/aws* 
mkdir -p $DIR/../pkg-build $DIR/../pkg-build/src $DIR/../dist
cd $DIR/../pkg-build
mkdir -p $PACKAGES

# Check which packages we have
cd $DIR/../pkg-build/src
for pkg in $PACKAGES;
do
       if [ "$(compgen -G $pkg*.deb | wc -l )" -eq 0 ];
       then
              PACKAGES_DOWNLOAD="$PACKAGES_DOWNLOAD $pkg:amd64"
       fi
done

# Download missing AMD64 packages
if [ -n "$PACKAGES_DOWNLOAD" ];
then
       sudo apt-get update

       echo -e '\n### Downloading original packages ###\n'
       echo "Missing packages: $PACKAGES_DOWNLOAD"
       apt download $PACKAGES_DOWNLOAD
fi

# Determine target architecture
TARGET_ARCH=$(dpkg --print-architecture)
if [ "$TARGET_ARCH" != "arm64" ] && [ "$TARGET_ARCH" != "amd64" ]; then
       echo "Unsupported architecture: $TARGET_ARCH. Exiting."
       exit 1
fi

# Build required packages
cd $DIR/../pkg-build
for pkg in $PACKAGES; 
do
       if [ "$pkg" == "aws-deepracer-util" ];
       then
              VERSION=$(jq -r ".[\"aws-deepracer-util\"]" $DIR/versions.json)
              echo -e "\n### Building aws-deepracer-util $VERISON ###\n"
              dpkg-deb -R src/aws-deepracer-util_*amd64.deb aws-deepracer-util
              cd aws-deepracer-util
              rm -rf opt/aws/deepracer/camera/installed/bin/mxuvc \
                     opt/aws/deepracer/camera/installed/bin/querydump \
                     opt/aws/deepracer/camera/installed/lib
              cp $DIR/../deps/geocam-bin-armhf/files/usr/bin/mxcam opt/aws/deepracer/camera/installed/bin
              cp $DIR/files/aws_deepracer-community.list etc/apt/sources.list.d/aws_deepracer.list
              cp $DIR/files/otg_eth.sh opt/aws/deepracer/util/otg_eth.sh
              cp $DIR/files/isc-dhcp-server opt/aws/deepracer/util/isc-dhcp-server
              cp $DIR/files/deepracer_dhcp.conf opt/aws/deepracer/util/deepracer_dhcp.conf
              sed -i "s/Architecture: amd64/Architecture: $TARGET_ARCH/" DEBIAN/control
              sed -i "s/Version: .*/Version: $VERSION/" DEBIAN/control
              sed -i 's/pyclean -p aws-deepracer-util/ /' DEBIAN/prerm
              cd ..
              dpkg-deb --root-owner-group -b aws-deepracer-util
              dpkg-name -o aws-deepracer-util.deb
              FILE=$(compgen -G aws-deepracer-util*.deb)
              mv $FILE $(echo $DIR/dist/$FILE | sed -e 's/\+/\-/')
       fi

       if [ "$pkg" == "aws-deepracer-device-console" ];
       then
              VERSION=$(jq -r ".[\"aws-deepracer-device-console\"]" $DIR/versions.json)
              echo -e "\n### Building aws-deepracer-device-console $VERSION ###\n"
              dpkg-deb -R src/aws-deepracer-device-console_*amd64.deb aws-deepracer-device-console
              cd aws-deepracer-device-console
              sed -i 's/Architecture: amd64/Architecture: all/' DEBIAN/control
              sed -i "s/Version: .*/Version: $VERSION/" DEBIAN/control
              sed -i 's/pyclean -p aws-deepracer-device-console/ /' DEBIAN/prerm
              sed -i 's/.range-btn-minus button,.range-btn-plus button{background-color:#aab7b8!important;border-radius:4px!important;border:1px solid #879596!important}/.range-btn-minus button,.range-btn-plus button{background-color:#aab7b8!important;border-radius:4px!important;border:1px solid #879596!important;touch-action: manipulation;user-select: none;}/' opt/aws/deepracer/lib/device_console/static/bundle.css
              sed -i 's/isVideoPlaying: true/isVideoPlaying: false/' opt/aws/deepracer/lib/device_console/static/bundle.js
              sed -i 's/BATTERY_AND_NETWORK_DETAIL_API_CALL_FREQUENCY = 1000;/BATTERY_AND_NETWORK_DETAIL_API_CALL_FREQUENCY = 10000;/' opt/aws/deepracer/lib/device_console/static/bundle.js
              cp $DIR/files/login.html opt/aws/deepracer/lib/device_console/templates/
              echo "/opt/aws/deepracer/nginx/nginx_install_certs.sh" | tee -a DEBIAN/postinst >/dev/null
              echo "systemctl restart nginx.service" | tee -a DEBIAN/postinst >/dev/null
              cd ..
              dpkg-deb --root-owner-group -b aws-deepracer-device-console
              dpkg-name -o aws-deepracer-device-console.deb 
              FILE=$(compgen -G aws-deepracer-device-console*.deb)
              mv $FILE $(echo $DIR/dist/$FILE | sed -e 's/\+/\-/')
       fi

       if [ "$pkg" == "aws-deepracer-core" ];
       then
              VERSION=$(jq -r ".[\"aws-deepracer-core\"]" $DIR/versions.json)
              echo -e "\n### Building aws-deepracer-core $VERSION ###\n"
              dpkg-deb -R src/aws-deepracer-core_*amd64.deb aws-deepracer-core
              cd aws-deepracer-core
              sed -i "s/Architecture: amd64/Architecture: $TARGET_ARCH/" DEBIAN/control
              sed -i "s/Version: .*/Version: $VERSION/" DEBIAN/control
              sed -i 's/python-apt/python3-apt/' DEBIAN/control
              sed -i '/Depends/ s/$/, gnupg/' DEBIAN/control
              sed -i 's/pyclean -p aws-deepracer-core/\/usr\/local\/bin\/pyclean \/opt\/aws\/deepracer\/lib/' DEBIAN/prerm
              sed -i 's/ExecStop=\/opt\/aws\/deepracer\/util\/otg_eth.sh stop/KillSignal=2/' etc/systemd/system/deepracer-core.service
              rm -rf opt/aws/deepracer/lib/*
              cp $DIR/files/start_ros.sh opt/aws/deepracer
              cp -r $DIR/../install/* opt/aws/deepracer/lib/
              rm DEBIAN/preinst
              cd ..
              dpkg-deb --root-owner-group -b aws-deepracer-core
              dpkg-name -o aws-deepracer-core.deb 
              FILE=$(compgen -G aws-deepracer-core*.deb)
              mv $FILE $(echo $DIR/dist/$FILE | sed -e 's/\+/\-/')
       fi

       if [ "$pkg" == "aws-deepracer-sample-models" ];
       then
              VERSION=$(jq -r ".[\"aws-deepracer-sample-models\"]" $DIR/versions.json)
              echo -e "\n### Building aws-deepracer-sample-models $VERISON ###\n"
              dpkg-deb -R src/aws-deepracer-sample-models_*amd64.deb aws-deepracer-sample-models
              cd aws-deepracer-sample-models
              sed -i 's/Architecture: amd64/Architecture: all/' DEBIAN/control
              sed -i "s/Version: .*/Version: $VERSION/" DEBIAN/control
              cd ..
              dpkg-deb --root-owner-group -b aws-deepracer-sample-models
              dpkg-name -o aws-deepracer-sample-models.deb 
              FILE=$(compgen -G aws-deepracer-sample-models*.deb)
              mv $FILE $(echo $DIR/dist/$FILE | sed -e 's/\+/\-/')
       fi
done