#!/usr/bin/env bash
set -e

export DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

PACKAGES="aws-deepracer-core"

while getopts "p:v:" opt; do
  case $opt in
  p)
    PACKAGES=$OPTARG
    ;;
  v)
    VERSION=$OPTARG
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

if [ -z "$VERSION" ]; then
       echo "No version provided. Exiting."
       exit 1
fi

# DeepRacer Repos
sudo cp $DIR/files/deepracer.asc /etc/apt/trusted.gpg.d/
sudo cp $DIR/files/aws_deepracer.list /etc/apt/sources.list.d/
sudo apt-get update

rm -rf $DIR/pkg-build/aws* 
mkdir -p $DIR/pkg-build $DIR/pkg-build/src $DIR/dist
cd $DIR/pkg-build
mkdir -p $PACKAGES

# Check which packages we have
cd $DIR/pkg-build/src
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

# Build required packages
cd $DIR/pkg-build
for pkg in $PACKAGES; 
do
       if [ "$pkg" == "aws-deepracer-core" ];
       then
              echo -e "\n### Building aws-deepracer-core $VERSION ###\n"
              dpkg-deb -R src/aws-deepracer-core_*amd64.deb aws-deepracer-core
              cd aws-deepracer-core
              sed -i "s/Version: .*/Version: $VERSION/" DEBIAN/control
              sed -i '/Depends/ s/$/, gnupg, ros-foxy-ros-base, ros-foxy-std-msgs, ros-foxy-sensor-msgs, ros-foxy-image-transport, ros-foxy-compressed-image-transport, ros-foxy-pybind11-vendor, ros-foxy-cv-bridge, python3-websocket, python3-click/' DEBIAN/control
              rm -rf opt/aws/deepracer/lib/*
              cp $DIR/files/start_ros.sh opt/aws/deepracer
              cp -r $DIR/ws/install/* opt/aws/deepracer/lib/
              rm DEBIAN/preinst
              cd ..
              dpkg-deb --root-owner-group -b aws-deepracer-core
              dpkg-name -o aws-deepracer-core.deb 
              FILE=$(compgen -G aws-deepracer-core*.deb)
              mv $FILE $(echo $DIR/dist/$FILE | sed -e 's/\+/\-/')
       fi

       if [ "$pkg" == "aws-deepracer-device-console" ];
       then
              echo -e "\n### Building aws-deepracer-device-console $VERSION ###\n"
              dpkg-deb -R src/aws-deepracer-device-console_*amd64.deb aws-deepracer-device-console
              cd aws-deepracer-device-console
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
done