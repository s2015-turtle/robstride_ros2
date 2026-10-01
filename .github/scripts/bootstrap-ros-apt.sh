#!/usr/bin/env bash
set -euo pipefail

# Avoid setup-ros's unchecked latest-release lookup (ros-tooling/setup-ros#892).
# Digests are from the official ros-infrastructure/ros-apt-source 1.3.0 release.
version=1.3.0
case "${1:-false}" in
  false) package=ros2-apt-source; repository=ros2 ;;
  true) package=ros2-testing-apt-source; repository=ros2-testing ;;
  *) echo "Expected use-ros2-testing to be true or false" >&2; exit 1 ;;
esac
codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
case "${package}:${codename}" in
  ros2-apt-source:jammy)
    checksum=110b9a462d55252decb8b7c816f61c2ba0d9890ce5fb93ac504e97cae5860d76 ;;
  ros2-apt-source:noble)
    checksum=f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296 ;;
  ros2-apt-source:resolute)
    checksum=e70bc980395f4a3b366b9c5e7f8f9b4d63a4064fb115f6ff3bc52f7faa02ef69 ;;
  ros2-testing-apt-source:jammy)
    checksum=2a872d898ca19abd6d7bcda2d75e455a2213c801696f99f3ad1dbc663e07da03 ;;
  ros2-testing-apt-source:noble)
    checksum=bca6a56c39f2c9315f2aee82cd27a88fa16d7b5572874722c6ed645132d2103b ;;
  ros2-testing-apt-source:resolute)
    checksum=20e38bf35f8de1a50da1a86fbfa391ca87db2d0e90010a92cd2639cd1efbe749 ;;
  *) echo "Unsupported ROS APT source: ${package}:${codename}" >&2; exit 1 ;;
esac

root=()
if [ "$(id -u)" -ne 0 ]; then
  root=(sudo)
fi
apt_options=(-o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30)
"${root[@]}" apt-get "${apt_options[@]}" update
"${root[@]}" apt-get "${apt_options[@]}" install --no-install-recommends -y curl ca-certificates

deb=$(mktemp --suffix=.deb)
trap 'rm -f "${deb}"' EXIT
url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/${package}_${version}.${codename}_all.deb"
curl --fail --show-error --silent --location \
  --retry 3 --retry-all-errors --retry-delay 2 --retry-max-time 180 \
  --connect-timeout 15 --max-time 60 --output "${deb}" "${url}"
printf '%s  %s\n' "${checksum}" "${deb}" | sha256sum --check --strict
"${root[@]}" dpkg -i "${deb}"

# setup-ros skips its download only after the selected repository is indexed.
"${root[@]}" apt-get "${apt_options[@]}" update
apt-cache policy | grep --fixed-strings "http://packages.ros.org/${repository}/ubuntu"
