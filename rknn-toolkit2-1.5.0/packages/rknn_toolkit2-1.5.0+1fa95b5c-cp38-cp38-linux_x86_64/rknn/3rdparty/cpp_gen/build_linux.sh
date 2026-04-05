#!/bin/bash
set -e

usage()
{
    echo "Now support rk3562/rk3566/rk3568/rk3588/rv1103/rv1106."
    echo "Usage as: $0 rk3562"
}

if [ $# == 0 ]; then
    usage
    exit 0
fi
echo "Build linux demo for "$1

if [ $1 == "rk3562" ]; then
  TARGET_SOC="rk356x"
elif [ $1 == "rk3566" ]; then
  TARGET_SOC="rk356x"
elif [ $1 == "rk3568" ]; then
  TARGET_SOC="rk356x"
elif [ $1 == "rk3588" ]; then
  TARGET_SOC="rk3588"
elif [ $1 == "rv1103" ]; then
  TARGET_SOC="rv110x"
elif [ $1 == "rv1106" ]; then
  TARGET_SOC="rv110x"
else 
    echo "$1 not recognize."
    usage
    exit 1
fi

echo "TARGET_SOC : $TARGET_SOC"

if [ $TARGET_SOC == "rv110x" ]; then
    echo "this script not support rv110x now"
    exit 1
else
    GCC_COMPILER=aarch64-linux-gnu
fi

ROOT_PWD=$( cd "$( dirname $0 )" && cd -P "$( dirname "$SOURCE" )" && pwd )

# build
BUILD_DIR=${ROOT_PWD}/build/build_linux_aarch64


if [ -z ${RKNPU2} ]
then
  echo "RKNPU2 was not set."
  echo "Please set RKNPU2 via 'export RKNPU2={rknpu2_path}'" 
  echo "or put this folder under the rknpu2."
  RKNPU2=$(pwd | sed 's/\(rknpu2\).*/\1/g')

  STR_MUST_EXIST="rknpu2"
  if [[ $RKNPU2 != *$STR_MUST_EXIST* ]]
  then
    exit
  fi
fi


if [[ ! -d "${BUILD_DIR}" ]]; then
  mkdir -p ${BUILD_DIR}
fi

cd ${BUILD_DIR}
cmake ../.. \
    -DTARGET_SOC=${TARGET_SOC} \
    -DCMAKE_C_COMPILER=${GCC_COMPILER}-gcc \
    -DCMAKE_CXX_COMPILER=${GCC_COMPILER}-g++ \
    -DRKNPU2=${RKNPU2}
make -j4
make install
cd -


