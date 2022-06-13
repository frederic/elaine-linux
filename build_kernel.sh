#!/bin/bash

exec_name=$0

set -o errtrace
trap 'echo Fatal error: script ${exec_name} aborting at line $LINENO, command \"$BASH_COMMAND\" returned $?; exit 1' ERR

arch=arm64
cross_compile=aarch64-linux-gnu-
cc_clang=aarch64-linux-gnu-gcc

cpu_num=$(grep -c processor /proc/cpuinfo)
readonly fctname="fct"
readonly bootdir=./arch/arm64/boot

function usage(){
  echo "Usage: ${exec_name} <board> [workspace path] [-d]"
  echo "supported boards: estelle-(p2, b1, b3, b4)"
  echo "                  newman-(p2, p2_1, b1, b3, b4)"
  echo "                  legion-p1"
  echo "                  elaine-(p1, p2, b1, b3, b4)"
  echo
  echo "-d : Build deterministicly. Use git revision and timestamp and"
  echo "     generic user and host for /proc/version"
}

function deterministic_build(){
  local deterministic_build=false
  if [[ "$1" == "-d" ]]; then
    deterministic_build=true
    shift
  fi

  # Set variables for deterministic kernel builds
  # These correspond to scripts/mkcompile_h
  if [[ "$deterministic_build" == true ]]; then
    export KBUILD_BUILD_VERSION=0
    export KBUILD_BUILD_TIMESTAMP="$(git log -1 --format=%cd) ($(git rev-parse HEAD | head -c 8))"
    export KBUILD_BUILD_USER=user
    export KBUILD_BUILD_HOST=host
  else
    # Add git hash to date. This corresponds to scripts/mkcompile_h
    # This shows up in /proc/version
    export KBUILD_BUILD_TIMESTAMP="$(date) ($(git rev-parse HEAD | head -c 8))"
  fi
}

function run_kernel_make(){
  echo "***** building $4 *****"
  make CLANG_TRIPLE=$cross_compile CC=$cc_clang CROSS_COMPILE=$1 ARCH=$3 -j$2 $4 CONFIG_DEBUG_SECTION_MISMATCH=y
  echo "***** completed building $4 *****"
}

function build_kernel(){
  local kernel_path=$(readlink -f $1)
  local defconfig_file_name=$2

  # If it is b series board, use bx defconfig.
  local board_type=`echo ${defconfig_file_name} | cut -d "-" -f2 | cut -c 1`
  if [[ "$board_type" == "b" ]]; then
    local product_surname=`echo ${defconfig_file_name} | cut -d "-" -f1`
    defconfig_file_name=${product_surname}-bx_defconfig
  fi

  pushd $kernel_path

  # Make a clean build and check .config and defconfig if different then abort.
  # make clean
  run_kernel_make $cross_compile $cpu_num $arch $defconfig_file_name
  #diff .config arch/arm64/configs/${defconfig_file_name}

  run_kernel_make $cross_compile $cpu_num $arch all

  popd
}

function build_dtb(){
  local kernel_path=$(readlink -f $1)
  local dtb_file_name=$2

  pushd $kernel_path
  run_kernel_make $cross_compile $cpu_num $arch $dtb_file_name
  popd
}

# Append $1 which is an integer value to the file at path $2. $1 is appended as
# a binary little endian integer.
append_uint32_le() {
  local val=$1
  local file=$2
  printf "0: %.8x" ${val} | sed -E 's/0: (..)(..)(..)(..)/0: \4\3\2\1/' \
    | xxd -r -g0 >> ${file}
}

# Pack the kernel along with its dtb file
# The format is [header][compressed kernel][dtb file]
#
# header is little endian and consists of
# struct {
#   char magic[KDTB_MAGIC_SZ];
#   uint32_t kernel_size;
#   uint32_t dtb_size;
# };
pack_kernel() {
  local dtb_file=$1
  local product=$2
  local board_name=$3
  local factory=$4
  local compressed_kernel=${bootdir}/Image.gz
  local magic="KDTB"
  if [ ! -z ${factory} ]; then
    local packed_kernel=${bootdir}/${factory}_kernel.${product}.gz-dtb.${board_name}
  else
    local packed_kernel=${bootdir}/kernel.${product}.gz-dtb.${board_name}
  fi

  echo -n ${magic} > ${packed_kernel}
  append_uint32_le $(stat -c %s ${compressed_kernel}) ${packed_kernel}
  append_uint32_le $(stat -c %s ${dtb_file}) ${packed_kernel}

  cat ${compressed_kernel} ${dtb_file} >> ${packed_kernel}
}

if (( $# < 1 ))
then
  usage
  exit 2
fi

readonly kernel_dir=.
readonly board=$1
readonly workspace_path=$2
deterministic_build $3

product=`echo ${board} | cut -d "-" -f1`

case $product in
  estelle | newman | legion | puddy | elaine)
    build_kernel ${kernel_dir} ${board}_defconfig
    dtb_file_name=${board}.dtb
    path_to_dtb_file=arch/arm64/boot/dts/amlogic/${dtb_file_name}
    build_dtb ${kernel_dir} ${dtb_file_name}
    pack_kernel $path_to_dtb_file ${product} ${board}
    # build factory kernel
    fct_dtb_file_name=${fctname}_${dtb_file_name}
    path_to_fct_dtb_file=arch/arm64/boot/dts/amlogic/${fct_dtb_file_name}
    build_dtb ${kernel_dir} ${fct_dtb_file_name}
    pack_kernel $path_to_fct_dtb_file ${product} ${board} ${fctname}

    if [ ! -z $workspace_path ]; then
      prebuilt_path=${workspace_path}/vendor/amlogic/${product}/prebuilt
      kernel_path=${prebuilt_path}/kernel
      mkdir -p ${kernel_path}
      cp ${bootdir}/kernel.${product}.gz-dtb.${board} \
        ${kernel_path}
      cp ${path_to_dtb_file} ${kernel_path}
      # Factory directory
      fct_kernel_path=${prebuilt_path}/factory/kernel
      mkdir -p ${fct_kernel_path}
      cp ${bootdir}/${fctname}_kernel.${product}.gz-dtb.${board} \
        ${fct_kernel_path}/kernel.${product}.gz-dtb.${board}
    fi
    ;;
  *)
    echo "unknown board: $board"
    exit 1
esac
