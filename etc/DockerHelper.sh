#!/usr/bin/env bash

set -euo pipefail

cd "$(dirname $(readlink -f $0))/../"

baseDir="$(pwd)"
# docker hub organization/user from where to pull/push images
org=openroad
depsPrefixesFile="/etc/openroad_deps_prefixes.txt"
args=("${@}")

_help() {
    cat <<EOF
usage: $0 [CMD] [OPTIONS]

  CMD:
  create                        Create a docker image
  test                          Test the docker image
  push                          Push the docker image to Docker Hub

  OPTIONS:
  -os=OS_NAME                   Choose between:
                                  ubuntu20.04, ubuntu22.04 (default),
                                  ubuntu24.04, rockylinux9, opensuse or debian11.
  -target=TARGET                Choose target for the Docker image:
                                  'dev': os + packages to compile app
                                  'builder': os + packages to compile app +
                                             copy source code and build app
                                  'binary': os + packages to run a compiled
                                            app + binary set as entrypoint
  -threads                      Max number of threads to use if compiling.
                                  Default = \$(nproc)
  -sha                          Use git commit sha as the tag image. Default is
                                  'latest'.
  -h -help                      Show this message and exits
  -local                        Installs with prefix /home/openroad-deps
  -username                     Docker Username
  -password                     Docker Password
  -deps-prefixes-path=PATH      Path where the file with dependency prefixes should be stored (in Docker image)
  -smoke                        Only run smoke test.

EOF
    exit "${1:-1}"
}

_setup() {
    commitSha="$(git rev-parse HEAD)"
    case "${compiler}" in
        "gcc" | "clang" )
            ;;
        * )
            echo "Compiler ${compiler} not supported" >&2
            _help
            ;;
    esac
    case "${os}" in
        "centos7")
            osBaseImage="centos:centos7"
            ;;
        "ubuntu20.04")
            osBaseImage="ubuntu:20.04"
            ;;
        "ubuntu22.04")
            osBaseImage="ubuntu:22.04"
            ;;
        "ubuntu24.04")
            osBaseImage="ubuntu:24.04"
            ;;
        "opensuse")
            osBaseImage="opensuse/leap"
            ;;
        "debian11")
            osBaseImage="debian:bullseye"
            ;;
        "rockylinux9")
            osBaseImage="rockylinux:9"
            ;;
        *)
            echo "Target OS ${os} not supported" >&2
            _help
            ;;
    esac
    imageName="${IMAGE_NAME_OVERRIDE:-"${org}/${os}-${target}"}"
    if [[ "${useCommitSha}" == "yes" ]]; then
        imageTag="${commitSha}"
    else
        imageTag="latest"
    fi
    case "${target}" in
        "builder" )
            fromImage="${FROM_IMAGE_OVERRIDE:-"${org}/${os}-dev"}:${imageTag}"
            context="."
            buildArgs="--build-arg compiler=${compiler}"
            buildArgs="${buildArgs} --build-arg numThreads=${numThreads}"
            buildArgs="${buildArgs} --build-arg depsPrefixFile=${depsPrefixesFile}"
            if [[ "${isLocal}" == "yes" ]]; then
                buildArgs="${buildArgs} --build-arg LOCAL_PATH=${LOCAL_PATH}/bin"
            fi
            imageName="${IMAGE_NAME_OVERRIDE:-"${imageName}-${compiler}"}"
            ;;
        "dev" )
            fromImage="${FROM_IMAGE_OVERRIDE:-${osBaseImage}}"
            context="etc"
            buildArgs="-save-deps-prefixes=${depsPrefixesFile}"
            if [[ "${isLocal}" == "yes" ]]; then
                buildArgs="${buildArgs} -prefix=${LOCAL_PATH}"
            fi
            if [[ "${equivalenceDeps}" == "yes" ]]; then
                buildArgs="${buildArgs} -eqy"
            fi
            if [[ "${CI}" == "yes" ]]; then
                buildArgs="${buildArgs} -ci"
            fi
            if [[ "${buildArgs}" != "" ]]; then
                buildArgs="--build-arg INSTALLER_ARGS='${buildArgs}'"
            fi
            ;;
        "binary" )
            fromImage="${FROM_IMAGE_OVERRIDE:-${org}/${os}-dev}:${imageTag}"
            context="etc"
            copyImage="${COPY_IMAGE_OVERRIDE:-"${org}/${os}-builder-${compiler}"}:${imageTag}"
            buildArgs="--build-arg copyImage=${copyImage}"
            ;;
        *)
            echo "Target ${target} not found" >&2
            _help
            ;;
    esac
    imagePath="${imageName}:${imageTag}"
    buildArgs="--build-arg fromImage=${fromImage} ${buildArgs}"
    file="docker/Dockerfile.${target}"
}

_test() {
    echo "Run regression test on ${imagePath}"
    case "${target}" in
        "builder" )
            ;;
        *)
            echo "Target ${target} is not valid candidate to run regression" >&2
            _help
            ;;
    esac
    if [[ "$(docker images -q ${imagePath} 2> /dev/null)" == "" ]]; then
        echo "Could not find ${imagePath}, will attempt to create it" >&2
        _create
    fi
    if [[ "${testTarget}" == "smoke" ]]; then
        docker run --rm "${imagePath}" bash -c './build/bin/openroad -help'
    else
        docker run --rm "${imagePath}" "./docker/test_wrapper.sh" "${compiler}" "ctest --test-dir build -j ${numThreads}"
    fi
}

_checkFromImage() {
    set +e
    # Check if the image exists locally
    if docker image inspect "${fromImage}" > /dev/null 2>&1; then
        echo "Image '${fromImage}' exists locally."
    else
        echo "Image '${fromImage}' does not exist locally. Attempting to pull..."
        # Try to pull the image
        if docker pull "${fromImage}"; then
            echo "Successfully pulled '${fromImage}'."
        else
            echo "Unable to pull '${fromImage}'. Attempting to build..."
            # Build the image using the createImage command
            newArgs=""
            newTarget=""
            for arg in "${args[@]}"; do
                # Check if the argument matches -target=builder
                if [[ "${arg}" == "-target=builder" ]]; then
                    newTarget="dev"
                elif [[ "${arg}" == "-target=binary" ]]; then
                    newTarget="builder"
                else
                    newArgs+=" ${arg}"
                fi
            done
            if [[ "${newTarget}" == "" ]]; then
                echo "Error"
                exit 1
            fi
            newArgs+=" -target=${newTarget}"
            createImage="$0 ${newArgs}"
            echo "Running: ${createImage}"
            if ${createImage}; then
                echo "Successfully built '${newTarget}' image."
            else
                echo "Failed to build '${newTarget}' needed for '${target}' target."
                return 1
            fi
        fi
    fi
    set -e
}

_create() {
    if [[ "${target}" == "binary" ]]; then
        _checkFromImage "builder"
    fi
    if [[ "${target}" == "builder" ]]; then
        _checkFromImage "dev"
    fi
    echo "Create docker image ${imagePath} using ${file}"
    eval docker buildx build \
        --file "${file}" \
        --tag "${imagePath}" \
        ${buildArgs} \
        "${context}"
}

_push() {
    case "${target}" in
        "dev" )
            read -p "Will push docker image ${imagePath} to DockerHub [y/N]" -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$  ]]; then
                mkdir -p build

                OS_LIST="centos7 ubuntu20.04 ubuntu22.04"
                # create image with sha and latest tag for all os
                for os in ${OS_LIST}; do
                    ./etc/DockerHelper.sh create -target=dev \
                        2>&1 | tee build/create-${os}-latest.log &
                done
                wait

                for os in ${OS_LIST}; do
                    ./etc/DockerHelper.sh create -target=dev -sha \
                        2>&1 | tee build/create-${os}-${commitSha}.log &
                done
                wait

                # test image with sha and latest tag for all os and compiler
                for os in ${OS_LIST}; do
                    ./etc/DockerHelper.sh test -target=builder -sha \
                        2>&1 | tee build/test-${os}-gcc-latest.log &
                done
                wait

                for os in ${OS_LIST}; do
                    ./etc/DockerHelper.sh test -target=builder -sha -compiler=clang \
                        2>&1 | tee build/test-${os}-clang-latest.log &
                done
                wait

                for os in ${OS_LIST}; do
                    echo [DRY-RUN] docker push openroad/${os}-dev:latest
                    echo [DRY-RUN] docker push openroad/${os}-dev:${commitSha}
                done

            else
                echo "Will not push."
            fi
            ;;
        *)
            echo "Target ${target} is not valid candidate for push to DockerHub." >&2
            _help
            ;;
    esac
}

#
# MAIN
#

# script has at least 1 argument, the rule
if [[ $# -lt 1 ]]; then
    echo "Too few arguments" >&2
    _help
fi

_rule="_${1}"
shift 1

# check if the rule is exists
if [[ -z $(command -v "${_rule}") ]]; then
    echo "Command ${_rule/_/} not found" >&2
    _help
fi

# default values, can be overwritten by cmdline args
os="ubuntu22.04"
target="dev"
compiler="gcc"
useCommitSha="no"
isLocal="no"
equivalenceDeps="yes"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
  numThreads=$(nproc --all)
elif [[ "$OSTYPE" == "darwin"* ]]; then
  numThreads=$(sysctl -n hw.ncpu)
else
  cat << EOF
WARNING: Unsupported OSTYPE: cannot determine number of host CPUs"
  Defaulting to 2 threads. Use --threads N to use N threads"
EOF
  numThreads=2
fi
LOCAL_PATH="/home/openroad-deps"
testTarget="ctest"

while [ "$#" -gt 0 ]; do
    case "${1}" in
        -h|-help)
            _help 0
            ;;
        -compiler=*)
            compiler="${1#*=}"
            ;;
        -smoke )
            testTarget="smoke"
            ;;
        -os=* )
            os="${1#*=}"
            ;;
        -target=* )
            target="${1#*=}"
            ;;
        -threads=* )
            numThreads="${1#*=}"
            ;;
        -sha )
            useCommitSha=yes
            ;;
        -local )
            isLocal=yes
            ;;
        -no_eqy )
            equivalenceDeps=no
            ;;
        -tag=* )
            tag="${1#*=}"
            ;;
        -deps-prefixes-path=* )
            depsPrefixesFile="${1#-deps-prefixes-path=}"
            ;;
        -os | -target | -compiler | -threads | -username | -password | -tag | -deps-prefixes-path )
            echo "${1} requires an argument" >&2
            _help
            ;;
        *)
            echo "unknown option: ${1}" >&2
            _help
            ;;
    esac
    shift 1
done

if [[ "${numThreads}" == "-1" ]]; then
    if [[ "${OSTYPE}" == "linux-gnu"* ]]; then
        numThreads=$(nproc --all)
    elif [[ "${OSTYPE}" == "darwin"* ]]; then
        numThreads=$(sysctl -n hw.ncpu)
    else
        numThreads=2
        cat << EOF
[WARNING] Unsupported OSTYPE: cannot determine number of host CPUs"
  Defaulting to 2 threads. Use --threads N to use N threads"
EOF
    fi
fi

_setup

"${_rule}"
