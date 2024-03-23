#!/bin/bash
#
# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of taproot-scripts.
#
# taproot-scripts is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# taproot-scripts is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with taproot-scripts.  If not, see <https://www.gnu.org/licenses/>.
#
# This script validates that the git repository has the most up to date
# lbuild directory structure. Note at this script may change the state
# of your repository's generated code and thus you should not use it on
# a repository that has uncommitted changes.

# Parameters:
# $1 : Path to the lbuild-based directory (my-repo/my-project-repo)
# $2 : Path to the external taproot folder (my-repo/taproot)
# $3 : (Optional) Space-delimited string containing acceptable remote taproot branches
#       to base this project on. Default value is "release".

if [[ "$#" < 2 ]]; then
    echo "usage: ./check_lbuild_diff.sh ./path/to/lbuild/dir ./path/to/taproot \"SPACE_DELIMITED_TAPROOT_BRANCHES\""
    exit 1
fi

LBUILD_DIR=$1
TAPROOT_DIR=$2
TEMP_DIR="tmp"

if [[ "$#" > 2 ]]; then
    TAPROOT_BRANCHES=$3
else
    TAPROOT_BRANCHES="release"
fi

# First ensure the taproot commit is actually
# on the correct taproot branch
cd $TAPROOT_DIR
TAPROOT_COMMIT_SHA="$(git rev-parse HEAD)"
ON_ACCEPTABLE_BRANCH=0
for BRANCH in TAPROOT_BRANCHES
do
    SHA_ON_BRANCH="$(git branch -r --contains $TAPROOT_COMMIT_SHA | grep $BRANCH)"
    if [[ !(-z $SHA_ON_BRANCH) ]]; then
        ON_ACCEPTABLE_BRANCH=1
        break
    fi
done

if [[ !(ON_ACCEPTABLE_BRANCH) ]]; then
    echo "sha not valid, must be on one of $TAPROOT_BRANCHES"
    exit 1
fi
cd -

# Next ensure generated taproot changes match the taproot repo's generated
# taproot
cd "$LBUILD_DIR"

cp -r "taproot" $TEMP_DIR

rm -rf "taproot"

lbuild build
if [[ "$?" != 0 ]]; then
    exit 1
fi

# Remove all project.xml.log files that were generated when running lbuild build
shopt -s globstar  # enable globstar, double astrisk (https://askubuntu.com/questions/1010707/how-to-enable-the-double-star-globstar-operator)
rm -f ./**/project.xml.log

if [[ ! -z "$(git diff --no-index "$TEMP_DIR" "taproot")" ]]; then
    echo "Generated lbuild is different, diff:"
    git diff --no-index "$TEMP_DIR" "taproot"
    rm -r $TEMP_DIR
    exit 1
else
    rm -r $TEMP_DIR
    exit 0
fi
