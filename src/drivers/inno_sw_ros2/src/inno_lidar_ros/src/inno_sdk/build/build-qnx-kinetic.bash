#!/bin/bash 
set -x
set -e

if [ "$#" -ne 1 ]; then
    echo "Usage: " $0 " build-tag"
    echo "Example: " $0 " release-1.4.0-rc7"
    exit
fi

script_name=$0
script_full_path=$(dirname "$0")

NO_UPLOAD_SCRIPT=${script_full_path}/build-qnx-kinetic-no-upload.bash
S3_UPLOAD=${script_full_path}/s3_upload.py

git pull
git checkout $1

${NO_UPLOAD_SCRIPT} $1

BITBUCKET_TAG=$1-qnx

python ${S3_UPLOAD} --bucket iv-release --artifact output/*${BITBUCKET_TAG}-public.* --bucket_key release/TAG/${BITBUCKET_TAG}
python ${S3_UPLOAD} --bucket iv-bitbucket-pipeline --artifact output/*${BITBUCKET_TAG}-internal.* --bucket_key build/TAG/${BITBUCKET_TAG}
