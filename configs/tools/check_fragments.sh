#!/bin/bash

RETVAL=0
ARCHES="arm64 arm x86"
if [ -z "$ANDROID_BUILD_TOP" ]; then
	echo "ANDROID_BUILD_TOP not set, exiting"
	exit 1
fi
AFRAGS="$ANDROID_BUILD_TOP/kernel/configs"
KERNEL_VERSIONS=`ls -d $AFRAGS/android-* | xargs -n 1 basename`

check_fragment()
{
	ERRORS=0

	while read line; do
		grep -q "$line" .config
		if [ $? -ne 0 ]
		then
			echo "Error, $line not found in merged config."
			ERRORS=1
		fi
	done < <(grep -v -E "^#  " $1)

	if [ $ERRORS -ne 0 ]
	then
		echo "Errors encountered while checking $1"
		RETVAL=1
	else
		echo "Fragment $1 is okay"
	fi
	echo ""
}

check_arches()
{
	for arch in $ARCHES; do
		rm .config
		make ARCH=$arch allnoconfig
		FRAGMENTS="$AFRAGS/$version/android-base.config \
			   $AFRAGS/$version/android-recommended.config"
		if [ -f $AFRAGS/$version/android-base-$arch.config ]; then
			FRAGMENTS="$FRAGMENTS $AFRAGS/$version/android-base-$arch.config"
		fi
		if [ -f $AFRAGS/$version/android-recommended-$arch.config ]; then
			FRAGMENTS="$FRAGMENTS $AFRAGS/$version/android-recommended-$arch.config"
		fi
		ARCH=$arch scripts/kconfig/merge_config.sh .config $FRAGMENTS &> /dev/null
		for f in $FRAGMENTS; do
			check_fragment $f
		done
	done
}

check_configs()
{
	for version in $KERNEL_VERSIONS; do
		echo "Changing to $KERNEL_PATH"
		cd $KERNEL_PATH
		git checkout $version
		check_arches
	done
}

show_help()
{
	cat << EOF
Usage: ${0##*/} [-h] [-a arches] [-v versions] -k path-to-kernel
Check that the kernel config fragments are consistent with the Kconfig files
in the given kernel versions. This requires an android common kernel repo to
be checked out and made available to this script via the -k option or the
KERNEL_PATH environment variable. Note this script does not verify the
configs in the conditional XML fragments.

	-h		display this help and exit
	-a arches	quote-enclosed whitespace separated list of
                        architectures to check, valid architectures are
			arm64, arm, and x86
	-k kernel	path to android common kernel repo
	-v versions	quote-enclosed whitespace separated list of
			kernel versions to check (android-x.y)
EOF
}

OPTIND=1
while getopts "h?a:v:k:" opt; do
	case "$opt" in
		h|\?)
			show_help
			exit 0
			;;
		a)
			ARCHES="$OPTARG"
			;;
		v)
			KERNEL_VERSIONS="$OPTARG"
			;;
		k)
			KERNEL_PATH="${OPTARG/#\~/$HOME}"
			;;
	esac
done

check_configs
