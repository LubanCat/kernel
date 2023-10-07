# Android Kernel Configs

## How are kernel config settings typically stored?

When building the Linux kernel for a particular platform one usually begins by
basing the kernel configuration off of a particular defconfig. The platform’s
defconfig contains all of the Linux kconfig settings required to properly
configure the kernel build (features, default system parameters, etc) for that
platform. Defconfig files are typically stored in the kernel tree at
`arch/*/configs/`.

It may be desirable to modify the kernel configuration beyond what the hardware
platform requires in order to support a particular hardware or software
feature. Storing these kernel configuration changes is often done using
fragments. These are files which have the same format as a defconfig but are
typically much smaller, as they only contain the kernel configuration settings
needed to support the hardware or software feature/behavior in question.
Maintainers of hardware extensions or software features can use fragments to
compactly express their kernel requirements. The fragments can be combined
with a platform defconfig using the kernel's make rules or using the
`scripts/kconfig/merge_config.sh` script in the kernel tree.

## How are Android's kernel configs stored?

The kernel configs are stored in the [kernel/configs repo](https://android.googlesource.com/kernel/configs/).

Kernel configuration settings that must be present for Android to function are
located in the base config fragment, `android-base.config`. Configuration settings
that enhance Android’s functionality in some way but are not required for it to
run are located in the recommended config fragment, `android-recommended.config`.

Some kernel config requirements only apply on certain architectures. Other
requirements only apply if some other kernel config option has a particular
value. The platform owner may also have a choice between several config
options. These types of constraints cannot be expressed with a simple kernel
config fragment. In releases up to and including Android P, kernel config
requirements that are specific to a particular architecture are contained in
architecture-specific base config fragments, such as
`android-base-arm64.config`. If an architecture-specific base config fragment
does not exist for a particular architecture in Android P or an earlier
release, it means there are no required kernel config options for Android
specific to that architecture. Note that the architecture-agnostic kernel
config requirements from `android-base.config` still apply in that case.

In releases after Android P the architecture-specific base config fragments are
removed, and conditional kernel config requirements are stored in
`android-base-conditional.xml`.

In Android R or above, additional config fragments are added conditionally
on build variants. In particular, `non_debuggable.config` contains additional
requirements for user builds.

Kernel configs vary by kernel version, so there are sets of kernel configs for
each version of the kernel that Android supports.

## How can I easily combine platform and required Android configs?

Assuming you already have a minimalist defconfig for your platform, a possible
way to enable these options would be to use the aforementioned
`merge_config.sh` script in the kernel tree. From the root of the kernel tree:

```sh
ARCH=<arch> scripts/kconfig/merge_config.sh <...>/<platform>_defconfig <...>/android-base.config <...>/android-base-<arch>.config <...>/android-recommended.config
```

This will generate a `.config` that can then be used to save a new defconfig or
compile a new kernel with Android features enabled.

The kernel build system also supports merging in config fragments directly. The
fragments must be located in the `kernel/configs` directory of the kernel tree
and they must have filenames that end with the extension ".config". The
platform defconfig must also be located in `arch/<arch>/configs/`. Once these
requirements are satisfied, the full defconfig can be prepared with:

```sh
make ARCH=<arch> <platform>_defconfig android-base.config android-base-<arch>.config android-recommended.config
```

If there is an `android-base-conditional.xml` file for your release/kernel
version combination, it is necessary to review it and manually edit your
defconfig to satisfy any applicable requirements.

## Are the config requirements tested?

Starting with Android O the base kernel configs are not just advisory. They are
tested as part of
[VTS](https://android.googlesource.com/platform/test/vts-testcase/hal/+/master/treble/framework_vintf/AndroidTest.xml)
(specifically the SystemVendorTest.KernelCompatibility subtest of
CtsOnGsiTrebleFrameworkVintfTest), and also during device boot when the vendor
interface (which includes the kernel configuration) and framework compatibility
matrix are compared.

## Ensuring Device Upgradability

Devices launched with prior releases of Android must be able to upgrade to
later releases of Android. This means that AOSP must function not only with
device kernels that adhere to the Android kernel configs of the current
release, but also with those device kernels that adhere to the configs of past
releases. To facilitate that in the VtsKernelConfig test and in the framework
compatibility matrix, past versions of the Android kernel config requirements
are stored in the kernel/configs repo. During tests the appropriate versions
of the configs are accessed depending on the launch level of the device.

If you are adding a new feature to AOSP which depends on a particular kernel
configuration value, either that kernel configuration value must already be
present in the base android config fragments of past releases still on the
supported upgrade path, or the feature must be designed in a way to degrade
gracefully when the required kernel configuration is not present (and not be
essential to AOSP’s overall functionality). All configs on the supported
upgrade path are in the kernel/configs repo.

Support for kernel configs from previous dessert releases is dropped from AOSP
when the upgrade path from that dessert release is no longer supported.

# Organization and Maintenance of the Kernel Config Repo

The top level of the kernel configs repo contains directories for each
supported kernel version. These contain the kernel config requirements (and
recommendations) for the next release. There are also directories at
the top level for previous releases. These directories contain the
final kernel config requirements (for all supported kernel versions) for those
releases and must not be changed once those releases have been
published. AOSP must support all kernel configurations in this repo.

For release branches the structure is similar, except there are no configs at
the top level. When a release is branched from master the top-level configs are
copied into a new directory for the release (this change is propagated to
master) and the top-level configs are removed (this change is not propagated to
master) since no development beyond that release is done in that branch.

## I want to add/modify/remove a kernel config requirement. What do I do?

Modify the top level kernel configs in AOSP. Make sure to modify the configs
for all applicable kernel versions. Do not modify the config fragments in
release directories.

Because there is no tool to consistently generate these config fragments,
please keep them alphabetically (not randomly) sorted.

### `android-x.y/android-base.config`

This file lists all common kernel configuration requirements on kernel version
`x.y`.

### `android-x.y/android-base-conditional.xml`

Contains the following:

* Minimum LTS required
* Conditional requirements.

### `android-x.y/Android.bp`

Build rules from the aforementioned files to a
[framework compatibility matrix](https://source.android.com/devices/architecture/vintf/comp-matrices)
. See
[this link](https://source.android.com/devices/architecture/vintf/match-rules#kernel)
for details of the output format.

## I want to freeze/release the current kernel requirements. What do I do?

Prior to a [FCM Version release](https://source.android.com/devices/architecture/vintf/fcm#new-fcm-versions)
(often accompanied with a dessert release as well), the kernel requirements must
be frozen. Follow the following steps

* Copy the top-level `android-*` directories to a release directory, preferably
  with the dessert name (for example, `q`).
  * Remove top-level `android-*` directories. This change is not propagated to
    master.
* Edit the new `<dessert>/android-*/Android.bp` files and rename the modules.
  For example, change `kernel_config_current_4.9` in `q/android-4.9/Android.bp`
  to `kernel_config_q_4.9`
* Under `hardware/interfaces/compatibility_matrices/Android.bp`, edit
  `kernel_config` field for the `framework_compatibility_matrix.current.xml`
  to use the new modules.
  * `framework_compatibility_matrix.current.xml` will be renamed to
    `framework_compatibility_matrix.<level>.xml` as part of the FCM Version
    release, which is a separate process.

## I want to edit a released kernel requirement. What do I do?

Don't edit a released kernel requirement unless necessary. If you have to make
such a change, after discussing the change with maintainers, keep in mind that
you **CANNOT** make a requirement more restrictive. Specifically,

### Allowed
* Support a new kernel version by creating a new `<dessert>/android-x.y`
  directory
* Remove a line from `<dessert>/android-*/android-base.config`
* Remove a line from `<dessert>/android-*/android-base-*.config`
* In `<dessert>/android-*/android-base-conditional.xml`
    * Lower minimum LTS requirement from `x.y.u` to `x.y.v` (where `v < u`)
    * Remove a `<group>`
    * Add a condition `<group><conditions><config>`
    * Remove a conditional requirement `<group><config>`

### Not allowed
* Add or change a line from `<dessert>/android-*/android-base.config`
* Add or change a line from `<dessert>/android-*/android-base-*.config`
* Add new conditional requirements `<dessert>/android-*/android-base-*.config`
* Rename existing conditional requirements `<dessert>/android-*/android-base-*.config`
* In `<dessert>/android-*/android-base-conditional.xml`
    * Raise minimum LTS requirement from `x.y.u` to `x.y.v` (where `v < u`)
    * Add a new `<group>`
    * Remove or change a condition `<conditions><config>`
    * Add or change a conditional requirement `<group><config>`
* Other changes that are not in the [Allowed](#allowed) list
