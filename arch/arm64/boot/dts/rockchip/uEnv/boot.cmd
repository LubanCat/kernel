echo [boot.cmd] run boot.cmd scripts ...;

setenv initrd_load
setenv kernel_load

if test -e ${devtype} ${devnum}:${distro_bootpart} /uEnv/uEnv.txt; then 

    echo [boot.cmd] load ${devtype} ${devnum}:${distro_bootpart} ${env_addr_r} /uEnv/uEnv.txt ...;
    load ${devtype} ${devnum}:${distro_bootpart} ${env_addr_r} /uEnv/uEnv.txt;

    echo [boot.cmd] Importing environment from ${devtype} ...
    env import -t ${env_addr_r} 0x8000

    if test "${stressapptest_enable}" = "1"; then
        echo [boot.cmd] Start the stressapptest test. Test duration: "${stressapptest_time}"s
        if stressapptest "${stressapptest_time}"; then
            setenv stressapptest_result "pass"
        else
            setenv stressapptest_result "fail"
        fi
        echo [boot.cmd] StressAppTest Result: "${stressapptest_result}"
    fi

    setenv bootargs ${bootargs} root=/dev/mmcblk${devnum}p3 ${cmdline}
    if test -n "${stressapptest_result}"; then
        setenv bootargs ${bootargs} stressapptest_result=${stressapptest_result}
    fi
    printenv bootargs

    echo "[boot.cmd] try load ${devtype} ${devnum}:${distro_bootpart} ${ramdisk_addr_r} /initrd ..."
    if load ${devtype} ${devnum}:${distro_bootpart} ${ramdisk_addr_r} /initrd; then
        setenv initrd_size ${filesize}
        setenv initrd_load yes
    else
        echo "[boot.cmd] try load ${devtype} ${devnum}:${distro_bootpart} ${ramdisk_addr_r} /initrd-${uname_r} ..."
        if load ${devtype} ${devnum}:${distro_bootpart} ${ramdisk_addr_r} /initrd-${uname_r}; then
            setenv initrd_size ${filesize}
            setenv initrd_load yes
        fi
    fi

    echo "[boot.cmd] try load ${devtype} ${devnum}:${distro_bootpart} ${kernel_addr_r} /Image ..."
    if load ${devtype} ${devnum}:${distro_bootpart} ${kernel_addr_r} /Image; then
        setenv kernel_load yes
    else
        echo "[boot.cmd] try load ${devtype} ${devnum}:${distro_bootpart} ${kernel_addr_r} /Image-${uname_r} ..."
        if load ${devtype} ${devnum}:${distro_bootpart} ${kernel_addr_r} /Image-${uname_r}; then
            setenv kernel_load yes
        fi
    fi

    echo [boot.cmd] loading default rk-kernel.dtb
    load ${devtype} ${devnum}:${distro_bootpart} ${fdt_addr_r} /rk-kernel.dtb

    echo [boot.cmd] check the I2C bus
    i2c bus
    if test $? -eq 0; then
        setenv i2c_exist 1
        if test "${enable_gsdt_auto_load}" = "1" && test -n "${i2c_devs}"; then
            for dev in ${i2c_devs}; do
                i2c dev ${dev}
            done
        fi
    else
        setenv i2c_exist 0
    fi

    fdt addr  ${fdt_addr_r}
    fdt set /chosen bootargs

    echo [boot.cmd] dtoverlay from /uEnv/uEnv.txt
    setenv dev_bootpart ${devnum}:${distro_bootpart}
    dtfile ${fdt_addr_r} ${fdt_over_addr}  /uEnv/uEnv.txt ${env_addr_r}

    if test "${enable_gsdt_auto_load}" = "1" && test -n "${i2c_devs}" && test "${i2c_exist}" = "1"; then
        setenv i2c_idx 0
        echo [boot.cmd] screen eeprom reading test...
        for dev in ${i2c_devs}; do
            i2c dev ${dev}
            i2c md "${eeprom_addr}" 0 1
            if test $? -eq 0; then
                if test "${i2c_idx}" = "0"; then
                    setenv plugin_file "${gsdt_plugin0}"
                elif test "${i2c_idx}" = "1"; then
                    setenv plugin_file "${gsdt_plugin1}"
                fi

                if test -n "${plugin_file}"; then
                    echo [boot.cmd] load the generic screen plugin:${plugin_file}
                    load ${devtype} ${devnum}:${distro_bootpart} ${fdt_over_addr} ${plugin_file}
                    if test $? -eq 0; then
                        fdt apply ${fdt_over_addr}
                    fi
                fi
            fi
            if test "${i2c_idx}" = "0"; then
                setenv i2c_idx 1
            fi
        done
    fi

    echo [boot.cmd] [${devtype} ${devnum}:${distro_bootpart}] ...
    if test "${initrd_load}" = "yes"; then
        echo [boot.cmd] booti ${kernel_addr_r} ${ramdisk_addr_r}:${initrd_size} ${fdt_addr_r} ...
        booti ${kernel_addr_r} ${ramdisk_addr_r}:${initrd_size} ${fdt_addr_r}
    else
        echo [boot.cmd] booti ${kernel_addr_r} - ${fdt_addr_r} ...
        booti ${kernel_addr_r} - ${fdt_addr_r}
    fi
fi

echo [boot.cmd] run boot.cmd scripts failed ...;

# Recompile with:
# mkimage -C none -A arm -T script -d /boot/boot.cmd /boot/boot.scr
