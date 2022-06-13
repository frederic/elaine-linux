# elaine-linux
Linux source tree for Google Nest Hub (2nd Gen) (elaine)

## setup
```
$ sudo apt install gcc-aarch64-linux-gnu
$ git clone -b android-tv-deadpool-4.9-android12 https://android.googlesource.com/kernel/amlogic-tv-modules/dhd-driver ../dhd-driver
```

## build
```
$ ./build_kernel.sh elaine-b4
```