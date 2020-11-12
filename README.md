#  ds34vita
- Combines functionality of [xerpi](https://github.com/xerpi "xerpi")'s [ds4vita](https://github.com/xerpi/ds4vita "ds4vita") and [ds3vita](https://github.com/xerpi/ds3vita "ds4vita").
- Alows to use up to two controllers at the same time.
- Has proper (PS TV way) buttons mapping.
- Fixed no-sleep bug
- Fixed multitouch issues
- Added deadzone for touchpanel (same as PS TV)
- Allows [reVita](https://github.com/MERLev/reVita "reVita") integration.

### Install
1. Copy **ds34vita.skprx** to *ur0:/tai/* folder
2. Add **ds34vita.skprx** to taiHEN's config (*ur0:/tai/config.txt*) under **KERNEL** section:
```
*KERNEL
ur0:tai/ds34vita.skprx
```

### Download: 
https://github.com/MERLev/ds34vita/releases

### Build
```bash
mkdir build
cd build
cmake ..
make
```

### Build as dependency
```bash
mkdir build
cd build
cmake ..
make install
```
**ds34vita-weak.yml** is used to create "weak" stubs for kernel.

### Credits
Based on [ds4vita](https://github.com/xerpi/ds4vita "ds4vita code") by [xerpi](https://github.com/xerpi "xerpi")\
Based on [ds3vita](https://github.com/xerpi/ds3vita "ds3vita code") by [xerpi](https://github.com/xerpi "xerpi")\
All testing done by [bosshunter](https://github.com/bosshunter)
