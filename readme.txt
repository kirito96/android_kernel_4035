How to build this kernel open source
Steps for building

1.Untar xxx.tar.xz
Example:xz -d xxx.tar.xz
        tar zxf xxx.tar

2.prepare environment
export PATH=/yourCodebase/$toolchain:$PATH
Example:export PATH=/yourCodebase/prebuilts/gcc/linux-x86/arm/arm-eabi-4.7/bin:$PATH

3.build kernel
./makeMtk -t soul4 n k
