出现bug：
./opt_ceres: symbol lookup error: /lib/libceres.so.1: undefined symbol: _ZN6google21kLogSiteUninitializedE

解决办法：
conda deactivate
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH